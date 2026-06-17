/*
 * drm-snake.c - Classic Snake Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background grid + border + score/length HUD
 *   - Overlay plane:  Snake body segments, food items, particles/effects
 *   - Cursor plane:   Snake head (player-controlled direction)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm_snake drm-snake.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm_snake drm-snake.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm_snake [/dev/dri/cardN]
 *
 * Controls:
 *   - Arrow keys: change direction
 *   - Touchscreen: swipe to change direction
 *   - ESC or Q: quit
 *   - P: pause
 *   - SPACE: restart after game over
 *
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

/* ============================================================
 * Configuration
 * ============================================================ */

#define TARGET_FPS          30
#define FRAME_TIME_NS       (1000000000 / TARGET_FPS)
#define NUM_BUFFERS         3   /* Triple-buffering for flicker-free atomic flips */

/* Display resolution (primary plane) */
#define SCREEN_W            1280
#define SCREEN_H            720

/* Overlay plane dimensions */
#define OVERLAY_W           1280
#define OVERLAY_H           720

/* Cursor plane dimensions (snake head) */
#define CURSOR_W            64
#define CURSOR_H            64

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define GRID_SIZE           16      /* pixel size of each grid cell */
#define SNAKE_MAX_LEN       1024
#define INITIAL_SNAKE_LEN   5
#define GAME_SPEED_INITIAL  8       /* frames between moves */
#define GAME_SPEED_MIN      3       /* fastest speed */

#define COLOR_BG            0xFF001100
#define COLOR_SNAKE_HEAD    0xFF00FF00
#define COLOR_SNAKE_BODY    0xFF00CC00
#define COLOR_SNAKE_TAIL    0xFF009900
#define COLOR_FOOD          0xFFFF3333
#define COLOR_FOOD_GLOW     0xFFFF6666
#define COLOR_BORDER        0xFF004400
#define COLOR_GRID          0xFF001800
#define COLOR_TEXT          0xFFCCFFCC
#define COLOR_GAMEOVER      0xFFFF4444
#define COLOR_TRANSPARENT   0x00000000

/* ============================================================
 * Data Structures
 * ============================================================ */

struct drm_buffer {
    uint32_t handle;
    uint32_t fb_id;
    uint32_t *map;
    uint32_t size;
    uint32_t stride;
    uint32_t width;
    uint32_t height;
};

/* Cached property IDs for atomic modesetting */
struct plane_props {
    uint32_t fb_id;
    uint32_t crtc_id;
    uint32_t crtc_x;
    uint32_t crtc_y;
    uint32_t crtc_w;
    uint32_t crtc_h;
    uint32_t src_x;
    uint32_t src_y;
    uint32_t src_w;
    uint32_t src_h;
    uint32_t alpha;
};

struct crtc_props {
    uint32_t active;
    uint32_t mode_id;
};

struct conn_props {
    uint32_t crtc_id;
};

struct drm_device {
    int fd;
    uint32_t conn_id;
    uint32_t crtc_id;
    uint32_t crtc_idx;
    uint32_t width;
    uint32_t height;
    drmModeModeInfo mode;
    drmModeCrtc *saved_crtc;
    uint32_t mode_blob_id;

    uint32_t primary_plane_id;
    uint32_t overlay_plane_id;
    uint32_t cursor_plane_id;

    struct plane_props primary_props;
    struct plane_props overlay_props;
    struct plane_props cursor_props;
    struct crtc_props crtc_props;
    struct conn_props conn_props;

    struct drm_buffer primary_buf[NUM_BUFFERS];
    struct drm_buffer overlay_buf[NUM_BUFFERS];
    struct drm_buffer cursor_buf;
    int primary_front;
    int overlay_front;
    bool pflip_pending;  /* Atomic page-flip synchronization flag */
};

enum direction {
    DIR_UP = 0,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT
};

struct point {
    int x, y;
};

struct game_state {
    struct point snake[SNAKE_MAX_LEN];
    int snake_len;
    enum direction dir;
    enum direction next_dir;
    struct point food;
    int grid_w, grid_h;
    int score;
    int high_score;
    int speed;          /* frames between moves */
    int frame_counter;
    bool running;
    bool paused;
    bool game_over;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;

/* Touch tracking for swipe detection */
static int touch_start_x = -1;
static int touch_start_y = -1;

/* ============================================================
 * Signal Handler
 * ============================================================ */

static void signal_handler(int sig) {
    (void)sig;
    g_quit = true;
}

/* ============================================================
 * Timing Utilities
 * ============================================================ */

static uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void sleep_ns(uint64_t ns) {
    struct timespec ts;
    ts.tv_sec = ns / 1000000000ULL;
    ts.tv_nsec = ns % 1000000000ULL;
    nanosleep(&ts, NULL);
}

/* ============================================================
 * DRM Functions
 * ============================================================ */

static int drm_open_device(struct drm_device *dev, const char *path) {
    const char *cards[] = { path, "/dev/dri/card0", "/dev/dri/card1", NULL };
    int start = path ? 0 : 1;
    for (int i = start; cards[i]; i++) {
        dev->fd = open(cards[i], O_RDWR | O_CLOEXEC);
        if (dev->fd >= 0) {
            drmModeRes *res = drmModeGetResources(dev->fd);
            if (res) { drmModeFreeResources(res); printf("[DRM] Opened %s\n", cards[i]); return 0; }
            close(dev->fd); dev->fd = -1;
        }
    }
    fprintf(stderr, "[DRM] ERROR: Cannot open any DRM device\n");
    return -1;
}

static int drm_find_connector(struct drm_device *dev) {
    drmModeRes *res = drmModeGetResources(dev->fd);
    if (!res) return -1;

    drmModeConnector *conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(dev->fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0) break;
        if (conn) { drmModeFreeConnector(conn); conn = NULL; }
    }
    if (!conn) {
        fprintf(stderr, "[DRM] ERROR: No connected display found\n");
        drmModeFreeResources(res);
        return -1;
    }

    dev->conn_id = conn->connector_id;
    dev->mode = conn->modes[0];
    dev->width = dev->mode.hdisplay;
    dev->height = dev->mode.vdisplay;

    printf("[DRM] Connector %u: %ux%u @ %uHz\n",
           dev->conn_id, dev->width, dev->height, dev->mode.vrefresh);

    drmModeEncoder *enc = NULL;
    if (conn->encoder_id)
        enc = drmModeGetEncoder(dev->fd, conn->encoder_id);

    if (enc) {
        dev->crtc_id = enc->crtc_id;
        drmModeFreeEncoder(enc);
    } else {
        for (int i = 0; i < conn->count_encoders; i++) {
            enc = drmModeGetEncoder(dev->fd, conn->encoders[i]);
            if (enc) {
                for (int j = 0; j < res->count_crtcs; j++) {
                    if (enc->possible_crtcs & (1 << j)) {
                        dev->crtc_id = res->crtcs[j];
                        break;
                    }
                }
                drmModeFreeEncoder(enc);
                if (dev->crtc_id) break;
            }
        }
    }

    if (!dev->crtc_id) {
        fprintf(stderr, "[DRM] ERROR: No suitable CRTC found\n");
        drmModeFreeConnector(conn);
        drmModeFreeResources(res);
        return -1;
    }

    for (int i = 0; i < res->count_crtcs; i++) {
        if (res->crtcs[i] == dev->crtc_id) {
            dev->crtc_idx = i;
            break;
        }
    }

    dev->saved_crtc = drmModeGetCrtc(dev->fd, dev->crtc_id);
    printf("[DRM] Using CRTC %u (index %u)\n", dev->crtc_id, dev->crtc_idx);

    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    return 0;
}

static int drm_find_planes(struct drm_device *dev) {
    drmModePlaneRes *plane_res = drmModeGetPlaneResources(dev->fd);
    if (!plane_res) {
        fprintf(stderr, "[DRM] ERROR: drmModeGetPlaneResources failed.\n");
        return -1;
    }

    printf("[DRM] Found %u planes\n", plane_res->count_planes);

    for (uint32_t i = 0; i < plane_res->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(dev->fd, plane_res->planes[i]);
        if (!plane) continue;

        if (!(plane->possible_crtcs & (1 << dev->crtc_idx))) {
            drmModeFreePlane(plane);
            continue;
        }

        drmModeObjectProperties *props = drmModeObjectGetProperties(
            dev->fd, plane_res->planes[i], DRM_MODE_OBJECT_PLANE);
        if (!props) { drmModeFreePlane(plane); continue; }

        int plane_type = -1;
        for (uint32_t j = 0; j < props->count_props; j++) {
            drmModePropertyRes *prop = drmModeGetProperty(dev->fd, props->props[j]);
            if (!prop) continue;
            if (strcmp(prop->name, "type") == 0) {
                plane_type = (int)props->prop_values[j];
                drmModeFreeProperty(prop);
                break;
            }
            drmModeFreeProperty(prop);
        }
        drmModeFreeObjectProperties(props);

        switch (plane_type) {
        case 1: /* DRM_PLANE_TYPE_PRIMARY */
            if (!dev->primary_plane_id) {
                dev->primary_plane_id = plane_res->planes[i];
                printf("[DRM] Primary plane: %u\n", dev->primary_plane_id);
            }
            break;
        case 0: /* DRM_PLANE_TYPE_OVERLAY */
            if (!dev->overlay_plane_id) {
                dev->overlay_plane_id = plane_res->planes[i];
                printf("[DRM] Overlay plane: %u\n", dev->overlay_plane_id);
            }
            break;
        case 2: /* DRM_PLANE_TYPE_CURSOR */
            if (!dev->cursor_plane_id) {
                dev->cursor_plane_id = plane_res->planes[i];
                printf("[DRM] Cursor plane: %u\n", dev->cursor_plane_id);
            }
            break;
        }

        drmModeFreePlane(plane);
    }

    drmModeFreePlaneResources(plane_res);

    if (!dev->primary_plane_id) {
        fprintf(stderr, "[DRM] ERROR: No primary plane found\n");
        return -1;
    }
    if (!dev->overlay_plane_id) {
        fprintf(stderr, "[DRM] ERROR: No overlay plane found\n");
        return -1;
    }
    if (!dev->cursor_plane_id) {
        fprintf(stderr, "[DRM] ERROR: No cursor plane found\n");
        return -1;
    }

    return 0;
}

/* ---- Atomic property helpers ---- */

static uint32_t get_prop_id(int fd, uint32_t obj_id, uint32_t obj_type, const char *name) {
    drmModeObjectProperties *props = drmModeObjectGetProperties(fd, obj_id, obj_type);
    if (!props) return 0;

    uint32_t prop_id = 0;
    for (uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyRes *prop = drmModeGetProperty(fd, props->props[i]);
        if (!prop) continue;
        if (strcmp(prop->name, name) == 0) {
            prop_id = prop->prop_id;
            drmModeFreeProperty(prop);
            break;
        }
        drmModeFreeProperty(prop);
    }
    drmModeFreeObjectProperties(props);
    return prop_id;
}

static void cache_plane_props(int fd, uint32_t plane_id, struct plane_props *pp) {
    pp->fb_id   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
    pp->crtc_id = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
    pp->crtc_x  = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
    pp->crtc_y  = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
    pp->crtc_w  = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
    pp->crtc_h  = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");
    pp->src_x   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
    pp->src_y   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
    pp->src_w   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
    pp->src_h   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
    pp->alpha   = get_prop_id(fd, plane_id, DRM_MODE_OBJECT_PLANE, "alpha");
}

static void cache_crtc_props(int fd, uint32_t crtc_id, struct crtc_props *cp) {
    cp->active  = get_prop_id(fd, crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
    cp->mode_id = get_prop_id(fd, crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
}

static void cache_conn_props(int fd, uint32_t conn_id, struct conn_props *cp) {
    cp->crtc_id = get_prop_id(fd, conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
}

static int add_plane_to_atomic(drmModeAtomicReqPtr req, uint32_t plane_id,
                               struct plane_props *pp,
                               uint32_t fb_id, uint32_t crtc_id,
                               int32_t crtc_x, int32_t crtc_y,
                               uint32_t crtc_w, uint32_t crtc_h,
                               uint32_t src_x, uint32_t src_y,
                               uint32_t src_w, uint32_t src_h) {
    int ret = 0;
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->fb_id, fb_id);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->crtc_id, crtc_id);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->crtc_x, crtc_x);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->crtc_y, crtc_y);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->crtc_w, crtc_w);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->crtc_h, crtc_h);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->src_x, src_x);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->src_y, src_y);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->src_w, src_w);
    ret |= drmModeAtomicAddProperty(req, plane_id, pp->src_h, src_h);
    return ret < 0 ? -1 : 0;
}

/* Page-flip event handler callback */
static void page_flip_handler(int fd, unsigned int sequence,
                              unsigned int tv_sec, unsigned int tv_usec,
                              void *user_data) {
    (void)fd; (void)sequence; (void)tv_sec; (void)tv_usec;
    bool *pflip_pending = user_data;
    *pflip_pending = false;
}

static int drm_create_buffer(struct drm_device *dev, struct drm_buffer *buf,
                             uint32_t width, uint32_t height, uint32_t format) {
    struct drm_mode_create_dumb create_req = {0};
    struct drm_mode_map_dumb map_req = {0};

    create_req.width = width;
    create_req.height = height;
    create_req.bpp = 32;

    if (drmIoctl(dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_req) < 0) {
        perror("[DRM] DRM_IOCTL_MODE_CREATE_DUMB");
        return -1;
    }

    buf->handle = create_req.handle;
    buf->stride = create_req.pitch;
    buf->size = create_req.size;
    buf->width = width;
    buf->height = height;

    /* Use drmModeAddFB2 to explicitly specify the pixel format */
    uint32_t handles[4] = { buf->handle, 0, 0, 0 };
    uint32_t strides[4] = { buf->stride, 0, 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };

    if (drmModeAddFB2(dev->fd, width, height, format,
                      handles, strides, offsets, &buf->fb_id, 0) != 0) {
        perror("[DRM] drmModeAddFB2");
        return -1;
    }

    map_req.handle = buf->handle;
    if (drmIoctl(dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &map_req) < 0) {
        perror("[DRM] DRM_IOCTL_MODE_MAP_DUMB");
        return -1;
    }

    buf->map = mmap(NULL, buf->size, PROT_READ | PROT_WRITE,
                    MAP_SHARED, dev->fd, map_req.offset);
    if (buf->map == MAP_FAILED) {
        perror("[DRM] mmap");
        return -1;
    }

    memset(buf->map, 0, buf->size);
    return 0;
}

static void drm_destroy_buffer(struct drm_device *dev, struct drm_buffer *buf) {
    if (buf->map && buf->map != MAP_FAILED) {
        munmap(buf->map, buf->size);
        buf->map = NULL;
    }
    if (buf->fb_id) {
        drmModeRmFB(dev->fd, buf->fb_id);
        buf->fb_id = 0;
    }
    if (buf->handle) {
        struct drm_mode_destroy_dumb destroy = { .handle = buf->handle };
        drmIoctl(dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        buf->handle = 0;
    }
}

static int drm_init(struct drm_device *dev, const char *card_path) {
    memset(dev, 0, sizeof(*dev));
    dev->fd = -1;

    if (drm_open_device(dev, card_path) < 0)
        return -1;

    /* Enable universal planes so we can enumerate overlay/cursor */
    if (drmSetClientCap(dev->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0) {
        fprintf(stderr, "[DRM] WARNING: Cannot set UNIVERSAL_PLANES cap\n");
    }

    /* Enable atomic modesetting - required for flicker-free multi-plane updates */
    if (drmSetClientCap(dev->fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
        fprintf(stderr, "[DRM] ERROR: Cannot set ATOMIC cap - kernel driver may not support it\n");
        return -1;
    }
    printf("[DRM] Atomic modesetting enabled\n");

    if (drm_find_connector(dev) < 0)
        return -1;

    if (drm_find_planes(dev) < 0)
        return -1;

    /* Cache property IDs for atomic commits */
    cache_plane_props(dev->fd, dev->primary_plane_id, &dev->primary_props);
    cache_plane_props(dev->fd, dev->overlay_plane_id, &dev->overlay_props);
    cache_plane_props(dev->fd, dev->cursor_plane_id, &dev->cursor_props);
    cache_crtc_props(dev->fd, dev->crtc_id, &dev->crtc_props);
    cache_conn_props(dev->fd, dev->conn_id, &dev->conn_props);

    /* Create a mode blob for atomic modeset */
    if (drmModeCreatePropertyBlob(dev->fd, &dev->mode, sizeof(dev->mode),
                                  &dev->mode_blob_id) != 0) {
        perror("[DRM] drmModeCreatePropertyBlob");
        return -1;
    }

    /* Create primary plane triple buffers (full screen, XRGB8888 - no alpha) */
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (drm_create_buffer(dev, &dev->primary_buf[i], dev->width, dev->height,
                              DRM_FORMAT_XRGB8888) < 0) {
            fprintf(stderr, "[DRM] ERROR: Failed to create primary buffer %d\n", i);
            return -1;
        }
    }
    dev->primary_front = 0;

    /* Create overlay plane buffers (ARGB8888 for per-pixel alpha transparency) */
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (drm_create_buffer(dev, &dev->overlay_buf[i], OVERLAY_W, OVERLAY_H,
                              DRM_FORMAT_ARGB8888) < 0) {
            fprintf(stderr, "[DRM] ERROR: Failed to create overlay buffer %d\n", i);
            return -1;
        }
    }
    dev->overlay_front = 0;

    /* Create cursor plane buffer (snake head, ARGB8888 for per-pixel alpha) */
    if (drm_create_buffer(dev, &dev->cursor_buf, CURSOR_W, CURSOR_H,
                          DRM_FORMAT_ARGB8888) < 0) {
        fprintf(stderr, "[DRM] ERROR: Failed to create cursor buffer\n");
        return -1;
    }

    /*
     * Perform initial atomic modeset to set up all planes.
     *
     * This single atomic commit:
     *   - Sets the display mode on the CRTC
     *   - Assigns framebuffers to all 3 planes
     *   - Sets alpha=255 on overlay/cursor for per-pixel alpha blending
     *
     * mpfs-dpsub driver alpha logic (mpfs_kms.c):
     *   if (alpha != 0 && alpha != 255) -> enable global alpha blending
     *   else -> disable global alpha (plane uses per-pixel alpha from ARGB)
     */
    drmModeAtomicReqPtr req = drmModeAtomicAlloc();
    if (!req) {
        fprintf(stderr, "[DRM] ERROR: drmModeAtomicAlloc failed\n");
        return -1;
    }

    /* CRTC: set mode */
    drmModeAtomicAddProperty(req, dev->crtc_id, dev->crtc_props.active, 1);
    drmModeAtomicAddProperty(req, dev->crtc_id, dev->crtc_props.mode_id, dev->mode_blob_id);

    /* Connector: bind to CRTC */
    drmModeAtomicAddProperty(req, dev->conn_id, dev->conn_props.crtc_id, dev->crtc_id);

    /* Primary plane */
    add_plane_to_atomic(req, dev->primary_plane_id, &dev->primary_props,
                        dev->primary_buf[0].fb_id, dev->crtc_id,
                        0, 0, dev->width, dev->height,
                        0, 0, dev->width << 16, dev->height << 16);

    /* Overlay plane (scaled from OVERLAY_W x OVERLAY_H to full screen) */
    add_plane_to_atomic(req, dev->overlay_plane_id, &dev->overlay_props,
                        dev->overlay_buf[0].fb_id, dev->crtc_id,
                        0, 0, dev->width, dev->height,
                        0, 0, OVERLAY_W << 16, OVERLAY_H << 16);
    if (dev->overlay_props.alpha)
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.alpha, OVERLAY_ALPHA);

    /* Cursor plane */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (dev->width / 2) - (CURSOR_W / 2),
                        (dev->height / 2) - (CURSOR_H / 2),
                        CURSOR_W, CURSOR_H,
                        0, 0, CURSOR_W << 16, CURSOR_H << 16);
    if (dev->cursor_props.alpha)
        drmModeAtomicAddProperty(req, dev->cursor_plane_id,
                                 dev->cursor_props.alpha, CURSOR_ALPHA);

    /* Commit with ALLOW_MODESET for initial setup (blocking) */
    int ret = drmModeAtomicCommit(dev->fd, req,
                                  DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
    drmModeAtomicFree(req);

    if (ret != 0) {
        perror("[DRM] Initial atomic commit failed");
        return -1;
    }

    dev->pflip_pending = false;

    printf("[DRM] Initialized: primary=%ux%u, overlay=%ux%u, cursor=%ux%u\n",
           dev->width, dev->height, OVERLAY_W, OVERLAY_H, CURSOR_W, CURSOR_H);

    printf("\n");
    printf("[PLANE INFO] This game uses 3 DRM/KMS planes:\n");
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background grid + border + score/length HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Snake body, food items, particles/effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Snake head (player-controlled)\n",
           dev->cursor_plane_id);
    printf("\n");

    return 0;
}

/*
 * drm_atomic_flip - Submit a single atomic commit for all 3 planes.
 *
 * This updates primary + overlay framebuffers and cursor position atomically
 * in one vblank, eliminating inter-plane flickering and tearing.
 */
static int drm_atomic_flip(struct drm_device *dev, int cursor_x, int cursor_y) {
    drmModeAtomicReqPtr req = drmModeAtomicAlloc();
    if (!req) return -1;

    int primary_back = (dev->primary_front + 1) % NUM_BUFFERS;
    int overlay_back = (dev->overlay_front + 1) % NUM_BUFFERS;

    /* Primary plane: swap to back buffer */
    drmModeAtomicAddProperty(req, dev->primary_plane_id,
                             dev->primary_props.fb_id,
                             dev->primary_buf[primary_back].fb_id);

    /* Overlay plane: swap to back buffer */
    drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                             dev->overlay_props.fb_id,
                             dev->overlay_buf[overlay_back].fb_id);

    /* Cursor plane: update position */
    drmModeAtomicAddProperty(req, dev->cursor_plane_id,
                             dev->cursor_props.crtc_x, cursor_x);
    drmModeAtomicAddProperty(req, dev->cursor_plane_id,
                             dev->cursor_props.crtc_y, cursor_y);

    /* Non-blocking commit with page-flip event for vsync */
    int ret = drmModeAtomicCommit(dev->fd, req,
                                  DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_ATOMIC_NONBLOCK,
                                  &dev->pflip_pending);
    drmModeAtomicFree(req);

    if (ret == 0) {
        dev->pflip_pending = true;
        dev->primary_front = primary_back;
        dev->overlay_front = overlay_back;
    } else if (errno == EBUSY) {
        /* Previous flip not yet complete - skip this frame */
        return 0;
    } else {
        perror("[DRM] Atomic commit failed");
    }

    return ret;
}

/* Wait for the page-flip event (vsync synchronization) */
static void drm_wait_flip(struct drm_device *dev) {
    if (!dev->pflip_pending)
        return;

    struct pollfd pfd = {
        .fd = dev->fd,
        .events = POLLIN,
    };

    drmEventContext ev_ctx = {
        .version = 2,
        .page_flip_handler = page_flip_handler,
    };

    while (dev->pflip_pending) {
        int ret = poll(&pfd, 1, 100); /* 100ms timeout */
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            perror("[DRM] poll");
            break;
        }
        if (ret == 0) {
            /* Timeout - shouldn't happen normally */
            break;
        }
        if (pfd.revents & POLLIN) {
            drmHandleEvent(dev->fd, &ev_ctx);
        }
    }
}

static void drm_cleanup(struct drm_device *dev) {
    /* Wait for any pending flip to complete before teardown */
    drm_wait_flip(dev);

    /* Disable overlay and cursor planes atomically */
    drmModeAtomicReqPtr req = drmModeAtomicAlloc();
    if (req) {
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.fb_id, 0);
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.crtc_id, 0);
        drmModeAtomicAddProperty(req, dev->cursor_plane_id,
                                 dev->cursor_props.fb_id, 0);
        drmModeAtomicAddProperty(req, dev->cursor_plane_id,
                                 dev->cursor_props.crtc_id, 0);
        drmModeAtomicCommit(dev->fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
        drmModeAtomicFree(req);
    }

    /* Restore previous CRTC */
    if (dev->saved_crtc) {
        drmModeSetCrtc(dev->fd, dev->saved_crtc->crtc_id,
                       dev->saved_crtc->buffer_id,
                       dev->saved_crtc->x, dev->saved_crtc->y,
                       &dev->conn_id, 1, &dev->saved_crtc->mode);
        drmModeFreeCrtc(dev->saved_crtc);
    }

    /* Destroy mode blob */
    if (dev->mode_blob_id)
        drmModeDestroyPropertyBlob(dev->fd, dev->mode_blob_id);

    for (int i = 0; i < NUM_BUFFERS; i++)
        drm_destroy_buffer(dev, &dev->primary_buf[i]);
    for (int i = 0; i < NUM_BUFFERS; i++)
        drm_destroy_buffer(dev, &dev->overlay_buf[i]);
    drm_destroy_buffer(dev, &dev->cursor_buf);

    if (dev->fd >= 0)
        close(dev->fd);

    printf("[DRM] Cleaned up\n");
}

/* ============================================================
 * Drawing Primitives
 * ============================================================ */

static inline void put_pixel(uint32_t *fb, uint32_t stride_px,
                             uint32_t w, uint32_t h,
                             int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h)
        fb[y * stride_px + x] = color;
}

static void draw_fill(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
}

static void draw_rect(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                       int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);

    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = color;
}

static void draw_rect_outline(uint32_t *fb, uint32_t stride,
                               uint32_t scr_w, uint32_t scr_h,
                               int x, int y, int w, int h,
                               int thickness, uint32_t color) {
    draw_rect(fb, stride, scr_w, scr_h, x, y, w, thickness, color);
    draw_rect(fb, stride, scr_w, scr_h, x, y + h - thickness, w, thickness, color);
    draw_rect(fb, stride, scr_w, scr_h, x, y, thickness, h, color);
    draw_rect(fb, stride, scr_w, scr_h, x + w - thickness, y, thickness, h, color);
}

static void draw_circle(uint32_t *fb, uint32_t stride,
                         uint32_t scr_w, uint32_t scr_h,
                         int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= r2) {
                int px = cx + dx, py = cy + dy;
                if (px >= 0 && px < (int)scr_w && py >= 0 && py < (int)scr_h)
                    fb[py * stride + px] = color;
            }
        }
    }
}

/* Simple 5x7 font for digits */
static const uint8_t font_5x7[][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, /* 0 */
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* 1 */
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, /* 2 */
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}, /* 3 */
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, /* 4 */
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, /* 5 */
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, /* 6 */
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* 7 */
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, /* 8 */
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, /* 9 */
};

static void draw_digit(uint32_t *fb, uint32_t stride,
                        uint32_t scr_w, uint32_t scr_h,
                        int x, int y, int digit, int scale, uint32_t color) {
    if (digit < 0 || digit > 9) return;
    const uint8_t *glyph = font_5x7[digit];
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if (glyph[row] & (0x10 >> col)) {
                draw_rect(fb, stride, scr_w, scr_h,
                          x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

static void draw_number(uint32_t *fb, uint32_t stride,
                         uint32_t scr_w, uint32_t scr_h,
                         int x, int y, int number, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", number);
    int offset = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride, scr_w, scr_h, x + offset, y, buf[i] - '0', scale, color);
        offset += 6 * scale;
    }
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64];
    input_count = 0;

    for (int i = 0; i < 8 && input_count < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("[INPUT] Opened %s: %s\n", path, name);
            input_fds[input_count++] = fd;
        }
    }
    if (input_count == 0)
        printf("[INPUT] WARNING: No input devices found.\n");
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++)
        close(input_fds[i]);
    input_count = 0;
}

static void input_process(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    struct input_event ev;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            /* Touch/absolute input for swipe detection */
            if (ev.type == EV_ABS) {
                if (ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0)
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    int screen_x = ev.value * (int)screen_w / max_val;

                    if (touch_start_x < 0) {
                        touch_start_x = screen_x;
                    } else {
                        int dx = screen_x - touch_start_x;
                        if (abs(dx) > 30) {
                            if (dx > 0 && game->dir != DIR_LEFT)
                                game->next_dir = DIR_RIGHT;
                            else if (dx < 0 && game->dir != DIR_RIGHT)
                                game->next_dir = DIR_LEFT;
                            touch_start_x = screen_x;
                        }
                    }
                }
                if (ev.code == ABS_Y || ev.code == ABS_MT_POSITION_Y) {
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0)
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    int screen_y = ev.value * (int)screen_h / max_val;

                    if (touch_start_y < 0) {
                        touch_start_y = screen_y;
                    } else {
                        int dy = screen_y - touch_start_y;
                        if (abs(dy) > 30) {
                            if (dy > 0 && game->dir != DIR_UP)
                                game->next_dir = DIR_DOWN;
                            else if (dy < 0 && game->dir != DIR_DOWN)
                                game->next_dir = DIR_UP;
                            touch_start_y = screen_y;
                        }
                    }
                }
            }

            /* Reset touch on release */
            if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) {
                touch_start_x = -1;
                touch_start_y = -1;
            }

            /* Keyboard input */
            if (ev.type == EV_KEY && ev.value == 1) {
                switch (ev.code) {
                case KEY_UP:
                    if (game->dir != DIR_DOWN) game->next_dir = DIR_UP;
                    break;
                case KEY_DOWN:
                    if (game->dir != DIR_UP) game->next_dir = DIR_DOWN;
                    break;
                case KEY_LEFT:
                    if (game->dir != DIR_RIGHT) game->next_dir = DIR_LEFT;
                    break;
                case KEY_RIGHT:
                    if (game->dir != DIR_LEFT) game->next_dir = DIR_RIGHT;
                    break;
                case KEY_SPACE:
                    if (game->game_over) {
                        game->game_over = false;
                        game->score = 0;
                        game->snake_len = INITIAL_SNAKE_LEN;
                        game->speed = GAME_SPEED_INITIAL;
                        game->dir = DIR_RIGHT;
                        game->next_dir = DIR_RIGHT;
                        /* Reset snake to center */
                        for (int s = 0; s < game->snake_len; s++) {
                            game->snake[s].x = game->grid_w / 2 - s;
                            game->snake[s].y = game->grid_h / 2;
                        }
                    } else if (game->paused) {
                        game->paused = false;
                    }
                    break;
                case KEY_P:
                    if (!game->game_over)
                        game->paused = !game->paused;
                    break;
                case KEY_ESC:
                case KEY_Q:
                    game->running = false;
                    break;
                default:
                    break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void place_food(struct game_state *game) {
    bool valid;
    do {
        valid = true;
        game->food.x = rand() % game->grid_w;
        game->food.y = rand() % game->grid_h;
        /* Make sure food doesn't overlap with snake */
        for (int i = 0; i < game->snake_len; i++) {
            if (game->snake[i].x == game->food.x &&
                game->snake[i].y == game->food.y) {
                valid = false;
                break;
            }
        }
    } while (!valid);
}

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));

    /* Calculate grid dimensions (leave border) */
    game->grid_w = (int)screen_w / GRID_SIZE - 2;
    game->grid_h = (int)screen_h / GRID_SIZE - 2;

    /* Initialize snake at center */
    game->snake_len = INITIAL_SNAKE_LEN;
    for (int i = 0; i < game->snake_len; i++) {
        game->snake[i].x = game->grid_w / 2 - i;
        game->snake[i].y = game->grid_h / 2;
    }

    game->dir = DIR_RIGHT;
    game->next_dir = DIR_RIGHT;
    game->speed = GAME_SPEED_INITIAL;
    game->frame_counter = 0;
    game->score = 0;
    game->high_score = 0;
    game->running = true;
    game->paused = false;
    game->game_over = false;

    srand((unsigned)time(NULL));
    place_food(game);
}

static void game_update(struct game_state *game) {
    if (game->paused || game->game_over)
        return;

    game->frame_counter++;
    if (game->frame_counter < game->speed)
        return;
    game->frame_counter = 0;

    /* Apply direction change */
    game->dir = game->next_dir;

    /* Calculate new head position */
    struct point new_head = game->snake[0];
    switch (game->dir) {
    case DIR_UP:    new_head.y--; break;
    case DIR_DOWN:  new_head.y++; break;
    case DIR_LEFT:  new_head.x--; break;
    case DIR_RIGHT: new_head.x++; break;
    }

    /* Check wall collision */
    if (new_head.x < 0 || new_head.x >= game->grid_w ||
        new_head.y < 0 || new_head.y >= game->grid_h) {
        game->game_over = true;
        if (game->score > game->high_score)
            game->high_score = game->score;
        printf("[GAME] Game Over! Score: %d | High Score: %d\n",
               game->score, game->high_score);
        return;
    }

    /* Check self-collision */
    for (int i = 0; i < game->snake_len; i++) {
        if (game->snake[i].x == new_head.x && game->snake[i].y == new_head.y) {
            game->game_over = true;
            if (game->score > game->high_score)
                game->high_score = game->score;
            printf("[GAME] Game Over! Score: %d | High Score: %d\n",
                   game->score, game->high_score);
            return;
        }
    }

    /* Check food collision */
    bool ate_food = (new_head.x == game->food.x && new_head.y == game->food.y);

    /* Move snake: shift body, place new head */
    if (!ate_food) {
        /* Remove tail */
        for (int i = game->snake_len - 1; i > 0; i--)
            game->snake[i] = game->snake[i - 1];
    } else {
        /* Grow: don't remove tail, add space for new head */
        if (game->snake_len < SNAKE_MAX_LEN) {
            for (int i = game->snake_len; i > 0; i--)
                game->snake[i] = game->snake[i - 1];
            game->snake_len++;
        } else {
            for (int i = game->snake_len - 1; i > 0; i--)
                game->snake[i] = game->snake[i - 1];
        }
        game->score += 10;

        /* Speed up */
        if (game->speed > GAME_SPEED_MIN && game->score % 50 == 0)
            game->speed--;

        place_food(game);
    }

    game->snake[0] = new_head;
}

/* ============================================================
 * Rendering - Primary Plane (Background grid + border + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Clear */
    draw_fill(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Grid offset (centered with border) */
    int offset_x = GRID_SIZE;
    int offset_y = GRID_SIZE;
    int grid_pixel_w = game->grid_w * GRID_SIZE;
    int grid_pixel_h = game->grid_h * GRID_SIZE;

    /* Draw grid lines (subtle) */
    for (int gx = 0; gx <= game->grid_w; gx++) {
        int px = offset_x + gx * GRID_SIZE;
        for (int py = offset_y; py < offset_y + grid_pixel_h; py++) {
            if (px >= 0 && px < (int)w && py >= 0 && py < (int)h)
                fb[py * stride + px] = COLOR_GRID;
        }
    }
    for (int gy = 0; gy <= game->grid_h; gy++) {
        int py = offset_y + gy * GRID_SIZE;
        for (int px = offset_x; px < offset_x + grid_pixel_w; px++) {
            if (px >= 0 && px < (int)w && py >= 0 && py < (int)h)
                fb[py * stride + px] = COLOR_GRID;
        }
    }

    /* Draw border */
    draw_rect_outline(fb, stride, w, h,
                      offset_x - 2, offset_y - 2,
                      grid_pixel_w + 4, grid_pixel_h + 4,
                      2, COLOR_BORDER);

    /* Draw HUD - score */
    draw_number(fb, stride, w, h, 10, (int)h - 30, game->score, 3, COLOR_TEXT);

    /* High score on the right */
    draw_number(fb, stride, w, h, (int)w - 100, (int)h - 30, game->high_score, 2, 0xFF668866);

    /* Snake length indicator */
    draw_number(fb, stride, w, h, (int)w / 2 - 20, (int)h - 30, game->snake_len, 2, 0xFF448844);

    /* Game Over overlay */
    if (game->game_over) {
        /* Darken center area */
        draw_rect(fb, stride, w, h,
                  (int)w / 4, (int)h / 2 - 30,
                  (int)w / 2, 60, 0xCC000000);
        /* Red X marks */
        int cx = (int)w / 2;
        int cy = (int)h / 2;
        draw_rect(fb, stride, w, h, cx - 20, cy - 4, 40, 8, COLOR_GAMEOVER);
        draw_rect(fb, stride, w, h, cx - 4, cy - 20, 8, 40, COLOR_GAMEOVER);
        /* Score below */
        draw_number(fb, stride, w, h, cx - 30, cy + 25, game->score, 2, COLOR_TEXT);
    }

    /* Paused indicator */
    if (game->paused && !game->game_over) {
        draw_rect(fb, stride, w, h,
                  (int)w / 2 - 30, (int)h / 2 - 10, 60, 20, 0xCC000000);
        draw_rect(fb, stride, w, h,
                  (int)w / 2 - 12, (int)h / 2 - 6, 8, 12, COLOR_TEXT);
        draw_rect(fb, stride, w, h,
                  (int)w / 2 + 4, (int)h / 2 - 6, 8, 12, COLOR_TEXT);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Snake body + food + particles)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    draw_fill(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Grid offset (matches primary plane) */
    int offset_x = GRID_SIZE;
    int offset_y = GRID_SIZE;

    /* Draw food (pulsing circle) */
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        int pulse = (ts.tv_nsec / 100000000) % 3;
        int fx = offset_x + game->food.x * GRID_SIZE + GRID_SIZE / 2;
        int fy = offset_y + game->food.y * GRID_SIZE + GRID_SIZE / 2;
        draw_circle(fb, stride, w, h, fx, fy, GRID_SIZE / 2 - 1 + pulse, COLOR_FOOD_GLOW);
        draw_circle(fb, stride, w, h, fx, fy, GRID_SIZE / 2 - 2, COLOR_FOOD);
    }

    /* Draw snake body (skip head at index 0 - rendered on cursor plane) */
    for (int i = 1; i < game->snake_len; i++) {
        int sx = offset_x + game->snake[i].x * GRID_SIZE + 1;
        int sy = offset_y + game->snake[i].y * GRID_SIZE + 1;
        uint32_t color;

        if (i < game->snake_len / 2)
            color = COLOR_SNAKE_BODY;
        else
            color = COLOR_SNAKE_TAIL;

        draw_rect(fb, stride, w, h, sx, sy, GRID_SIZE - 2, GRID_SIZE - 2, color);
    }

    /* Draw particle effects at food location when eating (glow effect) */
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        int frame = (int)(ts.tv_nsec / 50000000) % 8;
        /* Subtle sparkle around food */
        int fx = offset_x + game->food.x * GRID_SIZE + GRID_SIZE / 2;
        int fy = offset_y + game->food.y * GRID_SIZE + GRID_SIZE / 2;
        int sparkle_offsets[] = { -8, -6, 6, 8 };
        for (int s = 0; s < 4; s++) {
            int sx = fx + sparkle_offsets[(s + frame) % 4];
            int sy = fy + sparkle_offsets[(s + frame + 1) % 4];
            if (sx >= 0 && sx < (int)w && sy >= 0 && sy < (int)h)
                put_pixel(fb, stride, w, h, sx, sy, 0xAAFFFF44);
        }
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Snake head)
 * ============================================================ */

static void render_cursor(struct drm_device *dev, struct game_state *game) {
    uint32_t *fb = dev->cursor_buf.map;
    uint32_t stride = dev->cursor_buf.stride / 4;
    uint32_t w = CURSOR_W;
    uint32_t h = CURSOR_H;

    /* Clear to transparent */
    draw_fill(fb, dev->cursor_buf.size / 4, COLOR_TRANSPARENT);

    /* Draw snake head centered in cursor buffer */
    int hx = (int)(w - GRID_SIZE) / 2 + 1;
    int hy = (int)(h - GRID_SIZE) / 2 + 1;
    int head_size = GRID_SIZE - 2;

    /* Head body */
    draw_rect(fb, stride, w, h, hx, hy, head_size, head_size, COLOR_SNAKE_HEAD);

    /* Eyes */
    int ex1 = hx + 3, ex2 = hx + head_size - 6;
    int ey = hy + 4;
    switch (game->dir) {
    case DIR_UP:    ey = hy + 2; break;
    case DIR_DOWN:  ey = hy + head_size - 6; break;
    case DIR_LEFT:  ex1 = hx + 2; ex2 = hx + head_size / 2 - 1; break;
    case DIR_RIGHT: ex1 = hx + head_size / 2; ex2 = hx + head_size - 4; break;
    }
    draw_rect(fb, stride, w, h, ex1, ey, 3, 3, 0xFF000000);
    draw_rect(fb, stride, w, h, ex2, ey, 3, 3, 0xFF000000);
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char *argv[]) {
    struct drm_device dev;
    struct game_state game;
    const char *card_path = (argc > 1) ? argv[1] : NULL;

    printf("===========================================\n");
    printf("  DRM Snake Game - Microchip Demo\n");
    printf("  Built with libdrm (KMS/DRM, 3 planes)\n");
    printf("===========================================\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM.\n");
        return 1;
    }

    input_init();
    game_init(&game, dev.width, dev.height);

    printf("[GAME] Controls:\n");
    printf("  Arrow Keys / Swipe : Change direction\n");
    printf("  SPACE              : Restart / Unpause\n");
    printf("  P                  : Pause\n");
    printf("  ESC / Q            : Quit\n");
    printf("\n[GAME] Grid: %dx%d | Starting...\n\n", game.grid_w, game.grid_h);

    uint64_t frame_start, frame_end, frame_elapsed;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        input_process(&game, dev.width, dev.height);
        game_update(&game);

        /* Render all planes into their back buffers */
        render_primary(&dev, &game);
        render_overlay(&dev, &game);
        render_cursor(&dev, &game);

        /* Compute cursor position (snake head) */
        int offset_x = GRID_SIZE;
        int offset_y = GRID_SIZE;
        int cursor_x = offset_x + game.snake[0].x * GRID_SIZE + GRID_SIZE / 2 - CURSOR_W / 2;
        int cursor_y = offset_y + game.snake[0].y * GRID_SIZE + GRID_SIZE / 2 - CURSOR_H / 2;

        /* Wait for previous flip, then submit atomic commit for all planes */
        drm_wait_flip(&dev);
        drm_atomic_flip(&dev, cursor_x, cursor_y);

        frame_end = get_time_ns();
        frame_elapsed = frame_end - frame_start;
        if (frame_elapsed < FRAME_TIME_NS)
            sleep_ns(FRAME_TIME_NS - frame_elapsed);
    }

    printf("\n[GAME] Final Score: %d | High Score: %d\n", game.score, game.high_score);
    input_cleanup();
    drm_cleanup(&dev);
    printf("[GAME] Goodbye!\n");
    return 0;
}
