/*
 * drm-game.c - Breakout Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background + border + score/level/lives HUD
 *   - Overlay plane:  Bricks, ball, effects (game objects)
 *   - Cursor plane:   Player paddle (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-game drm-game.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-game drm-game.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-game [/dev/dri/cardN]
 *
 * Controls:
 *   - Touchscreen: move paddle left/right
 *   - Arrow keys: move paddle left/right
 *   - SPACE: launch ball / unpause
 *   - P: pause
 *   - ESC or Q: quit
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
#include <sys/select.h>
#include <linux/input.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

/* ============================================================
 * Configuration
 * ============================================================ */

#define TARGET_FPS          60
#define FRAME_TIME_NS       (1000000000 / TARGET_FPS)

/* Number of framebuffers for primary and overlay planes (triple-buffering).
 * Triple-buffering allows the GPU/CPU to render ahead by one frame without
 * stalling on vsync, reducing dropped frames when rendering is uneven. */
#define NUM_BUFFERS         3

/* Overlay plane dimensions */
#define OVERLAY_W           1280
#define OVERLAY_H           720

/* Cursor plane dimensions (player paddle) */
#define CURSOR_W            128
#define CURSOR_H            64

#define PADDLE_WIDTH        80
#define PADDLE_HEIGHT       12
#define PADDLE_SPEED        8
#define PADDLE_MARGIN       30

#define BALL_SIZE           12
#define BALL_INITIAL_VX     3.5f
#define BALL_INITIAL_VY     -4.5f
#define BALL_SPEED_INC      1.03f
#define BALL_MAX_SPEED      12.0f

#define BRICK_ROWS          5
#define BRICK_COLS          10
#define BRICK_HEIGHT        16
#define BRICK_MARGIN        4
#define BRICK_TOP_OFFSET    50

#define COLOR_BG            0xFF0A0A2A
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_PADDLE        0xFFFFFFFF
#define COLOR_BALL          0xFFFFFF00
#define COLOR_SCORE         0xFF00FF80
#define COLOR_BORDER        0xFF333366
#define COLOR_TEXT          0xFFCCCCCC

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

/* Brick colors per row */
static const uint32_t brick_colors[BRICK_ROWS] = {
    0xFFFF4444, /* Red */
    0xFFFF8844, /* Orange */
    0xFFFFCC00, /* Yellow */
    0xFF44FF44, /* Green */
    0xFF4488FF, /* Blue */
};

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

    /* Atomic page-flip synchronization */
    bool pflip_pending;
};

struct ball {
    float x, y;
    float vx, vy;
};

struct paddle {
    int x, y;
    int width, height;
};

struct game_state {
    struct paddle paddle;
    struct ball ball;
    bool bricks[BRICK_ROWS][BRICK_COLS];
    int bricks_remaining;
    int score;
    int lives;
    int level;
    bool running;
    bool paused;
    bool ball_attached; /* ball sitting on paddle before launch */
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false;
static bool key_right = false;

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
    if (!conn) { drmModeFreeResources(res); return -1; }

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

    /* Create primary plane buffers (full screen, XRGB8888 - no alpha) */
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

    /* Create cursor plane buffer (player paddle, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at center for paddle) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (dev->width / 2) - (CURSOR_W / 2),
                        (dev->height - PADDLE_MARGIN) - (CURSOR_H / 2),
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

    printf("[DRM] Initialized (atomic, %d buffers): primary=%ux%u, overlay=%ux%u, cursor=%ux%u\n",
           NUM_BUFFERS, dev->width, dev->height, OVERLAY_W, OVERLAY_H, CURSOR_W, CURSOR_H);

    printf("\n");
    printf("[PLANE INFO] This game uses 3 DRM/KMS planes (atomic commit):\n");
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background + border + score/level/lives HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Bricks, ball, effects (game objects)\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player paddle (player-controlled)\n",
           dev->cursor_plane_id);
    printf("[PLANE INFO] All planes updated atomically per vblank - no flickering.\n");
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
        /* Disable overlay plane (set FB_ID=0, CRTC_ID=0) */
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.fb_id, 0);
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.crtc_id, 0);
        /* Disable cursor plane */
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

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
}

static void draw_rect(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                      int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);

    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride_px + i] = color;
}

static void draw_rect_outline(uint32_t *fb, uint32_t stride_px,
                              uint32_t scr_w, uint32_t scr_h,
                              int x, int y, int w, int h,
                              int thickness, uint32_t color) {
    draw_rect(fb, stride_px, scr_w, scr_h, x, y, w, thickness, color);             /* top */
    draw_rect(fb, stride_px, scr_w, scr_h, x, y + h - thickness, w, thickness, color); /* bottom */
    draw_rect(fb, stride_px, scr_w, scr_h, x, y, thickness, h, color);             /* left */
    draw_rect(fb, stride_px, scr_w, scr_h, x + w - thickness, y, thickness, h, color); /* right */
}

static void draw_circle(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= r2) {
                put_pixel(fb, stride_px, scr_w, scr_h, cx + dx, cy + dy, color);
            }
        }
    }
}

/* Simple 5x7 font for digits */
static const uint8_t font_5x7[][7] = {
    /* '0' */ {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* '1' */ {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    /* '2' */ {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    /* '3' */ {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    /* '4' */ {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    /* '5' */ {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    /* '6' */ {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    /* '7' */ {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    /* '8' */ {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* '9' */ {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
};

static void draw_char_digit(uint32_t *fb, uint32_t stride_px,
                            uint32_t scr_w, uint32_t scr_h,
                            int x, int y, int digit, int scale, uint32_t color) {
    if (digit < 0 || digit > 9) return;
    const uint8_t *glyph = font_5x7[digit];
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if (glyph[row] & (0x10 >> col)) {
                draw_rect(fb, stride_px, scr_w, scr_h,
                          x + col * scale, y + row * scale,
                          scale, scale, color);
            }
        }
    }
}

static void draw_number(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int x, int y, int number, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", number);
    int offset = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            draw_char_digit(fb, stride_px, scr_w, scr_h,
                           x + offset, y, buf[i] - '0', scale, color);
        }
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

    if (input_count == 0) {
        printf("[INPUT] WARNING: No input devices found. Use Ctrl+C to quit.\n");
    }
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) {
        close(input_fds[i]);
    }
    input_count = 0;
}

static void input_process(struct game_state *game, uint32_t screen_w) {
    struct input_event ev;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            /* Touch/absolute input */
            if (ev.type == EV_ABS) {
                if (ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
                    /* Map touch X coordinate to screen width */
                    /* Typical Microchip touch: 0-4095 range */
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0) {
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    }
                    game->paddle.x = (ev.value * (int)screen_w / max_val)
                                     - game->paddle.width / 2;
                }
            }

            /* Keyboard input */
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2); /* 1=press, 2=repeat */

                switch (ev.code) {
                case KEY_LEFT:
                    key_left = pressed;
                    break;
                case KEY_RIGHT:
                    key_right = pressed;
                    break;
                case KEY_SPACE:
                    if (ev.value == 1) {
                        if (game->ball_attached)
                            game->ball_attached = false;
                        else if (game->paused)
                            game->paused = false;
                    }
                    break;
                case KEY_P:
                    if (ev.value == 1)
                        game->paused = !game->paused;
                    break;
                case KEY_ESC:
                case KEY_Q:
                    if (ev.value == 1)
                        game->running = false;
                    break;
                default:
                    break;
                }
            }
        }
    }

    /* Apply keyboard paddle movement */
    if (key_left)
        game->paddle.x -= PADDLE_SPEED;
    if (key_right)
        game->paddle.x += PADDLE_SPEED;

    /* Clamp paddle position */
    if (game->paddle.x < 0)
        game->paddle.x = 0;
    if (game->paddle.x > (int)screen_w - game->paddle.width)
        game->paddle.x = (int)screen_w - game->paddle.width;
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void game_reset_bricks(struct game_state *game) {
    game->bricks_remaining = 0;
    for (int r = 0; r < BRICK_ROWS; r++) {
        for (int c = 0; c < BRICK_COLS; c++) {
            game->bricks[r][c] = true;
            game->bricks_remaining++;
        }
    }
}

static void game_reset_ball(struct game_state *game) {
    game->ball_attached = true;
    game->ball.vx = BALL_INITIAL_VX;
    game->ball.vy = BALL_INITIAL_VY;
}

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));

    game->paddle.width = PADDLE_WIDTH;
    game->paddle.height = PADDLE_HEIGHT;
    game->paddle.x = (int)screen_w / 2 - game->paddle.width / 2;
    game->paddle.y = (int)screen_h - PADDLE_MARGIN;

    game->ball.x = (float)screen_w / 2.0f;
    game->ball.y = (float)game->paddle.y - BALL_SIZE - 2;

    game->score = 0;
    game->lives = 3;
    game->level = 1;
    game->running = true;
    game->paused = false;

    game_reset_bricks(game);
    game_reset_ball(game);
}

static void game_update(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    if (game->paused || !game->running)
        return;

    /* If ball is attached to paddle, track paddle position */
    if (game->ball_attached) {
        game->ball.x = game->paddle.x + game->paddle.width / 2.0f - BALL_SIZE / 2.0f;
        game->ball.y = game->paddle.y - BALL_SIZE - 1;
        return;
    }

    /* Move ball */
    game->ball.x += game->ball.vx;
    game->ball.y += game->ball.vy;

    /* Wall collisions */
    if (game->ball.x <= 0) {
        game->ball.x = 0;
        game->ball.vx = -game->ball.vx;
    }
    if (game->ball.x >= (float)screen_w - BALL_SIZE) {
        game->ball.x = (float)screen_w - BALL_SIZE;
        game->ball.vx = -game->ball.vx;
    }
    if (game->ball.y <= 0) {
        game->ball.y = 0;
        game->ball.vy = -game->ball.vy;
    }

    /* Paddle collision */
    if (game->ball.vy > 0 &&
        game->ball.y + BALL_SIZE >= game->paddle.y &&
        game->ball.y + BALL_SIZE <= game->paddle.y + game->paddle.height + 4 &&
        game->ball.x + BALL_SIZE >= game->paddle.x &&
        game->ball.x <= game->paddle.x + game->paddle.width) {

        game->ball.vy = -fabsf(game->ball.vy);

        /* Angle based on where ball hits paddle */
        float hit_pos = (game->ball.x + BALL_SIZE / 2.0f - game->paddle.x)
                        / (float)game->paddle.width;
        game->ball.vx = (hit_pos - 0.5f) * 8.0f;

        /* Clamp speed */
        float speed = sqrtf(game->ball.vx * game->ball.vx + game->ball.vy * game->ball.vy);
        if (speed > BALL_MAX_SPEED) {
            game->ball.vx = game->ball.vx / speed * BALL_MAX_SPEED;
            game->ball.vy = game->ball.vy / speed * BALL_MAX_SPEED;
        }
    }

    /* Brick collisions */
    int brick_w = ((int)screen_w - BRICK_MARGIN * (BRICK_COLS + 1)) / BRICK_COLS;
    for (int r = 0; r < BRICK_ROWS; r++) {
        for (int c = 0; c < BRICK_COLS; c++) {
            if (!game->bricks[r][c])
                continue;

            int bx = BRICK_MARGIN + c * (brick_w + BRICK_MARGIN);
            int by = BRICK_TOP_OFFSET + r * (BRICK_HEIGHT + BRICK_MARGIN);

            /* Check collision */
            if (game->ball.x + BALL_SIZE > bx &&
                game->ball.x < bx + brick_w &&
                game->ball.y + BALL_SIZE > by &&
                game->ball.y < by + BRICK_HEIGHT) {

                game->bricks[r][c] = false;
                game->bricks_remaining--;
                game->score += (BRICK_ROWS - r) * 10;

                /* Determine bounce direction */
                float ball_cx = game->ball.x + BALL_SIZE / 2.0f;
                float ball_cy = game->ball.y + BALL_SIZE / 2.0f;
                float brick_cx = bx + brick_w / 2.0f;
                float brick_cy = by + BRICK_HEIGHT / 2.0f;

                float dx = ball_cx - brick_cx;
                float dy = ball_cy - brick_cy;

                if (fabsf(dx) / brick_w > fabsf(dy) / BRICK_HEIGHT)
                    game->ball.vx = -game->ball.vx;
                else
                    game->ball.vy = -game->ball.vy;

                /* Speed up slightly */
                game->ball.vx *= BALL_SPEED_INC;
                game->ball.vy *= BALL_SPEED_INC;

                goto brick_done; /* only one brick per frame */
            }
        }
    }
brick_done:

    /* Ball fell off bottom */
    if (game->ball.y > (float)screen_h) {
        game->lives--;
        if (game->lives <= 0) {
            /* Game over - reset */
            printf("[GAME] Game Over! Final Score: %d\n", game->score);
            game->score = 0;
            game->lives = 3;
            game->level = 1;
            game_reset_bricks(game);
        }
        game_reset_ball(game);
    }

    /* Level complete */
    if (game->bricks_remaining <= 0) {
        game->level++;
        printf("[GAME] Level %d complete! Score: %d\n", game->level - 1, game->score);
        game_reset_bricks(game);
        game_reset_ball(game);
        /* Increase difficulty */
        game->ball.vx = BALL_INITIAL_VX * (1.0f + game->level * 0.15f);
        game->ball.vy = BALL_INITIAL_VY * (1.0f + game->level * 0.15f);
    }
}

/* ============================================================
 * Rendering - Primary Plane (Background + Score/Level HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Clear background */
    clear_buffer(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Draw border */
    draw_rect_outline(fb, stride_px, w, h, 0, 0, (int)w, (int)h, 2, COLOR_BORDER);

    /* Draw HUD - Score */
    draw_number(fb, stride_px, w, h, 10, 10, game->score, 3, COLOR_SCORE);

    /* Draw HUD - Lives (small squares) */
    for (int i = 0; i < game->lives; i++) {
        draw_rect(fb, stride_px, w, h,
                  (int)w - 30 - i * 20, 10, 14, 14, 0xFFFF4444);
    }

    /* Draw HUD - Level */
    draw_number(fb, stride_px, w, h, (int)w / 2 - 15, 10, game->level, 2, COLOR_TEXT);

    /* "PAUSED" indicator */
    if (game->paused) {
        draw_rect(fb, stride_px, w, h,
                  (int)w / 2 - 40, (int)h / 2 - 10, 80, 20, 0xCC000000);
        /* Simple pause bars */
        draw_rect(fb, stride_px, w, h,
                  (int)w / 2 - 12, (int)h / 2 - 6, 8, 12, COLOR_TEXT);
        draw_rect(fb, stride_px, w, h,
                  (int)w / 2 + 4, (int)h / 2 - 6, 8, 12, COLOR_TEXT);
    }

    /* "Press SPACE" when ball is attached */
    if (game->ball_attached && !game->paused) {
        /* Blinking indicator */
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        if ((ts.tv_nsec / 500000000) % 2 == 0) {
            draw_rect(fb, stride_px, w, h,
                      (int)w / 2 - 4, (int)h / 2, 8, 8, COLOR_TEXT);
        }
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Bricks + Ball + Effects)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw bricks */
    int brick_w = ((int)w - BRICK_MARGIN * (BRICK_COLS + 1)) / BRICK_COLS;
    for (int r = 0; r < BRICK_ROWS; r++) {
        for (int c = 0; c < BRICK_COLS; c++) {
            if (!game->bricks[r][c])
                continue;

            int bx = BRICK_MARGIN + c * (brick_w + BRICK_MARGIN);
            int by = BRICK_TOP_OFFSET + r * (BRICK_HEIGHT + BRICK_MARGIN);

            draw_rect(fb, stride_px, w, h, bx, by, brick_w, BRICK_HEIGHT,
                      brick_colors[r]);
            /* Brick highlight (top edge) */
            draw_rect(fb, stride_px, w, h, bx, by, brick_w, 2,
                      brick_colors[r] | 0x00404040);
        }
    }

    /* Draw ball */
    int ball_cx = (int)game->ball.x + BALL_SIZE / 2;
    int ball_cy = (int)game->ball.y + BALL_SIZE / 2;
    draw_circle(fb, stride_px, w, h, ball_cx, ball_cy, BALL_SIZE / 2, COLOR_BALL);
}

/* ============================================================
 * Rendering - Cursor Plane (Player Paddle)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    /* Draw player paddle centered in cursor buffer */
    int px = ((int)w - PADDLE_WIDTH) / 2;
    int py = ((int)h - PADDLE_HEIGHT) / 2;

    /* Core paddle */
    draw_rect(fb, stride_px, w, h, px, py, PADDLE_WIDTH, PADDLE_HEIGHT, COLOR_PADDLE);
    /* Paddle shadow */
    draw_rect(fb, stride_px, w, h, px + 2, py + PADDLE_HEIGHT, PADDLE_WIDTH, 3, 0xC0000000);
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char *argv[]) {
    struct drm_device dev;
    struct game_state game;
    const char *card_path = (argc > 1) ? argv[1] : NULL;

    printf("===========================================\n");
    printf("  DRM Breakout Game - Microchip Demo\n");
    printf("  Built with libdrm (KMS/DRM - 3 Planes)\n");
    printf("===========================================\n\n");

    /* Setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM. Ensure:\n");
        fprintf(stderr, "  - No compositor is running (stop weston/X11)\n");
        fprintf(stderr, "  - /dev/dri/card0 exists\n");
        fprintf(stderr, "  - You have root permissions\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game */
    game_init(&game, dev.width, dev.height);

    /* Draw the player paddle sprite once (cursor plane content is static, just moves) */
    render_cursor(&dev.cursor_buf);

    printf("\n[GAME] Controls:\n");
    printf("  Arrow Keys / Touch : Move paddle\n");
    printf("  SPACE              : Launch ball / Unpause\n");
    printf("  P                  : Pause\n");
    printf("  ESC / Q            : Quit\n");
    printf("\n[GAME] Starting... (display: %ux%u)\n\n", dev.width, dev.height);

    /* Main game loop */
    uint64_t frame_start, frame_end, frame_elapsed;
    uint64_t fps_timer = get_time_ns();
    int frame_count = 0;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        /* Process input */
        input_process(&game, dev.width);

        /* Update game state */
        game_update(&game, dev.width, dev.height);

        /* Render primary plane (background + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (bricks + ball + effects) into back buffer */
        render_overlay(&dev, &game);

        /* Compute cursor plane position (player paddle movement) */
        int cursor_x = game.paddle.x - ((int)CURSOR_W - game.paddle.width) / 2;
        int cursor_y = game.paddle.y - ((int)CURSOR_H - game.paddle.height) / 2;

        /*
         * Wait for previous flip to complete before submitting a new one.
         * This ensures we don't write to a buffer that is still being
         * scanned out, and provides natural vsync-based frame pacing.
         */
        drm_wait_flip(&dev);

        /*
         * Submit a single atomic commit that updates ALL 3 planes
         * simultaneously at the next vblank - no flickering.
         */
        drm_atomic_flip(&dev, cursor_x, cursor_y);

        /* Frame timing - sleep only if rendering was faster than target */
        frame_end = get_time_ns();
        frame_elapsed = frame_end - frame_start;

        if (frame_elapsed < FRAME_TIME_NS)
            sleep_ns(FRAME_TIME_NS - frame_elapsed);

        /* FPS counter (print every 5 seconds) */
        frame_count++;
        if (get_time_ns() - fps_timer >= 5000000000ULL) {
            float fps = (float)frame_count / 5.0f;
            printf("[GAME] FPS: %.1f | Score: %d | Lives: %d | Level: %d\n",
                   fps, game.score, game.lives, game.level);
            frame_count = 0;
            fps_timer = get_time_ns();
        }
    }

    /* Cleanup */
    printf("\n[GAME] Shutting down...\n");
    printf("[GAME] Final Score: %d (Level %d)\n", game.score, game.level);

    input_cleanup();
    drm_cleanup(&dev);

    printf("[GAME] Goodbye!\n");
    return 0;
}
