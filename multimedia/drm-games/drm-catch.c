/*
 * drm-catch.c - Catch/Dodge Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Static/scrolling background + score/lives HUD
 *   - Overlay plane:  Falling objects (fruits to catch, bombs to dodge)
 *   - Cursor plane:   Player-controlled basket/paddle
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-catch drm-catch.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-catch drm-catch.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-catch [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right arrow keys: move basket
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

/* Display resolution (primary plane) */
#define SCREEN_W            1280
#define SCREEN_H            720

/* Overlay plane dimensions (falling objects layer) */
#define OVERLAY_W           1280
#define OVERLAY_H           720

/* Cursor plane dimensions (player basket) */
#define CURSOR_W            128
#define CURSOR_H            128

/* Game tuning */
#define BASKET_SPEED        8
#define MAX_FALLING_OBJS    12
#define FALL_SPEED_MIN      2.0f
#define FALL_SPEED_MAX      6.0f
#define SPAWN_INTERVAL_MS   800
#define OBJ_SIZE            32

#define MAX_LIVES           5
#define FRUIT_SCORE         10
#define BOMB_PENALTY        1  /* lose a life */

/* Colors (ARGB8888 / XRGB8888) */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG_TOP        0xFF001030
#define COLOR_BG_BOT        0xFF000818
#define COLOR_BORDER        0xFF224488
#define COLOR_STAR          0xFFFFFFDD
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE_VAL     0xFF00FF80
#define COLOR_LIVES_VAL     0xFFFF4444

#define COLOR_FRUIT_APPLE   0xFFFF2222
#define COLOR_FRUIT_ORANGE  0xFFFF8800
#define COLOR_FRUIT_GRAPE   0xFF8844FF
#define COLOR_FRUIT_BANANA  0xFFFFDD00
#define COLOR_BOMB          0xFF333333
#define COLOR_BOMB_FUSE     0xFFFF6600

#define COLOR_BASKET_BODY   0xFF885522
#define COLOR_BASKET_RIM    0xFFAA7733
#define COLOR_BASKET_INNER  0xFF664411

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

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

struct plane_info {
    uint32_t plane_id;
    int type;  /* DRM_PLANE_TYPE_PRIMARY=1, OVERLAY=0, CURSOR=2 */
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
    uint32_t crtc_idx;  /* index in resources for possible_crtcs mask */
    uint32_t width;
    uint32_t height;
    drmModeModeInfo mode;
    drmModeCrtc *saved_crtc;
    uint32_t mode_blob_id;

    /* Plane IDs */
    uint32_t primary_plane_id;
    uint32_t overlay_plane_id;
    uint32_t cursor_plane_id;

    /* Cached property IDs for atomic commits */
    struct plane_props primary_props;
    struct plane_props overlay_props;
    struct plane_props cursor_props;
    struct crtc_props crtc_props;
    struct conn_props conn_props;

    /* Framebuffers - triple-buffered for primary and overlay */
    struct drm_buffer primary_buf[NUM_BUFFERS];
    struct drm_buffer overlay_buf[NUM_BUFFERS];
    struct drm_buffer cursor_buf;
    int primary_front;
    int overlay_front;

    /* Atomic page-flip synchronization */
    bool pflip_pending;
};

enum obj_type {
    OBJ_FRUIT_APPLE = 0,
    OBJ_FRUIT_ORANGE,
    OBJ_FRUIT_GRAPE,
    OBJ_FRUIT_BANANA,
    OBJ_BOMB,
    OBJ_TYPE_COUNT
};

struct falling_obj {
    float x, y;
    float speed;
    enum obj_type type;
    bool active;
};

struct star {
    int x, y;
    uint8_t brightness;
};

#define NUM_STARS 80

struct game_state {
    int basket_x;  /* basket center X position on screen */
    int basket_y;  /* basket Y position on screen */
    struct falling_obj objects[MAX_FALLING_OBJS];
    int score;
    int lives;
    int level;
    bool running;
    bool game_over;
    uint64_t last_spawn_time;
    int spawn_interval_ms;
    struct star stars[NUM_STARS];
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

static uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void sleep_ns(uint64_t ns) {
    struct timespec ts = {
        .tv_sec = ns / 1000000000,
        .tv_nsec = ns % 1000000000
    };
    nanosleep(&ts, NULL);
}

/* ============================================================
 * DRM Functions
 * ============================================================ */

static int drm_open_device(struct drm_device *dev, const char *path) {
    const char *cards[] = {
        path,
        "/dev/dri/card0",
        "/dev/dri/card1",
        NULL
    };

    int start = path ? 0 : 1;
    for (int i = start; cards[i]; i++) {
        dev->fd = open(cards[i], O_RDWR | O_CLOEXEC);
        if (dev->fd >= 0) {
            drmModeRes *res = drmModeGetResources(dev->fd);
            if (res) {
                drmModeFreeResources(res);
                printf("[DRM] Opened %s\n", cards[i]);
                return 0;
            }
            close(dev->fd);
            dev->fd = -1;
        }
    }

    fprintf(stderr, "[DRM] ERROR: Cannot open any DRM device\n");
    return -1;
}

static int drm_find_connector(struct drm_device *dev) {
    drmModeRes *res = drmModeGetResources(dev->fd);
    if (!res) {
        fprintf(stderr, "[DRM] ERROR: drmModeGetResources failed\n");
        return -1;
    }

    drmModeConnector *conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(dev->fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;
        if (conn) {
            drmModeFreeConnector(conn);
            conn = NULL;
        }
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

    /* Find encoder and CRTC */
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
                if (dev->crtc_id)
                    break;
            }
        }
    }

    if (!dev->crtc_id) {
        fprintf(stderr, "[DRM] ERROR: No suitable CRTC found\n");
        drmModeFreeConnector(conn);
        drmModeFreeResources(res);
        return -1;
    }

    /* Find CRTC index for plane possible_crtcs matching */
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

/*
 * Find plane IDs for primary, overlay, and cursor planes
 * that are compatible with our CRTC.
 */
static int drm_find_planes(struct drm_device *dev) {
    drmModePlaneRes *plane_res = drmModeGetPlaneResources(dev->fd);
    if (!plane_res) {
        fprintf(stderr, "[DRM] ERROR: drmModeGetPlaneResources failed.\n"
                        "       Make sure DRM_CLIENT_CAP_UNIVERSAL_PLANES is set.\n");
        return -1;
    }

    printf("[DRM] Found %u planes\n", plane_res->count_planes);

    for (uint32_t i = 0; i < plane_res->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(dev->fd, plane_res->planes[i]);
        if (!plane)
            continue;

        /* Check if this plane is compatible with our CRTC */
        if (!(plane->possible_crtcs & (1 << dev->crtc_idx))) {
            drmModeFreePlane(plane);
            continue;
        }

        /* Get the "type" property to determine plane type */
        drmModeObjectProperties *props = drmModeObjectGetProperties(
            dev->fd, plane_res->planes[i], DRM_MODE_OBJECT_PLANE);
        if (!props) {
            drmModeFreePlane(plane);
            continue;
        }

        int plane_type = -1;
        for (uint32_t j = 0; j < props->count_props; j++) {
            drmModePropertyRes *prop = drmModeGetProperty(dev->fd, props->props[j]);
            if (!prop)
                continue;
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

    /* Create cursor plane buffer (small basket sprite, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned where basket starts) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (dev->width / 2) - (CURSOR_W / 2),
                        dev->height - CURSOR_H - 20,
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background gradient + stars + score/lives/level HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Falling objects (fruits to catch, bombs to dodge)\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player-controlled basket/paddle\n",
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

static void draw_circle(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= r2)
                put_pixel(fb, stride_px, scr_w, scr_h, cx + dx, cy + dy, color);
        }
    }
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
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

static const uint8_t font_alpha[][7] = {
    /* A */ {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* B */ {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    /* C */ {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    /* D */ {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    /* E */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    /* F */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    /* G */ {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E},
    /* H */ {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* I */ {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* J */ {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    /* K */ {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    /* L */ {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    /* M */ {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    /* N */ {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    /* O */ {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* P */ {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    /* Q */ {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    /* R */ {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    /* S */ {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    /* T */ {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    /* U */ {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* V */ {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
    /* W */ {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    /* X */ {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    /* Y */ {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    /* Z */ {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
};

static void draw_char(uint32_t *fb, uint32_t stride_px,
                      uint32_t scr_w, uint32_t scr_h,
                      int px, int py, char ch, int scale, uint32_t color) {
    const uint8_t *glyph = NULL;

    if (ch >= '0' && ch <= '9')
        glyph = font_5x7[ch - '0'];
    else if (ch >= 'A' && ch <= 'Z')
        glyph = font_alpha[ch - 'A'];
    else if (ch >= 'a' && ch <= 'z')
        glyph = font_alpha[ch - 'a'];
    else
        return; /* unsupported char, skip */

    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if (glyph[row] & (0x10 >> col)) {
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        put_pixel(fb, stride_px, scr_w, scr_h,
                                  px + col * scale + sx,
                                  py + row * scale + sy, color);
            }
        }
    }
}

static void draw_string(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int px, int py, const char *str, int scale, uint32_t color) {
    int x = px;
    for (int i = 0; str[i]; i++) {
        if (str[i] == ' ') {
            x += 4 * scale;
            continue;
        }
        if (str[i] == ':') {
            /* draw colon */
            put_pixel(fb, stride_px, scr_w, scr_h, x + scale, py + 2 * scale, color);
            put_pixel(fb, stride_px, scr_w, scr_h, x + scale, py + 4 * scale, color);
            x += 3 * scale;
            continue;
        }
        draw_char(fb, stride_px, scr_w, scr_h, x, py, str[i], scale, color);
        x += 6 * scale;
    }
}

static void draw_number(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int px, int py, int num, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", num);
    /* Draw digit by digit */
    int x = px;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_char(fb, stride_px, scr_w, scr_h, x, py, buf[i], scale, color);
        x += 6 * scale;
    }
}

/* ============================================================
 * Drawing Game Elements
 * ============================================================ */

/* Draw the basket sprite into the cursor plane buffer */
static void draw_basket(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    int cx = w / 2;
    int cy = h / 2;
    int bw = 90;  /* basket width */
    int bh = 50;  /* basket height */

    /* Basket body (trapezoid approximation using rectangles) */
    for (int row = 0; row < bh; row++) {
        /* Widen slightly as we go up (inverted trapezoid = basket shape) */
        float t = (float)row / bh;
        int row_w = (int)(bw * 0.6f + bw * 0.4f * t);
        int x0 = cx - row_w / 2;
        int y = cy + bh / 2 - row;
        draw_rect(fb, stride_px, w, h, x0, y, row_w, 1, COLOR_BASKET_BODY);
    }

    /* Basket rim (top edge) */
    int rim_w = bw;
    draw_rect(fb, stride_px, w, h, cx - rim_w / 2, cy - bh / 2, rim_w, 4, COLOR_BASKET_RIM);

    /* Inner shadow/depth */
    int inner_w = (int)(bw * 0.7f);
    int inner_h = bh / 3;
    draw_rect(fb, stride_px, w, h, cx - inner_w / 2, cy - bh / 2 + 5,
              inner_w, inner_h, COLOR_BASKET_INNER);

    /* Weave pattern (horizontal lines) */
    for (int row = 8; row < bh - 4; row += 6) {
        int lw = (int)(bw * (0.6f + 0.4f * ((float)row / bh)));
        int x0 = cx - lw / 2;
        int y = cy + bh / 2 - row;
        draw_rect(fb, stride_px, w, h, x0, y, lw, 1, COLOR_BASKET_RIM);
    }
}

/* Draw a fruit on the overlay plane */
static void draw_fruit(uint32_t *fb, uint32_t stride_px, uint32_t w, uint32_t h,
                       int cx, int cy, enum obj_type type) {
    uint32_t color;
    switch (type) {
    case OBJ_FRUIT_APPLE:  color = COLOR_FRUIT_APPLE;  break;
    case OBJ_FRUIT_ORANGE: color = COLOR_FRUIT_ORANGE; break;
    case OBJ_FRUIT_GRAPE:  color = COLOR_FRUIT_GRAPE;  break;
    case OBJ_FRUIT_BANANA: color = COLOR_FRUIT_BANANA; break;
    default: color = COLOR_FRUIT_APPLE; break;
    }

    /* Draw fruit as a circle */
    draw_circle(fb, stride_px, w, h, cx, cy, OBJ_SIZE / 2, color);

    /* Add a stem */
    draw_rect(fb, stride_px, w, h, cx - 1, cy - OBJ_SIZE / 2 - 4, 3, 6, 0xFF226600);

    /* Add a highlight */
    put_pixel(fb, stride_px, w, h, cx - 4, cy - 4, 0xFFFFFFFF);
    put_pixel(fb, stride_px, w, h, cx - 3, cy - 5, 0xFFFFFFFF);
}

/* Draw a bomb on the overlay plane */
static void draw_bomb(uint32_t *fb, uint32_t stride_px, uint32_t w, uint32_t h,
                      int cx, int cy) {
    /* Bomb body */
    draw_circle(fb, stride_px, w, h, cx, cy, OBJ_SIZE / 2, COLOR_BOMB);

    /* Fuse */
    draw_rect(fb, stride_px, w, h, cx - 1, cy - OBJ_SIZE / 2 - 6, 3, 8, COLOR_BOMB_FUSE);

    /* Spark at fuse tip */
    put_pixel(fb, stride_px, w, h, cx, cy - OBJ_SIZE / 2 - 7, 0xFFFFFF00);
    put_pixel(fb, stride_px, w, h, cx - 1, cy - OBJ_SIZE / 2 - 8, 0xFFFF8800);
    put_pixel(fb, stride_px, w, h, cx + 1, cy - OBJ_SIZE / 2 - 8, 0xFFFF8800);

    /* Highlight on bomb */
    put_pixel(fb, stride_px, w, h, cx - 3, cy - 3, 0xFF666666);
    put_pixel(fb, stride_px, w, h, cx - 2, cy - 4, 0xFF555555);

    /* X mark on bomb */
    for (int i = -3; i <= 3; i++) {
        put_pixel(fb, stride_px, w, h, cx + i, cy + i, 0xFFFF0000);
        put_pixel(fb, stride_px, w, h, cx + i, cy - i, 0xFFFF0000);
    }
}

/* Draw the background (primary plane) */
static void draw_background(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Gradient background */
    for (uint32_t y = 0; y < h; y++) {
        float t = (float)y / h;
        uint8_t r = (uint8_t)(0x00 * (1 - t) + 0x00 * t);
        uint8_t g = (uint8_t)(0x10 * (1 - t) + 0x08 * t);
        uint8_t b = (uint8_t)(0x30 * (1 - t) + 0x18 * t);
        uint32_t color = 0xFF000000 | (r << 16) | (g << 8) | b;
        for (uint32_t x = 0; x < w; x++)
            fb[y * stride_px + x] = color;
    }

    /* Draw stars */
    for (int i = 0; i < NUM_STARS; i++) {
        uint32_t sc = 0xFF000000 | (game->stars[i].brightness << 16) |
                      (game->stars[i].brightness << 8) | game->stars[i].brightness;
        put_pixel(fb, stride_px, w, h, game->stars[i].x, game->stars[i].y, sc);
    }

    /* Border */
    draw_rect(fb, stride_px, w, h, 0, 0, w, 2, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, h - 2, w, 2, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, 0, 2, h, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, w - 2, 0, 2, h, COLOR_BORDER);

    /* HUD - Score */
    draw_string(fb, stride_px, w, h, 20, 10, "SCORE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 100, 10, game->score, 2, COLOR_SCORE_VAL);

    /* HUD - Lives */
    draw_string(fb, stride_px, w, h, w - 200, 10, "LIVES:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w - 120, 10, game->lives, 2, COLOR_LIVES_VAL);

    /* HUD - Level */
    draw_string(fb, stride_px, w, h, w / 2 - 50, 10, "LVL:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w / 2 + 10, 10, game->level, 2, COLOR_SCORE_VAL);

    /* Game Over text */
    if (game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 - 20,
                    "GAME OVER", 4, 0xFFFF0000);
        draw_string(fb, stride_px, w, h, w / 2 - 80, h / 2 + 30,
                    "PRESS Q", 3, COLOR_HUD_TEXT);
    }
}

/* Draw the overlay plane (falling objects) */
static void draw_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw each active falling object */
    for (int i = 0; i < MAX_FALLING_OBJS; i++) {
        if (!game->objects[i].active)
            continue;

        int cx = (int)game->objects[i].x;
        int cy = (int)game->objects[i].y;

        if (game->objects[i].type == OBJ_BOMB) {
            draw_bomb(fb, stride_px, w, h, cx, cy);
        } else {
            draw_fruit(fb, stride_px, w, h, cx, cy, game->objects[i].type);
        }
    }
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64];
    for (int i = 0; i < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) {
            input_fds[input_count++] = fd;
        }
    }
    printf("[INPUT] Opened %d input devices\n", input_count);
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++)
        close(input_fds[i]);
    input_count = 0;
}

static void input_poll(void) {
    struct input_event ev;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2); /* 2=repeat */
                switch (ev.code) {
                case KEY_LEFT:
                    key_left = pressed;
                    break;
                case KEY_RIGHT:
                    key_right = pressed;
                    break;
                case KEY_ESC:
                case KEY_Q:
                    if (ev.value == 1)
                        g_quit = true;
                    break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));
    game->basket_x = screen_w / 2;
    game->basket_y = screen_h - CURSOR_H / 2 - 20;
    game->lives = MAX_LIVES;
    game->level = 1;
    game->running = true;
    game->game_over = false;
    game->last_spawn_time = get_time_ms();
    game->spawn_interval_ms = SPAWN_INTERVAL_MS;
    game->score = 0;

    /* Initialize stars */
    srand(time(NULL));
    for (int i = 0; i < NUM_STARS; i++) {
        game->stars[i].x = rand() % screen_w;
        game->stars[i].y = rand() % screen_h;
        game->stars[i].brightness = 100 + rand() % 155;
    }
}

static void spawn_object(struct game_state *game) {
    for (int i = 0; i < MAX_FALLING_OBJS; i++) {
        if (!game->objects[i].active) {
            game->objects[i].active = true;
            game->objects[i].x = 40 + rand() % (SCREEN_W - 80);
            game->objects[i].y = -OBJ_SIZE;
            game->objects[i].speed = FALL_SPEED_MIN +
                (float)(rand() % 100) / 100.0f * (FALL_SPEED_MAX - FALL_SPEED_MIN);

            /* Higher levels have more bombs */
            int bomb_chance = 20 + game->level * 5;
            if (bomb_chance > 60) bomb_chance = 60;

            if (rand() % 100 < bomb_chance)
                game->objects[i].type = OBJ_BOMB;
            else
                game->objects[i].type = rand() % 4; /* random fruit */

            return;
        }
    }
}

static void game_update(struct game_state *game) {
    if (game->game_over)
        return;

    /* Move basket */
    if (key_left)
        game->basket_x -= BASKET_SPEED;
    if (key_right)
        game->basket_x += BASKET_SPEED;

    /* Clamp basket position */
    if (game->basket_x < CURSOR_W / 2)
        game->basket_x = CURSOR_W / 2;
    if (game->basket_x > SCREEN_W - CURSOR_W / 2)
        game->basket_x = SCREEN_W - CURSOR_W / 2;

    /* Spawn new objects */
    uint64_t now = get_time_ms();
    if (now - game->last_spawn_time > (uint64_t)game->spawn_interval_ms) {
        spawn_object(game);
        game->last_spawn_time = now;
    }

    /* Update falling objects */
    for (int i = 0; i < MAX_FALLING_OBJS; i++) {
        if (!game->objects[i].active)
            continue;

        game->objects[i].y += game->objects[i].speed;

        /* Check if caught by basket (collision with basket area) */
        int obj_cx = (int)game->objects[i].x;
        int obj_cy = (int)game->objects[i].y;
        int basket_left = game->basket_x - 45;  /* half basket width */
        int basket_right = game->basket_x + 45;
        int basket_top = game->basket_y - 25;
        int basket_bottom = game->basket_y + 10;

        if (obj_cx >= basket_left && obj_cx <= basket_right &&
            obj_cy >= basket_top && obj_cy <= basket_bottom) {
            /* Caught! */
            game->objects[i].active = false;
            if (game->objects[i].type == OBJ_BOMB) {
                game->lives -= BOMB_PENALTY;
                if (game->lives <= 0) {
                    game->lives = 0;
                    game->game_over = true;
                }
            } else {
                game->score += FRUIT_SCORE;
                /* Level up every 100 points */
                int new_level = 1 + game->score / 100;
                if (new_level > game->level) {
                    game->level = new_level;
                    /* Speed up spawning */
                    game->spawn_interval_ms = SPAWN_INTERVAL_MS - game->level * 50;
                    if (game->spawn_interval_ms < 200)
                        game->spawn_interval_ms = 200;
                }
            }
            continue;
        }

        /* Check if fell off screen (missed) */
        if (obj_cy > SCREEN_H + OBJ_SIZE) {
            game->objects[i].active = false;
            /* Missing a fruit costs nothing; missing doesn't penalize */
        }
    }

    /* Twinkle stars */
    for (int i = 0; i < NUM_STARS; i++) {
        int delta = (rand() % 7) - 3;
        int new_b = game->stars[i].brightness + delta;
        if (new_b < 80) new_b = 80;
        if (new_b > 255) new_b = 255;
        game->stars[i].brightness = new_b;
    }
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char *argv[]) {
    struct drm_device dev;
    struct game_state game;
    const char *card_path = (argc > 1) ? argv[1] : NULL;

    /* Setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== DRM Catch Game (Overlay + Cursor Planes) ===\n");
    printf("Controls: Left/Right arrows to move, Q/ESC to quit\n\n");

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game state */
    game_init(&game, dev.width, dev.height);

    /* Draw the basket sprite once (cursor plane is static content, just moves) */
    draw_basket(&dev.cursor_buf);

    printf("[GAME] Starting main loop...\n");

    /* Main game loop */
    struct timespec frame_start, frame_end;
    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        /* Poll input */
        input_poll();

        /* Update game state */
        game_update(&game);

        /* Render primary plane (background + HUD) into back buffer */
        draw_background(&dev, &game);

        /* Render overlay plane (falling objects) into back buffer */
        draw_overlay(&dev, &game);

        /* Compute cursor plane position (basket movement) */
        int cursor_x = game.basket_x - CURSOR_W / 2;
        int cursor_y = game.basket_y - CURSOR_H / 2;

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

        /* Frame rate limiting */
        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        uint64_t elapsed_ns = (frame_end.tv_sec - frame_start.tv_sec) * 1000000000ULL +
                              (frame_end.tv_nsec - frame_start.tv_nsec);
        if (elapsed_ns < FRAME_TIME_NS)
            sleep_ns(FRAME_TIME_NS - elapsed_ns);
    }

    /* Wait a moment on game over screen */
    if (game.game_over) {
        printf("[GAME] Game Over! Final score: %d (Level %d)\n", game.score, game.level);
        /* Keep displaying until quit */
        while (!g_quit) {
            input_poll();
            sleep_ns(50000000); /* 50ms */
        }
    }

    printf("[GAME] Shutting down...\n");

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);

    printf("Final Score: %d | Level: %d\n", game.score, game.level);
    return 0;
}
