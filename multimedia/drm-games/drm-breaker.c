/*
 * drm-breaker.c - Brick Breaker using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background gradient + brick wall + score/lives HUD
 *   - Overlay plane:  Ball, particles, power-ups
 *   - Cursor plane:   Player paddle
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-breaker drm-breaker.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-breaker drm-breaker.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-breaker [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right arrow keys: move paddle
 *   - Space: launch ball
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

#define SCREEN_W            1280
#define SCREEN_H            720

#define OVERLAY_W           1280
#define OVERLAY_H           720

#define CURSOR_W            128
#define CURSOR_H            32

/* Game tuning */
#define PADDLE_SPEED        8
#define BALL_SPEED          5.0f
#define BALL_RADIUS         8

#define BRICK_COLS          12
#define BRICK_ROWS          6
#define BRICK_W             90
#define BRICK_H             28
#define BRICK_GAP           4
#define BRICK_OFFSET_X      ((SCREEN_W - (BRICK_COLS * (BRICK_W + BRICK_GAP))) / 2)
#define BRICK_OFFSET_Y      80

#define MAX_PARTICLES       30
#define MAX_LIVES           5
#define BRICK_SCORE         10

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG_TOP        0xFF0A0A2E
#define COLOR_BG_BOT        0xFF050518
#define COLOR_BORDER        0xFF333366
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE_VAL     0xFF00FF80
#define COLOR_LIVES_VAL     0xFFFF4444

#define COLOR_BALL          0xFFFFFFFF
#define COLOR_BALL_GLOW     0xFF8888FF
#define COLOR_PADDLE_BODY   0xFF4488FF
#define COLOR_PADDLE_EDGE   0xFF2266DD
#define COLOR_PADDLE_HIGH   0xFF88BBFF

/* Brick colors per row */
static const uint32_t brick_colors[] = {
    0xFFFF2222, /* Red */
    0xFFFF8800, /* Orange */
    0xFFFFDD00, /* Yellow */
    0xFF22FF22, /* Green */
    0xFF2288FF, /* Blue */
    0xFF8844FF, /* Purple */
};

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
    float dx, dy;
    bool active;
    bool launched;
};

struct brick {
    bool alive;
    int hits;  /* hits remaining */
};

struct particle {
    float x, y;
    float dx, dy;
    int life;
    uint32_t color;
    bool active;
};

struct game_state {
    int paddle_x;
    int paddle_y;
    struct ball ball;
    struct brick bricks[BRICK_ROWS][BRICK_COLS];
    struct particle particles[MAX_PARTICLES];
    int score;
    int lives;
    int level;
    int bricks_remaining;
    bool running;
    bool game_over;
    bool level_complete;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false;
static bool key_right = false;
static bool key_space = false;
static bool key_space_prev = false;

/* ============================================================
 * Signal Handler
 * ============================================================ */

static void signal_handler(int sig) {
    (void)sig;
    g_quit = true;
}

/* ============================================================
 * Timing
 * ============================================================ */

static uint64_t get_time_ms(void) __attribute__((unused));
static uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void sleep_ns(uint64_t ns) {
    struct timespec ts = { .tv_sec = ns / 1000000000, .tv_nsec = ns % 1000000000 };
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

    drmModeEncoder *enc = NULL;
    if (conn->encoder_id) enc = drmModeGetEncoder(dev->fd, conn->encoder_id);
    if (enc) { dev->crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    else {
        for (int i = 0; i < conn->count_encoders; i++) {
            enc = drmModeGetEncoder(dev->fd, conn->encoders[i]);
            if (enc) {
                for (int j = 0; j < res->count_crtcs; j++) {
                    if (enc->possible_crtcs & (1 << j)) { dev->crtc_id = res->crtcs[j]; break; }
                }
                drmModeFreeEncoder(enc);
                if (dev->crtc_id) break;
            }
        }
    }
    if (!dev->crtc_id) { drmModeFreeConnector(conn); drmModeFreeResources(res); return -1; }

    for (int i = 0; i < res->count_crtcs; i++)
        if (res->crtcs[i] == dev->crtc_id) { dev->crtc_idx = i; break; }

    dev->saved_crtc = drmModeGetCrtc(dev->fd, dev->crtc_id);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    return 0;
}

static int drm_find_planes(struct drm_device *dev) {
    drmModePlaneRes *plane_res = drmModeGetPlaneResources(dev->fd);
    if (!plane_res) return -1;

    for (uint32_t i = 0; i < plane_res->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(dev->fd, plane_res->planes[i]);
        if (!plane) continue;
        if (!(plane->possible_crtcs & (1 << dev->crtc_idx))) { drmModeFreePlane(plane); continue; }

        drmModeObjectProperties *props = drmModeObjectGetProperties(
            dev->fd, plane_res->planes[i], DRM_MODE_OBJECT_PLANE);
        if (!props) { drmModeFreePlane(plane); continue; }

        int plane_type = -1;
        for (uint32_t j = 0; j < props->count_props; j++) {
            drmModePropertyRes *prop = drmModeGetProperty(dev->fd, props->props[j]);
            if (!prop) continue;
            if (strcmp(prop->name, "type") == 0) { plane_type = (int)props->prop_values[j]; drmModeFreeProperty(prop); break; }
            drmModeFreeProperty(prop);
        }
        drmModeFreeObjectProperties(props);

        switch (plane_type) {
        case 1: if (!dev->primary_plane_id) dev->primary_plane_id = plane_res->planes[i]; break;
        case 0: if (!dev->overlay_plane_id) dev->overlay_plane_id = plane_res->planes[i]; break;
        case 2: if (!dev->cursor_plane_id) dev->cursor_plane_id = plane_res->planes[i]; break;
        }
        drmModeFreePlane(plane);
    }
    drmModeFreePlaneResources(plane_res);

    if (!dev->primary_plane_id || !dev->overlay_plane_id || !dev->cursor_plane_id) {
        fprintf(stderr, "[DRM] ERROR: Could not find all three plane types\n");
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
    if (buf->map && buf->map != MAP_FAILED) { munmap(buf->map, buf->size); buf->map = NULL; }
    if (buf->fb_id) { drmModeRmFB(dev->fd, buf->fb_id); buf->fb_id = 0; }
    if (buf->handle) {
        struct drm_mode_destroy_dumb destroy = { .handle = buf->handle };
        drmIoctl(dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy); buf->handle = 0;
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

    /* Create cursor plane buffer (small paddle sprite, ARGB8888 for per-pixel alpha) */
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

    /* Overlay plane (full screen, positioned at 0,0) */
    add_plane_to_atomic(req, dev->overlay_plane_id, &dev->overlay_props,
                        dev->overlay_buf[0].fb_id, dev->crtc_id,
                        0, 0, dev->width, dev->height,
                        0, 0, OVERLAY_W << 16, OVERLAY_H << 16);
    if (dev->overlay_props.alpha)
        drmModeAtomicAddProperty(req, dev->overlay_plane_id,
                                 dev->overlay_props.alpha, OVERLAY_ALPHA);

    /* Cursor plane (positioned where paddle starts) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (int32_t)((dev->width / 2) - (CURSOR_W / 2)),
                        (int32_t)(dev->height - CURSOR_H - 30),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background gradient + brick wall + score/lives HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Ball, particles, power-ups\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player paddle\n",
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
        drmModeSetCrtc(dev->fd, dev->saved_crtc->crtc_id, dev->saved_crtc->buffer_id,
                       dev->saved_crtc->x, dev->saved_crtc->y, &dev->conn_id, 1, &dev->saved_crtc->mode);
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

    if (dev->fd >= 0) close(dev->fd);
}

/* ============================================================
 * Drawing Primitives
 * ============================================================ */

static inline void put_pixel(uint32_t *fb, uint32_t stride_px, uint32_t w, uint32_t h,
                             int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h)
        fb[y * stride_px + x] = color;
}

static void draw_rect(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                      int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride_px + i] = color;
}

static void draw_circle(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                        int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            if (dx * dx + dy * dy <= r2)
                put_pixel(fb, stride_px, scr_w, scr_h, cx + dx, cy + dy, color);
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++) fb[i] = color;
}

/* Simple 5x7 font */
static const uint8_t font_5x7[][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},{0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
};

static const uint8_t font_alpha[][7] = {
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},{0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E},{0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},{0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11},{0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},{0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},{0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},{0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
};

static void draw_char(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                      int px, int py, char ch, int scale, uint32_t color) {
    const uint8_t *glyph = NULL;
    if (ch >= '0' && ch <= '9') glyph = font_5x7[ch - '0'];
    else if (ch >= 'A' && ch <= 'Z') glyph = font_alpha[ch - 'A'];
    else if (ch >= 'a' && ch <= 'z') glyph = font_alpha[ch - 'a'];
    else return;
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (glyph[row] & (0x10 >> col))
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        put_pixel(fb, stride_px, scr_w, scr_h,
                                  px + col * scale + sx, py + row * scale + sy, color);
}

static void draw_string(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                        int px, int py, const char *str, int scale, uint32_t color) {
    int x = px;
    for (int i = 0; str[i]; i++) {
        if (str[i] == ' ') { x += 4 * scale; continue; }
        if (str[i] == ':') {
            put_pixel(fb, stride_px, scr_w, scr_h, x + scale, py + 2 * scale, color);
            put_pixel(fb, stride_px, scr_w, scr_h, x + scale, py + 4 * scale, color);
            x += 3 * scale; continue;
        }
        draw_char(fb, stride_px, scr_w, scr_h, x, py, str[i], scale, color);
        x += 6 * scale;
    }
}

static void draw_number(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                        int px, int py, int num, int scale, uint32_t color) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
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

/* Draw paddle sprite into cursor buffer */
static void draw_paddle(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    int pw = 100; /* paddle width */
    int ph = 16;  /* paddle height */
    int px = (w - pw) / 2;
    int py = (h - ph) / 2;

    /* Main body */
    draw_rect(fb, stride_px, w, h, px, py, pw, ph, COLOR_PADDLE_BODY);
    /* Rounded edges */
    draw_circle(fb, stride_px, w, h, px + 6, py + ph / 2, ph / 2, COLOR_PADDLE_EDGE);
    draw_circle(fb, stride_px, w, h, px + pw - 6, py + ph / 2, ph / 2, COLOR_PADDLE_EDGE);
    /* Highlight */
    draw_rect(fb, stride_px, w, h, px + 10, py + 2, pw - 20, 4, COLOR_PADDLE_HIGH);
}

/* Draw background + bricks (primary plane) */
static void draw_background(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Gradient */
    for (uint32_t y = 0; y < h; y++) {
        float t = (float)y / h;
        uint8_t r = (uint8_t)(0x0A * (1 - t) + 0x05 * t);
        uint8_t g = (uint8_t)(0x0A * (1 - t) + 0x05 * t);
        uint8_t b = (uint8_t)(0x2E * (1 - t) + 0x18 * t);
        uint32_t color = 0xFF000000 | (r << 16) | (g << 8) | b;
        for (uint32_t x = 0; x < w; x++)
            fb[y * stride_px + x] = color;
    }

    /* Border */
    draw_rect(fb, stride_px, w, h, 0, 0, w, 3, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, 0, 3, h, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, w - 3, 0, 3, h, COLOR_BORDER);

    /* Draw bricks */
    for (int row = 0; row < BRICK_ROWS; row++) {
        for (int col = 0; col < BRICK_COLS; col++) {
            if (!game->bricks[row][col].alive) continue;
            int bx = BRICK_OFFSET_X + col * (BRICK_W + BRICK_GAP);
            int by = BRICK_OFFSET_Y + row * (BRICK_H + BRICK_GAP);

            uint32_t color = brick_colors[row % 6];
            draw_rect(fb, stride_px, w, h, bx, by, BRICK_W, BRICK_H, color);
            /* Highlight on top */
            uint32_t hi = color | 0x00404040;
            draw_rect(fb, stride_px, w, h, bx + 2, by + 2, BRICK_W - 4, 4, hi);
            /* Shadow on bottom */
            uint32_t sh = (color & 0xFF000000) | ((color & 0x00FEFEFE) >> 1);
            draw_rect(fb, stride_px, w, h, bx + 2, by + BRICK_H - 4, BRICK_W - 4, 2, sh);
        }
    }

    /* HUD */
    draw_string(fb, stride_px, w, h, 20, 10, "SCORE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 100, 10, game->score, 2, COLOR_SCORE_VAL);
    draw_string(fb, stride_px, w, h, w - 200, 10, "LIVES:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w - 120, 10, game->lives, 2, COLOR_LIVES_VAL);
    draw_string(fb, stride_px, w, h, w / 2 - 50, 10, "LVL:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w / 2 + 10, 10, game->level, 2, COLOR_SCORE_VAL);

    if (game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 - 20, "GAME OVER", 4, 0xFFFF0000);
        draw_string(fb, stride_px, w, h, w / 2 - 80, h / 2 + 30, "PRESS Q", 3, COLOR_HUD_TEXT);
    }
    if (game->level_complete) {
        draw_string(fb, stride_px, w, h, w / 2 - 120, h / 2 - 20, "LEVEL CLEAR", 4, 0xFF00FF00);
    }
    if (!game->ball.launched && !game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 + 60, "PRESS SPACE", 2, COLOR_HUD_TEXT);
    }
}

/* Draw overlay (ball + particles) */
static void draw_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw ball */
    if (game->ball.active) {
        int bx = (int)game->ball.x;
        int by = (int)game->ball.y;
        /* Glow */
        draw_circle(fb, stride_px, w, h, bx, by, BALL_RADIUS + 3, COLOR_BALL_GLOW);
        /* Core */
        draw_circle(fb, stride_px, w, h, bx, by, BALL_RADIUS, COLOR_BALL);
        /* Highlight */
        put_pixel(fb, stride_px, w, h, bx - 2, by - 2, 0xFFFFFFFF);
    }

    /* Draw particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        int px = (int)game->particles[i].x;
        int py = (int)game->particles[i].y;
        int size = game->particles[i].life > 10 ? 3 : 2;
        draw_rect(fb, stride_px, w, h, px - size / 2, py - size / 2, size, size,
                  game->particles[i].color);
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
        if (fd >= 0) input_fds[input_count++] = fd;
    }
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static void input_poll(void) {
    struct input_event ev;
    key_space_prev = key_space;
    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == sizeof(ev)) {
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2);
                switch (ev.code) {
                case KEY_LEFT:  key_left = pressed; break;
                case KEY_RIGHT: key_right = pressed; break;
                case KEY_SPACE: key_space = pressed; break;
                case KEY_ESC:
                case KEY_Q:     if (ev.value == 1) g_quit = true; break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void init_bricks(struct game_state *game) {
    game->bricks_remaining = 0;
    for (int row = 0; row < BRICK_ROWS; row++) {
        for (int col = 0; col < BRICK_COLS; col++) {
            game->bricks[row][col].alive = true;
            game->bricks[row][col].hits = 1;
            game->bricks_remaining++;
        }
    }
}

static void spawn_particles(struct game_state *game, float x, float y, uint32_t color, int count) {
    for (int n = 0; n < count; n++) {
        for (int i = 0; i < MAX_PARTICLES; i++) {
            if (!game->particles[i].active) {
                game->particles[i].active = true;
                game->particles[i].x = x;
                game->particles[i].y = y;
                game->particles[i].dx = ((rand() % 100) - 50) / 15.0f;
                game->particles[i].dy = ((rand() % 100) - 50) / 15.0f;
                game->particles[i].life = 15 + rand() % 15;
                game->particles[i].color = color;
                break;
            }
        }
    }
}

static void game_init(struct game_state *game) {
    memset(game, 0, sizeof(*game));
    game->paddle_x = SCREEN_W / 2;
    game->paddle_y = SCREEN_H - 50;
    game->ball.x = SCREEN_W / 2;
    game->ball.y = game->paddle_y - CURSOR_H / 2 - BALL_RADIUS - 2;
    game->ball.dx = 0;
    game->ball.dy = 0;
    game->ball.active = true;
    game->ball.launched = false;
    game->lives = MAX_LIVES;
    game->level = 1;
    game->running = true;
    game->score = 0;
    srand(time(NULL));
    init_bricks(game);
}

static void launch_ball(struct game_state *game) {
    if (game->ball.launched) return;
    game->ball.launched = true;
    float angle = -M_PI / 4 + ((float)(rand() % 50) / 100.0f) * (M_PI / 2);
    game->ball.dx = BALL_SPEED * sinf(angle);
    game->ball.dy = -BALL_SPEED * cosf(angle);
}

static void reset_ball(struct game_state *game) {
    game->ball.x = game->paddle_x;
    game->ball.y = game->paddle_y - CURSOR_H / 2 - BALL_RADIUS - 2;
    game->ball.dx = 0;
    game->ball.dy = 0;
    game->ball.launched = false;
    game->ball.active = true;
}

static void game_update(struct game_state *game) {
    if (game->game_over || game->level_complete) return;

    /* Move paddle */
    if (key_left) game->paddle_x -= PADDLE_SPEED;
    if (key_right) game->paddle_x += PADDLE_SPEED;
    if (game->paddle_x < CURSOR_W / 2 + 3) game->paddle_x = CURSOR_W / 2 + 3;
    if (game->paddle_x > SCREEN_W - CURSOR_W / 2 - 3) game->paddle_x = SCREEN_W - CURSOR_W / 2 - 3;

    /* Launch ball */
    if (key_space && !key_space_prev) launch_ball(game);

    /* Ball follows paddle if not launched */
    if (!game->ball.launched) {
        game->ball.x = game->paddle_x;
        game->ball.y = game->paddle_y - CURSOR_H / 2 - BALL_RADIUS - 2;
        goto update_particles;
    }

    /* Move ball */
    game->ball.x += game->ball.dx;
    game->ball.y += game->ball.dy;

    /* Wall bounces */
    if (game->ball.x - BALL_RADIUS <= 3) {
        game->ball.x = 3 + BALL_RADIUS;
        game->ball.dx = -game->ball.dx;
    }
    if (game->ball.x + BALL_RADIUS >= SCREEN_W - 3) {
        game->ball.x = SCREEN_W - 3 - BALL_RADIUS;
        game->ball.dx = -game->ball.dx;
    }
    if (game->ball.y - BALL_RADIUS <= 3) {
        game->ball.y = 3 + BALL_RADIUS;
        game->ball.dy = -game->ball.dy;
    }

    /* Ball fell off bottom */
    if (game->ball.y > SCREEN_H + BALL_RADIUS) {
        game->lives--;
        if (game->lives <= 0) {
            game->lives = 0;
            game->game_over = true;
        } else {
            reset_ball(game);
        }
        goto update_particles;
    }

    /* Paddle collision */
    int paddle_left = game->paddle_x - 50;
    int paddle_right = game->paddle_x + 50;
    int paddle_top = game->paddle_y - 8;

    if (game->ball.dy > 0 &&
        game->ball.y + BALL_RADIUS >= paddle_top &&
        game->ball.y - BALL_RADIUS <= paddle_top + 16 &&
        game->ball.x >= paddle_left && game->ball.x <= paddle_right) {
        /* Reflect based on where ball hit paddle */
        float hit_pos = (game->ball.x - game->paddle_x) / 50.0f; /* -1 to 1 */
        float angle = hit_pos * (M_PI / 3); /* max 60 degrees */
        float speed = sqrtf(game->ball.dx * game->ball.dx + game->ball.dy * game->ball.dy);
        game->ball.dx = speed * sinf(angle);
        game->ball.dy = -speed * cosf(angle);
        game->ball.y = paddle_top - BALL_RADIUS;
    }

    /* Brick collision */
    for (int row = 0; row < BRICK_ROWS; row++) {
        for (int col = 0; col < BRICK_COLS; col++) {
            if (!game->bricks[row][col].alive) continue;

            int bx = BRICK_OFFSET_X + col * (BRICK_W + BRICK_GAP);
            int by = BRICK_OFFSET_Y + row * (BRICK_H + BRICK_GAP);

            /* Simple AABB collision */
            if (game->ball.x + BALL_RADIUS >= bx &&
                game->ball.x - BALL_RADIUS <= bx + BRICK_W &&
                game->ball.y + BALL_RADIUS >= by &&
                game->ball.y - BALL_RADIUS <= by + BRICK_H) {

                game->bricks[row][col].hits--;
                if (game->bricks[row][col].hits <= 0) {
                    game->bricks[row][col].alive = false;
                    game->bricks_remaining--;
                    game->score += BRICK_SCORE;
                    spawn_particles(game, bx + BRICK_W / 2, by + BRICK_H / 2,
                                    brick_colors[row % 6], 6);
                }

                /* Determine reflection direction */
                float cx = bx + BRICK_W / 2.0f;
                float cy_b = by + BRICK_H / 2.0f;
                float dx = game->ball.x - cx;
                float dy = game->ball.y - cy_b;

                if (fabsf(dx) / BRICK_W > fabsf(dy) / BRICK_H)
                    game->ball.dx = -game->ball.dx;
                else
                    game->ball.dy = -game->ball.dy;

                goto brick_done;
            }
        }
    }
brick_done:

    /* Check level complete */
    if (game->bricks_remaining <= 0) {
        game->level_complete = true;
        game->level++;
    }

update_particles:
    /* Update particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].dx;
        game->particles[i].y += game->particles[i].dy;
        game->particles[i].dy += 0.2f; /* gravity */
        game->particles[i].life--;
        if (game->particles[i].life <= 0)
            game->particles[i].active = false;
    }
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char *argv[]) {
    struct drm_device dev;
    struct game_state game;
    const char *card_path = (argc > 1) ? argv[1] : NULL;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== DRM Brick Breaker (Atomic + Triple-Buffered) ===\n");
    printf("Controls: Left/Right to move, Space to launch, Q/ESC to quit\n\n");

    if (drm_init(&dev, card_path) < 0) { fprintf(stderr, "Failed to initialize DRM\n"); return 1; }

    input_init();
    game_init(&game);
    draw_paddle(&dev.cursor_buf);

    printf("[GAME] Starting main loop...\n");

    struct timespec frame_start, frame_end;
    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        input_poll();
        game_update(&game);

        /* Render both planes into their back buffers */
        draw_background(&dev, &game);
        draw_overlay(&dev, &game);

        /* Compute cursor position */
        int cx = game.paddle_x - CURSOR_W / 2;
        int cy = game.paddle_y - CURSOR_H / 2;

        /* Wait for previous flip to complete (vsync pacing) */
        drm_wait_flip(&dev);

        /* Submit single atomic commit for all 3 planes */
        drm_atomic_flip(&dev, cx, cy);

        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        uint64_t elapsed_ns = (frame_end.tv_sec - frame_start.tv_sec) * 1000000000ULL +
                              (frame_end.tv_nsec - frame_start.tv_nsec);
        if (elapsed_ns < FRAME_TIME_NS) sleep_ns(FRAME_TIME_NS - elapsed_ns);

        /* Handle level complete - reset after brief pause */
        if (game.level_complete) {
            sleep_ns(1500000000ULL); /* 1.5s pause */
            game.level_complete = false;
            init_bricks(&game);
            reset_ball(&game);
        }
    }

    if (game.game_over) {
        printf("[GAME] Game Over! Final score: %d (Level %d)\n", game.score, game.level);
        while (!g_quit) { input_poll(); sleep_ns(50000000); }
    }

    input_cleanup();
    drm_cleanup(&dev);
    printf("Final Score: %d | Level: %d\n", game.score, game.level);
    return 0;
}
