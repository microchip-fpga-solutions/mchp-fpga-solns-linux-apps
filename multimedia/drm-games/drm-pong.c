/*
 * drm-pong.c - Classic Pong using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background gradient + grid + center line + scores HUD
 *   - Overlay plane:  Ball, ball trail, particles, AI paddle
 *   - Cursor plane:   Player paddle (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-pong drm-pong.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-pong drm-pong.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-pong [/dev/dri/cardN]
 *
 * Controls:
 *   - Up/Down arrow keys: move player paddle
 *   - P: pause
 *   - Space: restart after game over
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

/* Overlay plane dimensions */
#define OVERLAY_W           1280
#define OVERLAY_H           720

/* Cursor plane dimensions (player paddle) */
#define CURSOR_W            64
#define CURSOR_H            128

/* Game tuning */
#define PADDLE_W            12
#define PADDLE_H            60
#define PADDLE_SPEED        5
#define PADDLE_MARGIN       30
#define AI_SPEED            4

#define BALL_SIZE           10
#define BALL_SPEED_INIT     4.0f
#define BALL_SPEED_INC      0.15f
#define BALL_MAX_SPEED      12.0f

#define TRAIL_LENGTH        12
#define MAX_PARTICLES       80
#define SHAKE_FRAMES        15
#define WIN_SCORE           11

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG            0xFF0A0A1A
#define COLOR_GRID          0xFF151530
#define COLOR_PADDLE_P1     0xFF00AAFF
#define COLOR_PADDLE_P2     0xFFFF4488
#define COLOR_BALL          0xFFFFFF00
#define COLOR_BALL_GLOW     0xFF888800
#define COLOR_CENTER_LINE   0xFF333366
#define COLOR_SCORE_P1      0xFF0088CC
#define COLOR_SCORE_P2      0xFFCC2266

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

struct trail_point {
    float x, y;
    bool active;
};

struct particle {
    float x, y, vx, vy;
    float life;
    uint32_t color;
    bool active;
};

struct paddle {
    float x, y;
    float target_y;
    uint32_t color;
};

struct ball {
    float x, y;
    float vx, vy;
    float speed;
};

struct game_state {
    struct paddle player;
    struct paddle ai;
    struct ball ball;
    struct trail_point trail[TRAIL_LENGTH];
    int trail_idx;
    struct particle particles[MAX_PARTICLES];
    int score_p1, score_p2;
    bool running, paused;
    bool game_over;
    int rally_count;
    float shake_x, shake_y;
    int shake_timer;
    int frame_count;
    bool serving;
    int serve_timer;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_up = false, key_down = false;

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

    /* Cursor plane (positioned where player paddle starts) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        PADDLE_MARGIN - CURSOR_W / 4,
                        (int)(dev->height / 2) - (CURSOR_H / 2),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background gradient + grid + center line + scores HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Ball, ball trail, particles, AI paddle\n",
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

static void draw_number(uint32_t *fb, uint32_t stride_px,
                        uint32_t scr_w, uint32_t scr_h,
                        int px, int py, int num, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", num);
    int x = px;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            int d = buf[i] - '0';
            for (int row = 0; row < 7; row++)
                for (int col = 0; col < 5; col++)
                    if (font_5x7[d][row] & (0x10 >> col))
                        draw_rect(fb, stride_px, scr_w, scr_h,
                                  x + col * scale, py + row * scale, scale, scale, color);
        }
        x += 6 * scale;
    }
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64];
    char name[256];
    for (int i = 0; i < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) {
            input_fds[input_count++] = fd;
            if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0)
                printf("[INPUT] %s: %s\n", path, name);
            else
                printf("[INPUT] %s: (unknown)\n", path);
        }
    }
    printf("[INPUT] Opened %d input devices\n", input_count);
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++)
        close(input_fds[i]);
    input_count = 0;
}

static void input_poll(struct game_state *game, uint32_t screen_h) {
    struct input_event ev;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS) {
                /* Touchscreen support: map touch Y to paddle position */
                if (ev.code == ABS_Y || ev.code == ABS_MT_POSITION_Y) {
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0)
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    game->player.target_y = (float)(ev.value * (int)screen_h / max_val);
                }
            }
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2);
                switch (ev.code) {
                case KEY_UP:
                case KEY_W:
                    key_up = pressed;
                    break;
                case KEY_DOWN:
                case KEY_S:
                    key_down = pressed;
                    break;
                case KEY_P:
                    if (ev.value == 1)
                        game->paused = !game->paused;
                    break;
                case KEY_SPACE:
                    if (ev.value == 1 && game->game_over) {
                        game->game_over = false;
                        game->score_p1 = 0;
                        game->score_p2 = 0;
                        game->serving = true;
                        game->serve_timer = 60;
                    }
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

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void spawn_particles(struct game_state *game, float x, float y, int count, uint32_t color) {
    for (int i = 0; i < count; i++) {
        for (int p = 0; p < MAX_PARTICLES; p++) {
            if (!game->particles[p].active) {
                float angle = randf() * 6.28f;
                float speed = 1.0f + randf() * 4.0f;
                game->particles[p] = (struct particle){
                    .x = x, .y = y,
                    .vx = cosf(angle) * speed,
                    .vy = sinf(angle) * speed,
                    .life = 0.4f + randf() * 0.6f,
                    .color = color, .active = true
                };
                break;
            }
        }
    }
}

static void reset_ball(struct game_state *game, uint32_t w, uint32_t h) {
    game->ball.x = w / 2.0f;
    game->ball.y = h / 2.0f;
    game->ball.speed = BALL_SPEED_INIT;
    float angle = (randf() - 0.5f) * 1.0f;
    int dir = (rand() % 2) ? 1 : -1;
    game->ball.vx = dir * cosf(angle) * game->ball.speed;
    game->ball.vy = sinf(angle) * game->ball.speed;
    game->rally_count = 0;
    game->serving = true;
    game->serve_timer = 60;
    for (int i = 0; i < TRAIL_LENGTH; i++) game->trail[i].active = false;
}

static void game_init(struct game_state *game, uint32_t w, uint32_t h) {
    memset(game, 0, sizeof(*game));
    srand((unsigned)time(NULL));

    game->player.x = PADDLE_MARGIN;
    game->player.y = h / 2.0f - PADDLE_H / 2.0f;
    game->player.color = COLOR_PADDLE_P1;

    game->ai.x = w - PADDLE_MARGIN - PADDLE_W;
    game->ai.y = h / 2.0f - PADDLE_H / 2.0f;
    game->ai.color = COLOR_PADDLE_P2;

    game->running = true;
    reset_ball(game, w, h);
}

static void game_update(struct game_state *game, uint32_t w, uint32_t h) {
    if (game->paused || game->game_over) return;
    game->frame_count++;

    /* Serve delay */
    if (game->serving) {
        game->serve_timer--;
        if (game->serve_timer <= 0) game->serving = false;
        game->ball.x = w / 2.0f;
        game->ball.y = h / 2.0f;
    }

    /* Player paddle movement (keyboard) */
    if (key_up) game->player.y -= PADDLE_SPEED;
    if (key_down) game->player.y += PADDLE_SPEED;

    /* Touch movement (smooth tracking) */
    if (game->player.target_y > 0) {
        float diff = game->player.target_y - (game->player.y + PADDLE_H / 2.0f);
        if (fabsf(diff) > 2) game->player.y += diff * 0.15f;
    }

    /* Clamp player */
    if (game->player.y < 0) game->player.y = 0;
    if (game->player.y > h - PADDLE_H) game->player.y = h - PADDLE_H;

    /* AI paddle */
    float ai_target = game->ball.y - PADDLE_H / 2.0f;
    float ai_diff = ai_target - game->ai.y;
    float ai_speed = AI_SPEED + game->rally_count * 0.1f;
    if (ai_speed > 7) ai_speed = 7;
    if (ai_diff > ai_speed) game->ai.y += ai_speed;
    else if (ai_diff < -ai_speed) game->ai.y -= ai_speed;
    else game->ai.y += ai_diff;
    if (game->ai.y < 0) game->ai.y = 0;
    if (game->ai.y > h - PADDLE_H) game->ai.y = h - PADDLE_H;

    if (game->serving) return;

    /* Ball trail */
    game->trail[game->trail_idx] = (struct trail_point){
        .x = game->ball.x, .y = game->ball.y, .active = true
    };
    game->trail_idx = (game->trail_idx + 1) % TRAIL_LENGTH;

    /* Move ball */
    game->ball.x += game->ball.vx;
    game->ball.y += game->ball.vy;

    /* Top/bottom wall collision */
    if (game->ball.y <= 0) {
        game->ball.y = 0;
        game->ball.vy = -game->ball.vy;
        spawn_particles(game, game->ball.x, 0, 5, 0xFFFFFFFF);
    }
    if (game->ball.y >= h - BALL_SIZE) {
        game->ball.y = h - BALL_SIZE;
        game->ball.vy = -game->ball.vy;
        spawn_particles(game, game->ball.x, (float)h, 5, 0xFFFFFFFF);
    }

    /* Player paddle collision */
    if (game->ball.vx < 0 &&
        game->ball.x <= game->player.x + PADDLE_W &&
        game->ball.x >= game->player.x &&
        game->ball.y + BALL_SIZE >= game->player.y &&
        game->ball.y <= game->player.y + PADDLE_H) {

        game->ball.x = game->player.x + PADDLE_W;
        float hit_pos = (game->ball.y + BALL_SIZE / 2.0f - game->player.y) / PADDLE_H;
        float angle = (hit_pos - 0.5f) * 2.5f;
        game->ball.speed += BALL_SPEED_INC;
        if (game->ball.speed > BALL_MAX_SPEED) game->ball.speed = BALL_MAX_SPEED;
        game->ball.vx = cosf(angle) * game->ball.speed;
        game->ball.vy = sinf(angle) * game->ball.speed;
        if (game->ball.vx < 1.0f) game->ball.vx = 1.0f;
        game->rally_count++;
        spawn_particles(game, game->ball.x, game->ball.y, 8, COLOR_PADDLE_P1);
    }

    /* AI paddle collision */
    if (game->ball.vx > 0 &&
        game->ball.x + BALL_SIZE >= game->ai.x &&
        game->ball.x + BALL_SIZE <= game->ai.x + PADDLE_W &&
        game->ball.y + BALL_SIZE >= game->ai.y &&
        game->ball.y <= game->ai.y + PADDLE_H) {

        game->ball.x = game->ai.x - BALL_SIZE;
        float hit_pos = (game->ball.y + BALL_SIZE / 2.0f - game->ai.y) / PADDLE_H;
        float angle = 3.14159f - (hit_pos - 0.5f) * 2.5f;
        game->ball.speed += BALL_SPEED_INC;
        if (game->ball.speed > BALL_MAX_SPEED) game->ball.speed = BALL_MAX_SPEED;
        game->ball.vx = cosf(angle) * game->ball.speed;
        game->ball.vy = sinf(angle) * game->ball.speed;
        if (game->ball.vx > -1.0f) game->ball.vx = -1.0f;
        game->rally_count++;
        spawn_particles(game, game->ball.x + BALL_SIZE, game->ball.y, 8, COLOR_PADDLE_P2);
    }

    /* Scoring */
    if (game->ball.x < -BALL_SIZE * 2) {
        game->score_p2++;
        game->shake_timer = SHAKE_FRAMES;
        spawn_particles(game, 0, game->ball.y, 15, COLOR_PADDLE_P2);
        if (game->score_p2 >= WIN_SCORE) {
            game->game_over = true;
        } else {
            reset_ball(game, w, h);
        }
    }
    if (game->ball.x > (float)w + BALL_SIZE) {
        game->score_p1++;
        game->shake_timer = SHAKE_FRAMES;
        spawn_particles(game, (float)w, game->ball.y, 15, COLOR_PADDLE_P1);
        if (game->score_p1 >= WIN_SCORE) {
            game->game_over = true;
        } else {
            reset_ball(game, w, h);
        }
    }

    /* Screen shake */
    if (game->shake_timer > 0) {
        game->shake_timer--;
        game->shake_x = (randf() - 0.5f) * 6.0f * ((float)game->shake_timer / SHAKE_FRAMES);
        game->shake_y = (randf() - 0.5f) * 6.0f * ((float)game->shake_timer / SHAKE_FRAMES);
    } else {
        game->shake_x = game->shake_y = 0;
    }

    /* Update particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].vx;
        game->particles[i].y += game->particles[i].vy;
        game->particles[i].vx *= 0.96f;
        game->particles[i].vy *= 0.96f;
        game->particles[i].life -= 0.025f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }
}

/* ============================================================
 * Rendering - Primary Plane (Background + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Clear background */
    clear_buffer(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Background grid */
    int grid_spacing = 40;
    for (uint32_t y = 0; y < h; y += grid_spacing)
        for (uint32_t x = 0; x < w; x++)
            put_pixel(fb, stride_px, w, h, (int)x, (int)y, COLOR_GRID);
    for (uint32_t x = 0; x < w; x += grid_spacing)
        for (uint32_t y = 0; y < h; y++)
            put_pixel(fb, stride_px, w, h, (int)x, (int)y, COLOR_GRID);

    /* Center line (dashed) */
    int dash_h = 15, gap_h = 10;
    for (uint32_t y = 0; y < h; y++) {
        if (((int)y % (dash_h + gap_h)) < dash_h) {
            int cx = (int)w / 2;
            for (int dx = -1; dx <= 1; dx++)
                put_pixel(fb, stride_px, w, h, cx + dx, (int)y, COLOR_CENTER_LINE);
        }
    }

    /* Scores (large) */
    int score_x1 = (int)w / 4 - 15;
    draw_number(fb, stride_px, w, h, score_x1, 20, game->score_p1, 5, COLOR_SCORE_P1);
    int score_x2 = (int)w * 3 / 4 - 15;
    draw_number(fb, stride_px, w, h, score_x2, 20, game->score_p2, 5, COLOR_SCORE_P2);

    /* Top/bottom border */
    draw_rect(fb, stride_px, w, h, 0, 0, (int)w, 2, 0xFF444466);
    draw_rect(fb, stride_px, w, h, 0, (int)h - 2, (int)w, 2, 0xFF444466);

    /* Rally counter */
    if (game->rally_count > 3 && !game->serving) {
        draw_number(fb, stride_px, w, h, (int)w / 2 - 8, (int)h - 30,
                    game->rally_count, 2, 0xFF666688);
    }

    /* Game Over text */
    if (game->game_over) {
        uint32_t win_color = game->score_p1 >= WIN_SCORE ? COLOR_PADDLE_P1 : COLOR_PADDLE_P2;
        draw_rect(fb, stride_px, w, h, 0, (int)h / 2 - 30, (int)w, 60, 0xFF000000);
        draw_rect(fb, stride_px, w, h, (int)w / 2 - 50, (int)h / 2 - 3, 100, 6, win_color);
    }

    /* Pause indicator */
    if (game->paused) {
        draw_rect(fb, stride_px, w, h, (int)w / 2 - 30, (int)h / 2 - 12, 60, 24, 0xFF000011);
        draw_rect(fb, stride_px, w, h, (int)w / 2 - 10, (int)h / 2 - 6, 6, 12, 0xFFFFFFFF);
        draw_rect(fb, stride_px, w, h, (int)w / 2 + 4, (int)h / 2 - 6, 6, 12, 0xFFFFFFFF);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Ball + Trail + Particles + AI Paddle)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Ball trail */
    for (int i = 0; i < TRAIL_LENGTH; i++) {
        if (!game->trail[i].active) continue;
        int age = (game->trail_idx - i + TRAIL_LENGTH) % TRAIL_LENGTH;
        float alpha = 1.0f - (float)age / TRAIL_LENGTH;
        if (alpha <= 0) continue;
        int ts = (int)(BALL_SIZE * alpha * 0.8f);
        if (ts < 1) ts = 1;
        int tx = (int)game->trail[i].x;
        int ty = (int)game->trail[i].y;
        /* Use alpha in ARGB color */
        uint32_t a = (uint32_t)(alpha * 150);
        uint32_t tc = (a << 24) | (COLOR_BALL_GLOW & 0x00FFFFFF);
        draw_rect(fb, stride_px, w, h, tx, ty, ts, ts, tc);
    }

    /* Ball */
    if (!game->serving || (game->frame_count / 8) % 2) {
        int bx = (int)game->ball.x + BALL_SIZE / 2;
        int by = (int)game->ball.y + BALL_SIZE / 2;
        /* Glow */
        draw_circle(fb, stride_px, w, h, bx, by, BALL_SIZE / 2 + 4, 0x60888800);
        /* Core */
        draw_circle(fb, stride_px, w, h, bx, by, BALL_SIZE / 2, COLOR_BALL);
    }

    /* AI paddle */
    draw_rect(fb, stride_px, w, h,
              (int)game->ai.x, (int)game->ai.y,
              PADDLE_W, PADDLE_H, game->ai.color);
    /* AI paddle glow edge */
    draw_rect(fb, stride_px, w, h,
              (int)game->ai.x - 1, (int)game->ai.y - 1,
              PADDLE_W + 2, 1, 0x80FF4488);
    draw_rect(fb, stride_px, w, h,
              (int)game->ai.x - 1, (int)game->ai.y + PADDLE_H,
              PADDLE_W + 2, 1, 0x80FF4488);

    /* Particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        struct particle *p = &game->particles[i];
        int px = (int)p->x;
        int py = (int)p->y;
        int sz = (int)(p->life * 4) + 1;
        uint32_t a = (uint32_t)(p->life * 255);
        uint32_t pc = (a << 24) | (p->color & 0x00FFFFFF);
        draw_rect(fb, stride_px, w, h, px - sz / 2, py - sz / 2, sz, sz, pc);
    }
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
    int px = (w - PADDLE_W) / 2;
    int py = (h - PADDLE_H) / 2;

    /* Glow */
    draw_rect(fb, stride_px, w, h, px - 2, py - 2, PADDLE_W + 4, PADDLE_H + 4, 0x4000AAFF);
    /* Core paddle */
    draw_rect(fb, stride_px, w, h, px, py, PADDLE_W, PADDLE_H, COLOR_PADDLE_P1);
    /* Highlight edge */
    draw_rect(fb, stride_px, w, h, px, py, PADDLE_W, 2, 0xFF44CCFF);
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

    printf("=== DRM Pong Game (Overlay + Cursor Planes) ===\n");
    printf("Controls: Up/Down or W/S=Move paddle, P=Pause, Space=Restart, ESC/Q=Quit\n\n");

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game state */
    game_init(&game, dev.width, dev.height);

    /* Draw the player paddle sprite once (cursor plane content is static, just moves) */
    render_cursor(&dev.cursor_buf);

    printf("[GAME] First to %d wins!\n", WIN_SCORE);
    printf("[GAME] Starting main loop...\n");

    /* Main game loop */
    struct timespec frame_start, frame_end;
    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        /* Poll input */
        input_poll(&game, dev.height);

        /* Update game state */
        game_update(&game, dev.width, dev.height);

        /* Render primary plane (background + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (ball + trail + particles + AI paddle) into back buffer */
        render_overlay(&dev, &game);

        /* Compute cursor plane position (player paddle movement) */
        int cursor_x = (int)game.player.x - (CURSOR_W - PADDLE_W) / 2;
        int cursor_y = (int)game.player.y - (CURSOR_H - PADDLE_H) / 2;

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

    if (game.game_over) {
        printf("[GAME] Game Over! P1: %d - P2: %d\n", game.score_p1, game.score_p2);
        printf("[GAME] %s wins!\n", game.score_p1 >= WIN_SCORE ? "Player" : "AI");
    }

    printf("[GAME] Shutting down...\n");

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);

    printf("Final Score: P1 %d - P2 %d\n", game.score_p1, game.score_p2);
    return 0;
}
