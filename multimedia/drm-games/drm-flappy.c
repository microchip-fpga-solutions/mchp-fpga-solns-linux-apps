/*
 * drm-flappy.c - Flappy Bird Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background (sky gradient, clouds, ground) + score HUD
 *   - Overlay plane:  Pipes, particles/effects
 *   - Cursor plane:   Bird (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm_flappy drm-flappy.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm_flappy drm-flappy.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm_flappy [/dev/dri/cardN]
 *
 * Controls:
 *   - SPACE / Tap: flap (jump)
 *   - ESC or Q: quit
 *   - SPACE after game over: restart
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

/* Cursor plane dimensions (bird sprite) */
#define CURSOR_W            64
#define CURSOR_H            64

/* Game tuning */
#define GRAVITY             0.4f
#define FLAP_VELOCITY       -6.5f
#define MAX_FALL_SPEED      10.0f
#define BIRD_X_POS          80      /* fixed X position on screen */
#define BIRD_RADIUS         10

#define PIPE_WIDTH          50
#define PIPE_GAP            120     /* vertical gap between top and bottom pipes */
#define PIPE_SPEED          3.0f
#define PIPE_SPACING        200     /* horizontal spacing between pipes */
#define MAX_PIPES           8

#define GROUND_HEIGHT       40

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG_TOP        0xFF4EC0CA
#define COLOR_BG_BOT        0xFF87CEEB
#define COLOR_BIRD          0xFFFFCC00
#define COLOR_BIRD_WING     0xFFFF8800
#define COLOR_BIRD_EYE      0xFF000000
#define COLOR_BIRD_BEAK     0xFFFF4400
#define COLOR_PIPE          0xFF33CC33
#define COLOR_PIPE_DARK     0xFF228B22
#define COLOR_PIPE_LIP      0xFF44DD44
#define COLOR_GROUND        0xFF8B6914
#define COLOR_GROUND_GRASS  0xFF44AA22
#define COLOR_TEXT          0xFFFFFFFF
#define COLOR_SCORE         0xFFFFFFFF
#define COLOR_GAMEOVER      0xFFFF4444

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

struct pipe {
    float x;
    int gap_y;      /* center of the gap */
    bool scored;    /* has the bird passed this pipe */
    bool active;
};

struct bird {
    float y;
    float vy;
    float rotation; /* tilt angle in degrees */
};

struct game_state {
    struct bird bird;
    struct pipe pipes[MAX_PIPES];
    int score;
    int high_score;
    float scroll_offset;
    bool running;
    bool started;       /* has the game started (first flap) */
    bool game_over;
    int death_timer;    /* frames since death for animation */
    uint32_t screen_w, screen_h;
    int ground_y;
    int frame_count;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;

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

    /* Create cursor plane buffer (bird sprite, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned where bird starts) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        BIRD_X_POS - CURSOR_W / 2,
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background (sky gradient, clouds, ground) + score HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Pipes, particles/effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Bird (player-controlled)\n",
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

static void draw_circle(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                         int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            if (dx*dx + dy*dy <= r2)
                put_pixel(fb, stride_px, scr_w, scr_h, cx + dx, cy + dy, color);
}

/* Gradient fill for background */
static void draw_gradient_v(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                             int y0, int y1, uint32_t color_top, uint32_t color_bot) {
    if (y0 < 0) y0 = 0;
    if (y1 > (int)sh) y1 = (int)sh;
    int range = y1 - y0;
    if (range <= 0) return;

    for (int y = y0; y < y1; y++) {
        float t = (float)(y - y0) / (float)range;
        uint8_t r = (uint8_t)((1-t) * ((color_top>>16)&0xFF) + t * ((color_bot>>16)&0xFF));
        uint8_t g = (uint8_t)((1-t) * ((color_top>>8)&0xFF) + t * ((color_bot>>8)&0xFF));
        uint8_t b = (uint8_t)((1-t) * (color_top&0xFF) + t * (color_bot&0xFF));
        uint32_t c = 0xFF000000 | (r << 16) | (g << 8) | b;
        for (uint32_t x = 0; x < sw; x++)
            fb[y * stride_px + x] = c;
    }
}

/* 5x7 digit font */
static const uint8_t font_5x7[][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
};

static void draw_digit(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                        int x, int y, int d, int scale, uint32_t color) {
    if (d < 0 || d > 9) return;
    const uint8_t *g = font_5x7[d];
    for (int r = 0; r < 7; r++)
        for (int c = 0; c < 5; c++)
            if (g[r] & (0x10 >> c))
                draw_rect(fb, stride_px, sw, sh, x+c*scale, y+r*scale, scale, scale, color);
}

static void draw_number(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                         int x, int y, int num, int scale, uint32_t color) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int off = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride_px, sw, sh, x+off, y, buf[i]-'0', scale, color);
        off += 6*scale;
    }
}

/* Draw number centered */
static void draw_number_centered(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                                   int cx, int y, int num, int scale, uint32_t color) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int len = strlen(buf);
    int total_w = len * 6 * scale - scale;
    draw_number(fb, stride_px, sw, sh, cx - total_w / 2, y, num, scale, color);
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64]; input_count = 0;
    for (int i = 0; i < 8 && input_count < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("[INPUT] %s: %s\n", path, name);
            input_fds[input_count++] = fd;
        }
    }
    printf("[INPUT] Opened %d input devices\n", input_count);
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static bool input_process(struct game_state *game) {
    struct input_event ev;
    bool flap = false;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_KEY && ev.value == 1) {
                switch (ev.code) {
                case KEY_SPACE:
                case KEY_UP:
                case BTN_TOUCH:
                    flap = true;
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

    return flap;
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void spawn_pipe(struct game_state *game, float x) {
    for (int i = 0; i < MAX_PIPES; i++) {
        if (!game->pipes[i].active) {
            game->pipes[i].x = x;
            /* Random gap position, keeping it reasonable */
            int min_gap_y = PIPE_GAP / 2 + 40;
            int max_gap_y = game->ground_y - PIPE_GAP / 2 - 40;
            game->pipes[i].gap_y = min_gap_y + rand() % (max_gap_y - min_gap_y);
            game->pipes[i].scored = false;
            game->pipes[i].active = true;
            return;
        }
    }
}

static void game_init(struct game_state *game, uint32_t sw, uint32_t sh) {
    memset(game, 0, sizeof(*game));
    game->screen_w = sw;
    game->screen_h = sh;
    game->ground_y = (int)sh - GROUND_HEIGHT;
    game->running = true;
    game->started = false;
    game->game_over = false;

    game->bird.y = (float)sh / 2.0f;
    game->bird.vy = 0;
    game->bird.rotation = 0;

    game->score = 0;
    game->high_score = 0;
    game->frame_count = 0;

    srand((unsigned)time(NULL));

    /* Spawn initial pipes */
    for (int i = 0; i < 3; i++) {
        spawn_pipe(game, (float)sw + i * PIPE_SPACING);
    }
}

static void game_restart(struct game_state *game) {
    if (game->score > game->high_score)
        game->high_score = game->score;

    game->bird.y = (float)game->screen_h / 2.0f;
    game->bird.vy = 0;
    game->bird.rotation = 0;
    game->score = 0;
    game->started = false;
    game->game_over = false;
    game->death_timer = 0;

    /* Reset pipes */
    for (int i = 0; i < MAX_PIPES; i++)
        game->pipes[i].active = false;
    for (int i = 0; i < 3; i++)
        spawn_pipe(game, (float)game->screen_w + i * PIPE_SPACING);
}

static void game_update(struct game_state *game, bool flap) {
    game->frame_count++;

    if (game->game_over) {
        game->death_timer++;
        if (flap && game->death_timer > 30) {
            game_restart(game);
        }
        return;
    }

    if (!game->started) {
        /* Bobbing animation before start */
        game->bird.y = (float)game->screen_h / 2.0f +
                       sinf((float)game->frame_count * 0.05f) * 10.0f;
        if (flap) {
            game->started = true;
            game->bird.vy = FLAP_VELOCITY;
        }
        return;
    }

    /* Apply flap */
    if (flap) {
        game->bird.vy = FLAP_VELOCITY;
    }

    /* Apply gravity */
    game->bird.vy += GRAVITY;
    if (game->bird.vy > MAX_FALL_SPEED)
        game->bird.vy = MAX_FALL_SPEED;

    game->bird.y += game->bird.vy;

    /* Bird rotation (tilt based on velocity) */
    game->bird.rotation = game->bird.vy * 3.0f;
    if (game->bird.rotation > 45.0f) game->bird.rotation = 45.0f;
    if (game->bird.rotation < -30.0f) game->bird.rotation = -30.0f;

    /* Move pipes */
    float rightmost_x = 0;
    for (int i = 0; i < MAX_PIPES; i++) {
        if (!game->pipes[i].active) continue;
        game->pipes[i].x -= PIPE_SPEED;

        if (game->pipes[i].x > rightmost_x)
            rightmost_x = game->pipes[i].x;

        /* Score when bird passes pipe */
        if (!game->pipes[i].scored && game->pipes[i].x + PIPE_WIDTH < BIRD_X_POS) {
            game->pipes[i].scored = true;
            game->score++;
        }

        /* Remove off-screen pipes */
        if (game->pipes[i].x + PIPE_WIDTH < -10) {
            game->pipes[i].active = false;
        }
    }

    /* Spawn new pipes */
    if (rightmost_x < (float)game->screen_w - PIPE_SPACING + PIPE_WIDTH) {
        spawn_pipe(game, rightmost_x + PIPE_SPACING);
    }

    /* Collision detection */
    int bird_cx = BIRD_X_POS;
    int bird_cy = (int)game->bird.y;

    /* Ground/ceiling collision */
    if (bird_cy + BIRD_RADIUS >= game->ground_y || bird_cy - BIRD_RADIUS <= 0) {
        game->game_over = true;
        if (game->score > game->high_score)
            game->high_score = game->score;
        printf("[GAME] Game Over! Score: %d | High: %d\n", game->score, game->high_score);
        return;
    }

    /* Pipe collision (simple AABB with circle approximation) */
    for (int i = 0; i < MAX_PIPES; i++) {
        if (!game->pipes[i].active) continue;

        int pipe_left = (int)game->pipes[i].x;
        int pipe_right = pipe_left + PIPE_WIDTH;
        int gap_top = game->pipes[i].gap_y - PIPE_GAP / 2;
        int gap_bot = game->pipes[i].gap_y + PIPE_GAP / 2;

        /* Is bird horizontally overlapping with pipe? */
        if (bird_cx + BIRD_RADIUS > pipe_left && bird_cx - BIRD_RADIUS < pipe_right) {
            /* Is bird outside the gap? */
            if (bird_cy - BIRD_RADIUS < gap_top || bird_cy + BIRD_RADIUS > gap_bot) {
                game->game_over = true;
                if (game->score > game->high_score)
                    game->high_score = game->score;
                printf("[GAME] Game Over! Score: %d | High: %d\n",
                       game->score, game->high_score);
                return;
            }
        }
    }

    /* Scroll ground */
    game->scroll_offset += PIPE_SPEED;
    if (game->scroll_offset >= 20.0f)
        game->scroll_offset -= 20.0f;
}

/* ============================================================
 * Rendering - Primary Plane (Background + Ground + Score HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t sw = dev->width;
    uint32_t sh = dev->height;

    /* Sky gradient */
    draw_gradient_v(fb, stride_px, sw, sh, 0, game->ground_y, COLOR_BG_TOP, COLOR_BG_BOT);

    /* Draw ground */
    draw_rect(fb, stride_px, sw, sh, 0, game->ground_y, (int)sw, GROUND_HEIGHT, COLOR_GROUND);
    /* Grass strip */
    draw_rect(fb, stride_px, sw, sh, 0, game->ground_y, (int)sw, 4, COLOR_GROUND_GRASS);
    /* Ground pattern (scrolling) */
    int scroll = (int)game->scroll_offset;
    for (int x = -scroll; x < (int)sw; x += 20) {
        draw_rect(fb, stride_px, sw, sh, x, game->ground_y + 6, 10, 2, 0xFF7A5C10);
    }

    /* Draw score (centered, large) */
    draw_number_centered(fb, stride_px, sw, sh, (int)sw / 2, 30, game->score, 4, COLOR_SCORE);
    /* Score shadow */
    draw_number_centered(fb, stride_px, sw, sh, (int)sw / 2 + 2, 32, game->score, 4, 0x44000000);

    /* Game over overlay */
    if (game->game_over && game->death_timer > 15) {
        /* Semi-transparent overlay */
        draw_rect(fb, stride_px, sw, sh,
                  (int)sw / 4, (int)sh / 2 - 50,
                  (int)sw / 2, 100, 0xCC000000);

        /* Score display */
        draw_number_centered(fb, stride_px, sw, sh, (int)sw / 2, (int)sh / 2 - 30,
                             game->score, 3, COLOR_SCORE);

        /* High score */
        draw_number_centered(fb, stride_px, sw, sh, (int)sw / 2, (int)sh / 2 + 5,
                             game->high_score, 2, 0xFF88FF88);

        /* "Tap" blinker */
        if ((game->death_timer / 20) % 2 == 0 && game->death_timer > 30) {
            draw_rect(fb, stride_px, sw, sh,
                      (int)sw / 2 - 15, (int)sh / 2 + 30, 30, 6, COLOR_TEXT);
        }
    }

    /* "Tap to start" blinker before game starts */
    if (!game->started && !game->game_over) {
        if ((game->frame_count / 30) % 2 == 0) {
            draw_rect(fb, stride_px, sw, sh,
                      (int)sw / 2 - 20, (int)sh / 2 + 50, 40, 6, COLOR_TEXT);
        }
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Pipes + Particles/Effects)
 * ============================================================ */

static void draw_pipe(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                       int x, int gap_y, int ground_y) {
    int gap_top = gap_y - PIPE_GAP / 2;
    int gap_bot = gap_y + PIPE_GAP / 2;
    int lip_h = 16;
    int lip_overhang = 4;

    /* Top pipe */
    draw_rect(fb, stride_px, sw, sh, x, 0, PIPE_WIDTH, gap_top - lip_h, COLOR_PIPE);
    /* Top pipe darker edge */
    draw_rect(fb, stride_px, sw, sh, x, 0, 4, gap_top - lip_h, COLOR_PIPE_DARK);
    draw_rect(fb, stride_px, sw, sh, x + PIPE_WIDTH - 4, 0, 4, gap_top - lip_h, COLOR_PIPE_DARK);
    /* Top pipe lip */
    draw_rect(fb, stride_px, sw, sh, x - lip_overhang, gap_top - lip_h,
              PIPE_WIDTH + lip_overhang * 2, lip_h, COLOR_PIPE_LIP);
    draw_rect(fb, stride_px, sw, sh, x - lip_overhang, gap_top - lip_h,
              PIPE_WIDTH + lip_overhang * 2, 3, COLOR_PIPE_DARK);

    /* Bottom pipe */
    draw_rect(fb, stride_px, sw, sh, x, gap_bot + lip_h, PIPE_WIDTH, ground_y - gap_bot - lip_h,
              COLOR_PIPE);
    draw_rect(fb, stride_px, sw, sh, x, gap_bot + lip_h, 4, ground_y - gap_bot - lip_h,
              COLOR_PIPE_DARK);
    draw_rect(fb, stride_px, sw, sh, x + PIPE_WIDTH - 4, gap_bot + lip_h, 4,
              ground_y - gap_bot - lip_h, COLOR_PIPE_DARK);
    /* Bottom pipe lip */
    draw_rect(fb, stride_px, sw, sh, x - lip_overhang, gap_bot,
              PIPE_WIDTH + lip_overhang * 2, lip_h, COLOR_PIPE_LIP);
    draw_rect(fb, stride_px, sw, sh, x - lip_overhang, gap_bot + lip_h - 3,
              PIPE_WIDTH + lip_overhang * 2, 3, COLOR_PIPE_DARK);
}

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw pipes */
    for (int i = 0; i < MAX_PIPES; i++) {
        if (!game->pipes[i].active) continue;
        draw_pipe(fb, stride_px, w, h,
                  (int)game->pipes[i].x, game->pipes[i].gap_y, game->ground_y);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Bird)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf, float rotation, int frame) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    /* Draw bird centered in cursor buffer */
    int cx = (int)w / 2;
    int cy = (int)h / 2;

    /* Body */
    draw_circle(fb, stride_px, w, h, cx, cy, BIRD_RADIUS, COLOR_BIRD);

    /* Wing (animated) */
    int wing_offset = (frame / 6) % 3;
    int wing_y = cy + (wing_offset == 0 ? -2 : wing_offset == 1 ? 0 : 2);
    draw_rect(fb, stride_px, w, h, cx - BIRD_RADIUS - 2, wing_y - 2, 8, 5, COLOR_BIRD_WING);

    /* Eye */
    draw_circle(fb, stride_px, w, h, cx + 4, cy - 3, 3, 0xFFFFFFFF);
    draw_rect(fb, stride_px, w, h, cx + 5, cy - 3, 2, 2, COLOR_BIRD_EYE);

    /* Beak */
    draw_rect(fb, stride_px, w, h, cx + BIRD_RADIUS - 2, cy - 1, 8, 4, COLOR_BIRD_BEAK);

    /* Tilt indicator (small tail feather if falling) */
    if (rotation > 10.0f) {
        draw_rect(fb, stride_px, w, h, cx - BIRD_RADIUS - 4, cy - 4, 5, 3, COLOR_BIRD_WING);
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

    printf("===========================================\n");
    printf("  DRM Flappy Bird (Overlay + Cursor Planes)\n");
    printf("  Built with libdrm (KMS/DRM)\n");
    printf("===========================================\n\n");

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM.\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game state */
    game_init(&game, dev.width, dev.height);

    printf("[GAME] Controls:\n");
    printf("  SPACE / Tap : Flap\n");
    printf("  ESC / Q     : Quit\n\n");
    printf("[GAME] Starting main loop...\n");

    /* Main game loop */
    uint64_t frame_start, frame_end, frame_elapsed;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        /* Poll input */
        bool flap = input_process(&game);

        /* Update game state */
        game_update(&game, flap);

        /* Render primary plane (background + ground + score HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (pipes + effects) into back buffer */
        render_overlay(&dev, &game);

        /* Compute cursor plane position (bird) */
        int cursor_x, cursor_y;
        if (!game.game_over || game.death_timer < 30) {
            int bird_cy = (int)game.bird.y;
            if (game.game_over) {
                /* Death fall animation */
                bird_cy += game.death_timer * 2;
            }
            render_cursor(&dev.cursor_buf, game.bird.rotation, game.frame_count);
            cursor_x = BIRD_X_POS - CURSOR_W / 2;
            cursor_y = bird_cy - CURSOR_H / 2;
        } else {
            /* Hide bird after death animation by moving off-screen */
            cursor_x = -CURSOR_W;
            cursor_y = -CURSOR_H;
        }

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
    }

    printf("[GAME] Final Score: %d | High Score: %d\n", game.score, game.high_score);
    printf("[GAME] Shutting down...\n");

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);

    return 0;
}
