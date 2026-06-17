/*
 * drm-asteroids.c - Asteroids Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background starfield + score/lives HUD
 *   - Overlay plane:  Asteroids, bullets, explosions/particles
 *   - Cursor plane:   Player ship (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-asteroids drm-asteroids.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-asteroids drm-asteroids.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-asteroids [/dev/dri/cardN]
 *
 * Features:
 *   - Rotating ship rendered with vector-style lines
 *   - Asteroids that split into smaller pieces when shot
 *   - Particle explosion effects with multiple colors
 *   - Star field background with parallax depth layers
 *   - Thrust flame animation on the ship
 *   - Screen wrapping for all objects
 *   - Progressive difficulty with waves
 *
 * Controls:
 *   - Left/Right: rotate ship
 *   - Up: thrust
 *   - Space: fire
 *   - P: pause
 *   - ESC/Q: quit
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

/* Cursor plane dimensions (player ship) */
#define CURSOR_W            64
#define CURSOR_H            64

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define PI                  3.14159265f
#define TWO_PI              6.28318530f

#define SHIP_RADIUS         14
#define SHIP_TURN_SPEED     0.07f
#define SHIP_THRUST         0.15f
#define SHIP_MAX_SPEED      6.0f
#define SHIP_FRICTION       0.99f

#define MAX_BULLETS         20
#define BULLET_SPEED        8.0f
#define BULLET_LIFE         50
#define FIRE_COOLDOWN       8

#define MAX_ASTEROIDS       24
#define ASTEROID_SPEED_MIN  0.5f
#define ASTEROID_SPEED_MAX  2.5f
#define ASTEROID_LARGE      3
#define ASTEROID_MEDIUM     2
#define ASTEROID_SMALL      1

#define MAX_PARTICLES       120
#define MAX_STARS           80

#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG            0xFF000011
#define COLOR_SHIP          0xFF00FFAA
#define COLOR_SHIP_THRUST   0xFFFF8800
#define COLOR_BULLET        0xFFFFFF44
#define COLOR_ASTEROID      0xFFAAAA88
#define COLOR_ASTEROID_EDGE 0xFFDDDDBB

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

struct vec2 { float x, y; };

struct ship {
    struct vec2 pos;
    struct vec2 vel;
    float angle;
    bool thrusting;
    bool alive;
    int respawn_timer;
    int fire_cooldown;
};

struct bullet {
    struct vec2 pos;
    struct vec2 vel;
    int life;
    bool active;
};

struct asteroid {
    struct vec2 pos;
    struct vec2 vel;
    float angle;
    float rot_speed;
    int size;  /* LARGE=3, MEDIUM=2, SMALL=1 */
    float radius;
    float shape[12]; /* vertex offsets for irregular shape */
    int num_verts;
    bool active;
};

struct particle {
    struct vec2 pos;
    struct vec2 vel;
    float life;
    float max_life;
    uint32_t color;
    bool active;
};

struct star {
    float x, y;
    float brightness;
    int layer; /* 0=far, 1=mid, 2=near */
};

struct game_state {
    struct ship ship;
    struct bullet bullets[MAX_BULLETS];
    struct asteroid asteroids[MAX_ASTEROIDS];
    struct particle particles[MAX_PARTICLES];
    struct star stars[MAX_STARS];
    int score;
    int lives;
    int wave;
    bool running;
    bool paused;
    bool game_over;
    int frame_count;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false, key_right = false, key_up = false, key_fire = false;

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
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static void sleep_ns(uint64_t ns) {
    struct timespec ts = {
        .tv_sec = ns / 1000000000ULL,
        .tv_nsec = ns % 1000000000ULL
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

    /* Create cursor plane buffer (player ship, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at ship starting location) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (int32_t)((dev->width / 2) - (CURSOR_W / 2)),
                        (int32_t)((dev->height / 2) - (CURSOR_H / 2)),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background starfield + score/lives HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Asteroids, bullets, explosions/particles\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player ship (player-controlled)\n",
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

static inline void put_pixel(uint32_t *fb, uint32_t stride, uint32_t w, uint32_t h,
                             int x, int y, uint32_t c) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h)
        fb[y * stride + x] = c;
}

static inline uint32_t blend_add(uint32_t dst, uint32_t src) {
    uint32_t r = ((dst >> 16) & 0xFF) + ((src >> 16) & 0xFF);
    uint32_t g = ((dst >> 8) & 0xFF) + ((src >> 8) & 0xFF);
    uint32_t b = (dst & 0xFF) + (src & 0xFF);
    if (r > 255) r = 255; if (g > 255) g = 255; if (b > 255) b = 255;
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

/* Bresenham line */
static void draw_line(uint32_t *fb, uint32_t stride, uint32_t w, uint32_t h,
                      int x0, int y0, int x1, int y1, uint32_t color) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (1) {
        put_pixel(fb, stride, w, h, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

/* Thick line */
static void draw_line_thick(uint32_t *fb, uint32_t stride, uint32_t w, uint32_t h,
                             int x0, int y0, int x1, int y1, int thickness, uint32_t color) {
    for (int t = -thickness/2; t <= thickness/2; t++) {
        draw_line(fb, stride, w, h, x0+t, y0, x1+t, y1, color);
        draw_line(fb, stride, w, h, x0, y0+t, x1, y1+t, color);
    }
}

static void draw_rect(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                       int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = (x+w) > (int)sw ? (int)sw : (x+w);
    int y1 = (y+h) > (int)sh ? (int)sh : (y+h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = color;
}

static void draw_circle(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int cx, int cy, int r, uint32_t color) {
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r2)
                put_pixel(fb, stride, sw, sh, cx+dx, cy+dy, color);
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
}

/* Font */
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

static void draw_number(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int x, int y, int num, int s, uint32_t c) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int off = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            int d = buf[i] - '0';
            for (int row = 0; row < 7; row++)
                for (int col = 0; col < 5; col++)
                    if (font_5x7[d][row] & (0x10 >> col))
                        draw_rect(fb, stride, sw, sh, x+off+col*s, y+row*s, s, s, c);
        }
        off += 6*s;
    }
}

/* ============================================================
 * Input
 * ============================================================ */

static void input_init(void) {
    char path[64]; input_count = 0;
    for (int i = 0; i < 8 && input_count < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) input_fds[input_count++] = fd;
    }
    printf("[INPUT] Opened %d input devices\n", input_count);
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static void input_process(struct game_state *game) {
    struct input_event ev;
    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2);
                switch (ev.code) {
                case KEY_LEFT:  key_left = pressed; break;
                case KEY_RIGHT: key_right = pressed; break;
                case KEY_UP:    key_up = pressed; break;
                case KEY_SPACE: key_fire = pressed; break;
                case KEY_P: if (ev.value == 1) game->paused = !game->paused; break;
                case KEY_R:
                    if (ev.value == 1 && game->game_over) {
                        game->game_over = false;
                        game->score = 0; game->lives = 3; game->wave = 0;
                        game->ship.alive = true;
                        game->ship.pos.x = game->ship.pos.y = 0; /* will be reset */
                        for (int a = 0; a < MAX_ASTEROIDS; a++) game->asteroids[a].active = false;
                    }
                    break;
                case KEY_ESC: case KEY_Q: if (ev.value == 1) g_quit = true; break;
                default: break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static float wrap_coord(float v, float max) {
    if (v < 0) return v + max;
    if (v >= max) return v - max;
    return v;
}

static void spawn_particle(struct game_state *game, float x, float y,
                            float vx, float vy, float life, uint32_t color) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) {
            game->particles[i] = (struct particle){
                .pos = {x, y}, .vel = {vx, vy},
                .life = life, .max_life = life,
                .color = color, .active = true
            };
            return;
        }
    }
}

static void spawn_explosion(struct game_state *game, float x, float y, int count, float spread) {
    uint32_t colors[] = {0xFFFF4400, 0xFFFFAA00, 0xFFFFFF44, 0xFFFF6600, 0xFFFFCC00};
    for (int i = 0; i < count; i++) {
        float angle = randf() * TWO_PI;
        float speed = randf() * spread;
        spawn_particle(game, x, y,
                       cosf(angle) * speed, sinf(angle) * speed,
                       0.5f + randf() * 0.8f, colors[i % 5]);
    }
}

static void init_asteroid(struct asteroid *a, float x, float y, int size) {
    a->pos = (struct vec2){x, y};
    float speed = ASTEROID_SPEED_MIN + randf() * (ASTEROID_SPEED_MAX - ASTEROID_SPEED_MIN);
    float angle = randf() * TWO_PI;
    a->vel = (struct vec2){cosf(angle) * speed, sinf(angle) * speed};
    a->angle = randf() * TWO_PI;
    a->rot_speed = (randf() - 0.5f) * 0.05f;
    a->size = size;
    a->radius = size == ASTEROID_LARGE ? 30 : (size == ASTEROID_MEDIUM ? 18 : 10);
    a->num_verts = 8 + rand() % 4;
    for (int i = 0; i < a->num_verts; i++)
        a->shape[i] = 0.7f + randf() * 0.6f; /* random radius variations */
    a->active = true;
}

static void spawn_wave(struct game_state *game, uint32_t w, uint32_t h) {
    game->wave++;
    int count = 3 + game->wave;
    if (count > MAX_ASTEROIDS / 2) count = MAX_ASTEROIDS / 2;

    for (int i = 0; i < count; i++) {
        for (int a = 0; a < MAX_ASTEROIDS; a++) {
            if (!game->asteroids[a].active) {
                float x, y;
                /* Spawn at edges away from player */
                if (rand() % 2) {
                    x = (rand() % 2) ? 0 : (float)w;
                    y = randf() * h;
                } else {
                    x = randf() * w;
                    y = (rand() % 2) ? 0 : (float)h;
                }
                init_asteroid(&game->asteroids[a], x, y, ASTEROID_LARGE);
                break;
            }
        }
    }
}

static void game_init(struct game_state *game, uint32_t w, uint32_t h) {
    memset(game, 0, sizeof(*game));
    srand((unsigned)time(NULL));

    game->ship.pos = (struct vec2){w / 2.0f, h / 2.0f};
    game->ship.angle = -PI / 2.0f;
    game->ship.alive = true;
    game->lives = 3;
    game->running = true;

    /* Stars */
    for (int i = 0; i < MAX_STARS; i++) {
        game->stars[i].x = randf() * w;
        game->stars[i].y = randf() * h;
        game->stars[i].brightness = 0.3f + randf() * 0.7f;
        game->stars[i].layer = rand() % 3;
    }

    spawn_wave(game, w, h);
}

static void game_update(struct game_state *game, uint32_t w, uint32_t h) {
    if (game->paused || game->game_over) return;
    game->frame_count++;

    struct ship *s = &game->ship;

    if (s->alive) {
        /* Rotation */
        if (key_left) s->angle -= SHIP_TURN_SPEED;
        if (key_right) s->angle += SHIP_TURN_SPEED;

        /* Thrust */
        s->thrusting = key_up;
        if (s->thrusting) {
            s->vel.x += cosf(s->angle) * SHIP_THRUST;
            s->vel.y += sinf(s->angle) * SHIP_THRUST;
            /* Thrust particles */
            if (game->frame_count % 3 == 0) {
                float bx = s->pos.x - cosf(s->angle) * SHIP_RADIUS;
                float by = s->pos.y - sinf(s->angle) * SHIP_RADIUS;
                spawn_particle(game, bx, by,
                              -cosf(s->angle) * 2 + (randf()-0.5f),
                              -sinf(s->angle) * 2 + (randf()-0.5f),
                              0.3f, COLOR_SHIP_THRUST);
            }
        }

        /* Speed limit */
        float speed = sqrtf(s->vel.x * s->vel.x + s->vel.y * s->vel.y);
        if (speed > SHIP_MAX_SPEED) {
            s->vel.x = s->vel.x / speed * SHIP_MAX_SPEED;
            s->vel.y = s->vel.y / speed * SHIP_MAX_SPEED;
        }

        /* Friction */
        s->vel.x *= SHIP_FRICTION;
        s->vel.y *= SHIP_FRICTION;

        /* Move and wrap */
        s->pos.x = wrap_coord(s->pos.x + s->vel.x, (float)w);
        s->pos.y = wrap_coord(s->pos.y + s->vel.y, (float)h);

        /* Fire */
        if (key_fire && s->fire_cooldown <= 0) {
            for (int i = 0; i < MAX_BULLETS; i++) {
                if (!game->bullets[i].active) {
                    game->bullets[i].pos.x = s->pos.x + cosf(s->angle) * SHIP_RADIUS;
                    game->bullets[i].pos.y = s->pos.y + sinf(s->angle) * SHIP_RADIUS;
                    game->bullets[i].vel.x = cosf(s->angle) * BULLET_SPEED + s->vel.x * 0.3f;
                    game->bullets[i].vel.y = sinf(s->angle) * BULLET_SPEED + s->vel.y * 0.3f;
                    game->bullets[i].life = BULLET_LIFE;
                    game->bullets[i].active = true;
                    s->fire_cooldown = FIRE_COOLDOWN;
                    break;
                }
            }
        }
        if (s->fire_cooldown > 0) s->fire_cooldown--;
    } else {
        s->respawn_timer--;
        if (s->respawn_timer <= 0) {
            s->pos = (struct vec2){w / 2.0f, h / 2.0f};
            s->vel = (struct vec2){0, 0};
            s->angle = -PI / 2.0f;
            s->alive = true;
        }
    }

    /* Update bullets */
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!game->bullets[i].active) continue;
        game->bullets[i].pos.x = wrap_coord(game->bullets[i].pos.x + game->bullets[i].vel.x, (float)w);
        game->bullets[i].pos.y = wrap_coord(game->bullets[i].pos.y + game->bullets[i].vel.y, (float)h);
        game->bullets[i].life--;
        if (game->bullets[i].life <= 0) game->bullets[i].active = false;
    }

    /* Update asteroids */
    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        if (!game->asteroids[i].active) continue;
        struct asteroid *a = &game->asteroids[i];
        a->pos.x = wrap_coord(a->pos.x + a->vel.x, (float)w);
        a->pos.y = wrap_coord(a->pos.y + a->vel.y, (float)h);
        a->angle += a->rot_speed;
    }

    /* Bullet-asteroid collision */
    for (int b = 0; b < MAX_BULLETS; b++) {
        if (!game->bullets[b].active) continue;
        for (int a = 0; a < MAX_ASTEROIDS; a++) {
            if (!game->asteroids[a].active) continue;
            float dx = game->bullets[b].pos.x - game->asteroids[a].pos.x;
            float dy = game->bullets[b].pos.y - game->asteroids[a].pos.y;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist < game->asteroids[a].radius) {
                /* Hit! */
                game->bullets[b].active = false;
                spawn_explosion(game, game->asteroids[a].pos.x,
                               game->asteroids[a].pos.y,
                               game->asteroids[a].size * 8, 3.0f);

                /* Score */
                game->score += (4 - game->asteroids[a].size) * 50;

                /* Split */
                if (game->asteroids[a].size > ASTEROID_SMALL) {
                    int new_size = game->asteroids[a].size - 1;
                    float ax = game->asteroids[a].pos.x;
                    float ay = game->asteroids[a].pos.y;
                    game->asteroids[a].active = false;
                    for (int k = 0; k < 2; k++) {
                        for (int n = 0; n < MAX_ASTEROIDS; n++) {
                            if (!game->asteroids[n].active) {
                                init_asteroid(&game->asteroids[n], ax, ay, new_size);
                                break;
                            }
                        }
                    }
                } else {
                    game->asteroids[a].active = false;
                }
                break;
            }
        }
    }

    /* Ship-asteroid collision */
    if (s->alive) {
        for (int a = 0; a < MAX_ASTEROIDS; a++) {
            if (!game->asteroids[a].active) continue;
            float dx = s->pos.x - game->asteroids[a].pos.x;
            float dy = s->pos.y - game->asteroids[a].pos.y;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist < game->asteroids[a].radius + SHIP_RADIUS * 0.7f) {
                /* Ship destroyed */
                spawn_explosion(game, s->pos.x, s->pos.y, 20, 4.0f);
                s->alive = false;
                s->respawn_timer = 120;
                game->lives--;
                if (game->lives <= 0) {
                    game->game_over = true;
                    printf("[GAME] Game Over! Score: %d (Wave %d)\n", game->score, game->wave);
                }
                break;
            }
        }
    }

    /* Check if wave cleared */
    bool any_active = false;
    for (int a = 0; a < MAX_ASTEROIDS; a++)
        if (game->asteroids[a].active) { any_active = true; break; }
    if (!any_active)
        spawn_wave(game, w, h);

    /* Update particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].pos.x += game->particles[i].vel.x;
        game->particles[i].pos.y += game->particles[i].vel.y;
        game->particles[i].vel.x *= 0.97f;
        game->particles[i].vel.y *= 0.97f;
        game->particles[i].life -= 0.02f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }

    /* Twinkle stars */
    for (int i = 0; i < MAX_STARS; i++) {
        game->stars[i].brightness += (randf() - 0.5f) * 0.05f;
        if (game->stars[i].brightness < 0.2f) game->stars[i].brightness = 0.2f;
        if (game->stars[i].brightness > 1.0f) game->stars[i].brightness = 1.0f;
    }
}

/* ============================================================
 * Rendering - Primary Plane (Background starfield + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Clear to space */
    clear_buffer(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Stars */
    for (int i = 0; i < MAX_STARS; i++) {
        int sx = (int)game->stars[i].x;
        int sy = (int)game->stars[i].y;
        float b = game->stars[i].brightness;
        uint32_t v = (uint32_t)(b * 255);
        uint32_t c = 0xFF000000 | (v << 16) | (v << 8) | v;
        put_pixel(fb, stride, w, h, sx, sy, c);
        if (game->stars[i].layer >= 1)
            put_pixel(fb, stride, w, h, sx+1, sy, c);
        if (game->stars[i].layer >= 2) {
            put_pixel(fb, stride, w, h, sx, sy+1, c);
            put_pixel(fb, stride, w, h, sx+1, sy+1, c);
        }
    }

    /* HUD */
    draw_rect(fb, stride, w, h, 0, 0, (int)w, 28, 0x88000022);
    draw_number(fb, stride, w, h, 10, 6, game->score, 3, 0xFF00FF88);
    /* Lives as small triangles */
    for (int i = 0; i < game->lives; i++) {
        int lx = (int)w - 30 - i * 22;
        draw_line(fb, stride, w, h, lx, 6, lx-6, 18, 0xFF00FFAA);
        draw_line(fb, stride, w, h, lx, 6, lx+6, 18, 0xFF00FFAA);
        draw_line(fb, stride, w, h, lx-6, 18, lx+6, 18, 0xFF00FFAA);
    }
    /* Wave indicator */
    draw_number(fb, stride, w, h, (int)w/2 - 10, 6, game->wave, 2, 0xFF8888FF);

    /* Game Over */
    if (game->game_over) {
        draw_rect(fb, stride, w, h, 0, (int)h/2 - 25, (int)w, 50, 0xCC000011);
        draw_number(fb, stride, w, h, (int)w/2 - 40, (int)h/2 - 8, game->score, 3, 0xFFFF4444);
    }

    /* Pause */
    if (game->paused) {
        draw_rect(fb, stride, w, h, (int)w/2 - 30, (int)h/2 - 12, 60, 24, 0xCC000022);
        draw_rect(fb, stride, w, h, (int)w/2 - 10, (int)h/2 - 6, 6, 12, COLOR_SHIP);
        draw_rect(fb, stride, w, h, (int)w/2 + 4, (int)h/2 - 6, 6, 12, COLOR_SHIP);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Asteroids + Bullets + Particles)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Asteroids */
    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        if (!game->asteroids[i].active) continue;
        struct asteroid *a = &game->asteroids[i];
        int cx = (int)a->pos.x, cy = (int)a->pos.y;
        int nv = a->num_verts;

        /* Draw polygon edges */
        for (int v = 0; v < nv; v++) {
            int next = (v + 1) % nv;
            float a1 = a->angle + TWO_PI * v / nv;
            float a2 = a->angle + TWO_PI * next / nv;
            int x0 = cx + (int)(cosf(a1) * a->radius * a->shape[v]);
            int y0 = cy + (int)(sinf(a1) * a->radius * a->shape[v]);
            int x1 = cx + (int)(cosf(a2) * a->radius * a->shape[next]);
            int y1 = cy + (int)(sinf(a2) * a->radius * a->shape[next]);
            draw_line_thick(fb, stride, w, h, x0, y0, x1, y1, 2, COLOR_ASTEROID_EDGE);
        }
        /* Interior fill (simplified) */
        draw_circle(fb, stride, w, h, cx, cy, (int)(a->radius * 0.6f), COLOR_ASTEROID);
    }

    /* Bullets */
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!game->bullets[i].active) continue;
        int bx = (int)game->bullets[i].pos.x;
        int by = (int)game->bullets[i].pos.y;
        /* Glowing bullet */
        draw_circle(fb, stride, w, h, bx, by, 3, COLOR_BULLET);
        draw_circle(fb, stride, w, h, bx, by, 1, 0xFFFFFFFF);
    }

    /* Particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        struct particle *p = &game->particles[i];
        int px = (int)p->pos.x, py = (int)p->pos.y;
        float alpha = p->life / p->max_life;
        int sz = (int)(alpha * 3) + 1;
        uint32_t av = (uint32_t)(alpha * 255);
        uint32_t c = (av << 24) | (p->color & 0x00FFFFFF);
        for (int dy = -sz; dy <= sz; dy++)
            for (int dx = -sz; dx <= sz; dx++)
                if (dx*dx + dy*dy <= sz*sz) {
                    int fx = px+dx, fy = py+dy;
                    if (fx >= 0 && fx < (int)w && fy >= 0 && fy < (int)h)
                        fb[fy*stride+fx] = c;
                }
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Player Ship)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf, struct ship *ship, int frame_count) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    if (!ship->alive) return;

    float a = ship->angle;
    int cx = (int)(w / 2);
    int cy = (int)(h / 2);

    /* Ship triangle (3 points) */
    int nose_x = cx + (int)(cosf(a) * SHIP_RADIUS);
    int nose_y = cy + (int)(sinf(a) * SHIP_RADIUS);
    int left_x = cx + (int)(cosf(a + 2.4f) * SHIP_RADIUS * 0.8f);
    int left_y = cy + (int)(sinf(a + 2.4f) * SHIP_RADIUS * 0.8f);
    int right_x = cx + (int)(cosf(a - 2.4f) * SHIP_RADIUS * 0.8f);
    int right_y = cy + (int)(sinf(a - 2.4f) * SHIP_RADIUS * 0.8f);

    draw_line_thick(fb, stride, w, h, nose_x, nose_y, left_x, left_y, 2, COLOR_SHIP);
    draw_line_thick(fb, stride, w, h, nose_x, nose_y, right_x, right_y, 2, COLOR_SHIP);
    draw_line_thick(fb, stride, w, h, left_x, left_y, right_x, right_y, 1, COLOR_SHIP);

    /* Thrust flame */
    if (ship->thrusting) {
        float rf = (float)(frame_count % 7) / 7.0f;
        int flame_len = 8 + (int)(rf * 8);
        int tail_x = cx - (int)(cosf(a) * flame_len);
        int tail_y = cy - (int)(sinf(a) * flame_len);
        int mid_x = cx - (int)(cosf(a) * SHIP_RADIUS * 0.4f);
        int mid_y = cy - (int)(sinf(a) * SHIP_RADIUS * 0.4f);
        draw_line_thick(fb, stride, w, h, mid_x, mid_y, tail_x, tail_y, 2, COLOR_SHIP_THRUST);
        /* Inner white flame */
        int inner_x = cx - (int)(cosf(a) * (flame_len * 0.6f));
        int inner_y = cy - (int)(sinf(a) * (flame_len * 0.6f));
        draw_line(fb, stride, w, h, mid_x, mid_y, inner_x, inner_y, 0xFFFFFF88);
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

    printf("=== DRM Asteroids Game (Overlay + Cursor Planes) ===\n");
    printf("Controls: Left/Right=Rotate, Up=Thrust, Space=Fire, R=Restart, ESC/Q=Quit\n\n");

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game state */
    game_init(&game, dev.width, dev.height);

    printf("[GAME] Starting main loop...\n");

    uint64_t fps_timer = get_time_ns();
    int frame_count = 0;

    /* Main game loop */
    while (game.running && !g_quit) {
        uint64_t t0 = get_time_ns();

        /* Poll input */
        input_process(&game);

        /* Update game state */
        game_update(&game, dev.width, dev.height);

        /* Render primary plane (background starfield + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (asteroids + bullets + particles) into back buffer */
        render_overlay(&dev, &game);

        /* Render cursor plane (player ship) */
        render_cursor(&dev.cursor_buf, &game.ship, game.frame_count);

        /* Compute cursor plane position (ship movement) */
        int cursor_x = (int)game.ship.pos.x - (CURSOR_W / 2);
        int cursor_y = (int)game.ship.pos.y - (CURSOR_H / 2);

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
        uint64_t elapsed = get_time_ns() - t0;
        if (elapsed < FRAME_TIME_NS) sleep_ns(FRAME_TIME_NS - elapsed);

        frame_count++;
        if (get_time_ns() - fps_timer >= 5000000000ULL) {
            printf("[GAME] FPS: %.1f | Score: %d | Wave: %d\n",
                   frame_count / 5.0f, game.score, game.wave);
            frame_count = 0; fps_timer = get_time_ns();
        }
    }

    printf("[GAME] Final Score: %d (Wave %d)\n", game.score, game.wave);

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);

    return 0;
}
