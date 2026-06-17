/*
 * drm-invaders.c - Space Invaders Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background (space/stars) + score/lives/wave HUD
 *   - Overlay plane:  Alien invaders, bullets (player and enemy), explosions/particles
 *   - Cursor plane:   Player ship (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm_invaders drm-invaders.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm_invaders drm-invaders.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm_invaders [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right arrows: move ship
 *   - SPACE: fire
 *   - Touchscreen: move ship, tap to fire
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
#define CURSOR_W            128
#define CURSOR_H            128

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define SHIP_WIDTH          30
#define SHIP_HEIGHT         16
#define SHIP_SPEED          5
#define SHIP_MARGIN         30

#define BULLET_WIDTH        3
#define BULLET_HEIGHT       10
#define BULLET_SPEED        8
#define MAX_BULLETS         10
#define FIRE_COOLDOWN       10  /* frames between shots */

#define ALIEN_ROWS          5
#define ALIEN_COLS          11
#define ALIEN_WIDTH         24
#define ALIEN_HEIGHT        18
#define ALIEN_SPACING_X     36
#define ALIEN_SPACING_Y     30
#define ALIEN_MOVE_SPEED    2
#define ALIEN_DROP_AMOUNT   12
#define ALIEN_INITIAL_DELAY 30  /* frames between alien moves */
#define ALIEN_MIN_DELAY     4

#define ALIEN_BULLET_SPEED  4
#define ALIEN_FIRE_CHANCE   200  /* 1 in N chance per frame per column */
#define MAX_ALIEN_BULLETS   8

#define SHIELD_COUNT        4
#define SHIELD_WIDTH        40
#define SHIELD_HEIGHT       24
#define SHIELD_MARGIN       60  /* above ship */

#define COLOR_BG            0xFF000010
#define COLOR_SHIP          0xFF00FF88
#define COLOR_SHIP_WING     0xFF00CC66
#define COLOR_BULLET        0xFFFFFF00
#define COLOR_ALIEN_BULLET  0xFFFF4444
#define COLOR_SHIELD        0xFF44AA44
#define COLOR_BORDER        0xFF222244
#define COLOR_TEXT          0xFFCCCCCC
#define COLOR_SCORE         0xFF00FFCC
#define COLOR_EXPLOSION     0xFFFFAA00
#define COLOR_TRANSPARENT   0x00000000

/* Alien colors per row */
static const uint32_t alien_colors[ALIEN_ROWS] = {
    0xFFFF2222, /* top row - hardest */
    0xFFFF8800,
    0xFFFFFF00,
    0xFF88FF00,
    0xFF00FF88, /* bottom row - easiest */
};

static const int alien_points[ALIEN_ROWS] = {
    50, 40, 30, 20, 10
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

struct bullet {
    float x, y;
    bool active;
};

struct ship {
    int x, y;
    int width, height;
};

struct alien_grid {
    bool alive[ALIEN_ROWS][ALIEN_COLS];
    int alive_count;
    float offset_x;
    float offset_y;
    int direction;      /* 1 = right, -1 = left */
    int move_timer;
    int move_delay;
    int anim_frame;
};

struct shield_pixel {
    bool intact;
};

struct shield {
    int x, y;
    struct shield_pixel pixels[SHIELD_HEIGHT][SHIELD_WIDTH];
};

struct explosion {
    int x, y;
    int timer;
    bool active;
};

struct game_state {
    struct ship ship;
    struct bullet bullets[MAX_BULLETS];
    struct bullet alien_bullets[MAX_ALIEN_BULLETS];
    struct alien_grid aliens;
    struct shield shields[SHIELD_COUNT];
    struct explosion explosions[16];
    int score;
    int lives;
    int level;
    int fire_cooldown;
    bool running;
    bool paused;
    bool game_over;
    uint32_t screen_w, screen_h;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false;
static bool key_right = false;
static bool key_fire = false;

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

    /* Cursor plane (positioned where player ship is) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (dev->width / 2) - (CURSOR_W / 2),
                        (dev->height - SHIP_MARGIN) - (CURSOR_H / 2),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background starfield + score/lives/wave HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Alien invaders, bullets, explosions\n",
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

static inline void put_pixel(uint32_t *fb, uint32_t stride_px,
                             uint32_t w, uint32_t h,
                             int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h)
        fb[y * stride_px + x] = color;
}

static void draw_fill(uint32_t *fb, uint32_t total, uint32_t color) {
    for (uint32_t i = 0; i < total; i++) fb[i] = color;
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
                         int cx, int cy, int radius, uint32_t color) {
    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            if (dx*dx + dy*dy <= r2) {
                int px = cx+dx, py = cy+dy;
                if (px >= 0 && px < (int)sw && py >= 0 && py < (int)sh)
                    fb[py * stride + px] = color;
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

static void draw_digit(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                        int x, int y, int d, int scale, uint32_t color) {
    if (d < 0 || d > 9) return;
    const uint8_t *g = font_5x7[d];
    for (int r = 0; r < 7; r++)
        for (int c = 0; c < 5; c++)
            if (g[r] & (0x10 >> c))
                draw_rect(fb, stride, sw, sh, x+c*scale, y+r*scale, scale, scale, color);
}

static void draw_number(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int x, int y, int num, int scale, uint32_t color) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int off = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride, sw, sh, x+off, y, buf[i]-'0', scale, color);
        off += 6*scale;
    }
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
}

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static void input_process(struct game_state *game) {
    struct input_event ev;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS) {
                if (ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0)
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    game->ship.x = (ev.value * (int)game->screen_w / max_val)
                                   - game->ship.width / 2;
                }
            }

            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2);
                switch (ev.code) {
                case KEY_LEFT: key_left = pressed; break;
                case KEY_RIGHT: key_right = pressed; break;
                case KEY_SPACE:
                case BTN_TOUCH:
                    key_fire = (ev.value == 1);
                    if (ev.value == 1 && game->game_over) {
                        /* Reset game */
                        game->game_over = false;
                        game->score = 0;
                        game->lives = 3;
                        game->level = 1;
                    }
                    break;
                case KEY_P:
                    if (ev.value == 1 && !game->game_over)
                        game->paused = !game->paused;
                    break;
                case KEY_ESC: case KEY_Q:
                    if (ev.value == 1) game->running = false;
                    break;
                default: break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void init_shields(struct game_state *game) {
    int shield_y = game->ship.y - SHIELD_MARGIN - SHIELD_HEIGHT;
    int total_width = SHIELD_COUNT * SHIELD_WIDTH + (SHIELD_COUNT - 1) * 40;
    int start_x = (int)game->screen_w / 2 - total_width / 2;

    for (int s = 0; s < SHIELD_COUNT; s++) {
        game->shields[s].x = start_x + s * (SHIELD_WIDTH + 40);
        game->shields[s].y = shield_y;
        /* Fill shield pixels (arch shape) */
        for (int r = 0; r < SHIELD_HEIGHT; r++) {
            for (int c = 0; c < SHIELD_WIDTH; c++) {
                /* Create arch shape */
                bool in_arch = true;
                /* Top rounded corners */
                if (r < 6) {
                    int corner_dist = 6 - r;
                    if (c < corner_dist || c >= SHIELD_WIDTH - corner_dist)
                        in_arch = false;
                }
                /* Bottom notch */
                if (r >= SHIELD_HEIGHT - 8 && c >= SHIELD_WIDTH/3 && c < 2*SHIELD_WIDTH/3)
                    in_arch = false;

                game->shields[s].pixels[r][c].intact = in_arch;
            }
        }
    }
}

static void init_aliens(struct game_state *game) {
    game->aliens.alive_count = 0;
    for (int r = 0; r < ALIEN_ROWS; r++) {
        for (int c = 0; c < ALIEN_COLS; c++) {
            game->aliens.alive[r][c] = true;
            game->aliens.alive_count++;
        }
    }
    game->aliens.offset_x = 20;
    game->aliens.offset_y = 40;
    game->aliens.direction = 1;
    game->aliens.move_timer = 0;
    game->aliens.move_delay = ALIEN_INITIAL_DELAY - (game->level - 1) * 3;
    if (game->aliens.move_delay < ALIEN_MIN_DELAY)
        game->aliens.move_delay = ALIEN_MIN_DELAY;
    game->aliens.anim_frame = 0;
}

static void add_explosion(struct game_state *game, int x, int y) {
    for (int i = 0; i < 16; i++) {
        if (!game->explosions[i].active) {
            game->explosions[i].x = x;
            game->explosions[i].y = y;
            game->explosions[i].timer = 12;
            game->explosions[i].active = true;
            return;
        }
    }
}

static void game_init(struct game_state *game, uint32_t sw, uint32_t sh) {
    memset(game, 0, sizeof(*game));
    game->screen_w = sw;
    game->screen_h = sh;
    game->running = true;
    game->lives = 3;
    game->level = 1;

    game->ship.width = SHIP_WIDTH;
    game->ship.height = SHIP_HEIGHT;
    game->ship.x = (int)sw / 2 - SHIP_WIDTH / 2;
    game->ship.y = (int)sh - SHIP_MARGIN;

    srand((unsigned)time(NULL));
    init_aliens(game);
    init_shields(game);
}

static void fire_bullet(struct game_state *game) {
    if (game->fire_cooldown > 0) return;

    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!game->bullets[i].active) {
            game->bullets[i].x = game->ship.x + game->ship.width / 2.0f - BULLET_WIDTH / 2.0f;
            game->bullets[i].y = game->ship.y - BULLET_HEIGHT;
            game->bullets[i].active = true;
            game->fire_cooldown = FIRE_COOLDOWN;
            return;
        }
    }
}

static void alien_fire(struct game_state *game) {
    /* Find bottom-most alien in a random column and fire */
    if (rand() % ALIEN_FIRE_CHANCE != 0) return;

    int col = rand() % ALIEN_COLS;
    int row = -1;
    for (int r = ALIEN_ROWS - 1; r >= 0; r--) {
        if (game->aliens.alive[r][col]) { row = r; break; }
    }
    if (row < 0) return;

    for (int i = 0; i < MAX_ALIEN_BULLETS; i++) {
        if (!game->alien_bullets[i].active) {
            game->alien_bullets[i].x = game->aliens.offset_x + col * ALIEN_SPACING_X +
                                       ALIEN_WIDTH / 2.0f;
            game->alien_bullets[i].y = game->aliens.offset_y + row * ALIEN_SPACING_Y +
                                       ALIEN_HEIGHT;
            game->alien_bullets[i].active = true;
            return;
        }
    }
}

static void damage_shield(struct game_state *game, int sx, int sy, int radius) {
    for (int s = 0; s < SHIELD_COUNT; s++) {
        for (int r = 0; r < SHIELD_HEIGHT; r++) {
            for (int c = 0; c < SHIELD_WIDTH; c++) {
                if (!game->shields[s].pixels[r][c].intact) continue;
                int px = game->shields[s].x + c;
                int py = game->shields[s].y + r;
                int dx = px - sx, dy = py - sy;
                if (dx*dx + dy*dy <= radius*radius)
                    game->shields[s].pixels[r][c].intact = false;
            }
        }
    }
}

static void game_update(struct game_state *game) {
    if (game->paused || game->game_over) return;

    /* Move ship */
    if (key_left) game->ship.x -= SHIP_SPEED;
    if (key_right) game->ship.x += SHIP_SPEED;
    if (game->ship.x < 0) game->ship.x = 0;
    if (game->ship.x > (int)game->screen_w - game->ship.width)
        game->ship.x = (int)game->screen_w - game->ship.width;

    /* Fire */
    if (key_fire) { fire_bullet(game); key_fire = false; }
    if (game->fire_cooldown > 0) game->fire_cooldown--;

    /* Move player bullets */
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!game->bullets[i].active) continue;
        game->bullets[i].y -= BULLET_SPEED;
        if (game->bullets[i].y < 0)
            game->bullets[i].active = false;
    }

    /* Move alien bullets */
    for (int i = 0; i < MAX_ALIEN_BULLETS; i++) {
        if (!game->alien_bullets[i].active) continue;
        game->alien_bullets[i].y += ALIEN_BULLET_SPEED;
        if (game->alien_bullets[i].y > (float)game->screen_h)
            game->alien_bullets[i].active = false;
    }

    /* Move aliens */
    game->aliens.move_timer++;
    if (game->aliens.move_timer >= game->aliens.move_delay) {
        game->aliens.move_timer = 0;
        game->aliens.anim_frame ^= 1;

        /* Check if we need to drop */
        bool need_drop = false;
        for (int r = 0; r < ALIEN_ROWS; r++) {
            for (int c = 0; c < ALIEN_COLS; c++) {
                if (!game->aliens.alive[r][c]) continue;
                float ax = game->aliens.offset_x + c * ALIEN_SPACING_X;
                if (game->aliens.direction > 0 && ax + ALIEN_WIDTH >= (float)game->screen_w - 10)
                    need_drop = true;
                if (game->aliens.direction < 0 && ax <= 10)
                    need_drop = true;
            }
        }

        if (need_drop) {
            game->aliens.offset_y += ALIEN_DROP_AMOUNT;
            game->aliens.direction = -game->aliens.direction;
            /* Speed up slightly */
            if (game->aliens.move_delay > ALIEN_MIN_DELAY)
                game->aliens.move_delay--;
        } else {
            game->aliens.offset_x += game->aliens.direction * ALIEN_MOVE_SPEED;
        }
    }

    /* Alien firing */
    alien_fire(game);

    /* Check bullet-alien collisions */
    for (int b = 0; b < MAX_BULLETS; b++) {
        if (!game->bullets[b].active) continue;
        for (int r = 0; r < ALIEN_ROWS; r++) {
            for (int c = 0; c < ALIEN_COLS; c++) {
                if (!game->aliens.alive[r][c]) continue;
                float ax = game->aliens.offset_x + c * ALIEN_SPACING_X;
                float ay = game->aliens.offset_y + r * ALIEN_SPACING_Y;
                if (game->bullets[b].x + BULLET_WIDTH > ax &&
                    game->bullets[b].x < ax + ALIEN_WIDTH &&
                    game->bullets[b].y < ay + ALIEN_HEIGHT &&
                    game->bullets[b].y + BULLET_HEIGHT > ay) {
                    game->aliens.alive[r][c] = false;
                    game->aliens.alive_count--;
                    game->bullets[b].active = false;
                    game->score += alien_points[r];
                    add_explosion(game, (int)ax + ALIEN_WIDTH/2, (int)ay + ALIEN_HEIGHT/2);

                    /* Speed up aliens as they die */
                    if (game->aliens.alive_count > 0) {
                        int new_delay = ALIEN_INITIAL_DELAY * game->aliens.alive_count /
                                        (ALIEN_ROWS * ALIEN_COLS);
                        if (new_delay < ALIEN_MIN_DELAY) new_delay = ALIEN_MIN_DELAY;
                        game->aliens.move_delay = new_delay;
                    }
                    goto next_bullet;
                }
            }
        }
        next_bullet:;
    }

    /* Check bullet-shield collisions */
    for (int b = 0; b < MAX_BULLETS; b++) {
        if (!game->bullets[b].active) continue;
        for (int s = 0; s < SHIELD_COUNT; s++) {
            int bx = (int)game->bullets[b].x;
            int by = (int)game->bullets[b].y;
            int sx = game->shields[s].x, sy = game->shields[s].y;
            if (bx + BULLET_WIDTH > sx && bx < sx + SHIELD_WIDTH &&
                by + BULLET_HEIGHT > sy && by < sy + SHIELD_HEIGHT) {
                /* Check individual pixels */
                int lx = bx - sx, ly = by - sy;
                for (int pr = ly; pr < ly + BULLET_HEIGHT && pr < SHIELD_HEIGHT; pr++) {
                    for (int pc = lx; pc < lx + BULLET_WIDTH && pc < SHIELD_WIDTH; pc++) {
                        if (pr >= 0 && pc >= 0 && game->shields[s].pixels[pr][pc].intact) {
                            game->shields[s].pixels[pr][pc].intact = false;
                            game->bullets[b].active = false;
                            damage_shield(game, bx, by, 4);
                            goto bullet_done;
                        }
                    }
                }
            }
        }
        bullet_done:;
    }

    /* Check alien bullet-shield and alien bullet-ship collisions */
    for (int b = 0; b < MAX_ALIEN_BULLETS; b++) {
        if (!game->alien_bullets[b].active) continue;

        /* Alien bullet vs shields */
        for (int s = 0; s < SHIELD_COUNT; s++) {
            int bx = (int)game->alien_bullets[b].x;
            int by = (int)game->alien_bullets[b].y;
            int sx = game->shields[s].x, sy = game->shields[s].y;
            if (bx + BULLET_WIDTH > sx && bx < sx + SHIELD_WIDTH &&
                by + BULLET_HEIGHT > sy && by < sy + SHIELD_HEIGHT) {
                int lx = bx - sx, ly = by - sy;
                for (int pr = ly; pr < ly + BULLET_HEIGHT && pr < SHIELD_HEIGHT; pr++) {
                    for (int pc = lx; pc < lx + BULLET_WIDTH && pc < SHIELD_WIDTH; pc++) {
                        if (pr >= 0 && pc >= 0 && game->shields[s].pixels[pr][pc].intact) {
                            game->shields[s].pixels[pr][pc].intact = false;
                            game->alien_bullets[b].active = false;
                            damage_shield(game, bx, by + BULLET_HEIGHT, 4);
                            goto alien_bullet_done;
                        }
                    }
                }
            }
        }

        /* Alien bullet vs ship */
        {
            int bx = (int)game->alien_bullets[b].x;
            int by = (int)game->alien_bullets[b].y;
            if (bx + BULLET_WIDTH > game->ship.x &&
                bx < game->ship.x + game->ship.width &&
                by + BULLET_HEIGHT > game->ship.y &&
                by < game->ship.y + game->ship.height) {
                game->alien_bullets[b].active = false;
                game->lives--;
                add_explosion(game, game->ship.x + game->ship.width/2, game->ship.y);
                if (game->lives <= 0) {
                    game->game_over = true;
                    printf("[GAME] Game Over! Score: %d\n", game->score);
                }
            }
        }
        alien_bullet_done:;
    }

    /* Check if aliens reached ship level */
    for (int r = ALIEN_ROWS - 1; r >= 0; r--) {
        for (int c = 0; c < ALIEN_COLS; c++) {
            if (!game->aliens.alive[r][c]) continue;
            float ay = game->aliens.offset_y + r * ALIEN_SPACING_Y + ALIEN_HEIGHT;
            if (ay >= game->ship.y) {
                game->game_over = true;
                printf("[GAME] Aliens reached you! Game Over! Score: %d\n", game->score);
                return;
            }
        }
    }

    /* Check level complete */
    if (game->aliens.alive_count <= 0) {
        game->level++;
        printf("[GAME] Level %d cleared! Score: %d\n", game->level - 1, game->score);
        init_aliens(game);
        init_shields(game);
        /* Clear bullets */
        for (int i = 0; i < MAX_BULLETS; i++) game->bullets[i].active = false;
        for (int i = 0; i < MAX_ALIEN_BULLETS; i++) game->alien_bullets[i].active = false;
    }

    /* Update explosions */
    for (int i = 0; i < 16; i++) {
        if (game->explosions[i].active) {
            game->explosions[i].timer--;
            if (game->explosions[i].timer <= 0)
                game->explosions[i].active = false;
        }
    }
}

/* ============================================================
 * Rendering - Primary Plane (Background + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t sw = dev->width;
    uint32_t sh = dev->height;

    /* Clear background */
    draw_fill(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Draw stars (static background) */
    srand(42); /* fixed seed for consistent stars */
    for (int i = 0; i < 50; i++) {
        int sx = rand() % (int)sw;
        int sy = rand() % (int)sh;
        if (sy >= 0 && sy < (int)sh && sx >= 0 && sx < (int)sw)
            fb[sy * stride + sx] = 0xFF444466;
    }
    srand((unsigned)time(NULL)); /* restore random */

    /* Draw shields (on primary - they are part of the background/terrain) */
    for (int s = 0; s < SHIELD_COUNT; s++) {
        for (int r = 0; r < SHIELD_HEIGHT; r++) {
            for (int c = 0; c < SHIELD_WIDTH; c++) {
                if (!game->shields[s].pixels[r][c].intact) continue;
                int px = game->shields[s].x + c;
                int py = game->shields[s].y + r;
                if (px >= 0 && px < (int)sw && py >= 0 && py < (int)sh)
                    fb[py * stride + px] = COLOR_SHIELD;
            }
        }
    }

    /* HUD - Score */
    draw_number(fb, stride, sw, sh, 10, 10, game->score, 3, COLOR_SCORE);

    /* HUD - Lives */
    for (int i = 0; i < game->lives; i++) {
        int lx = (int)sw - 30 - i * 22;
        draw_rect(fb, stride, sw, sh, lx, 10, 16, 10, COLOR_SHIP);
    }

    /* HUD - Level/Wave */
    draw_number(fb, stride, sw, sh, (int)sw / 2 - 10, 10, game->level, 2, COLOR_TEXT);

    /* Game over */
    if (game->game_over) {
        draw_rect(fb, stride, sw, sh, (int)sw/4, (int)sh/2 - 25, (int)sw/2, 50, 0xCC000000);
        draw_rect(fb, stride, sw, sh, (int)sw/2 - 20, (int)sh/2 - 3, 40, 6, 0xFFFF4444);
        draw_rect(fb, stride, sw, sh, (int)sw/2 - 3, (int)sh/2 - 20, 6, 40, 0xFFFF4444);
        draw_number(fb, stride, sw, sh, (int)sw/2 - 30, (int)sh/2 + 25, game->score, 2, COLOR_TEXT);
    }

    /* Paused */
    if (game->paused && !game->game_over) {
        draw_rect(fb, stride, sw, sh, (int)sw/2 - 30, (int)sh/2 - 10, 60, 20, 0xCC000000);
        draw_rect(fb, stride, sw, sh, (int)sw/2 - 12, (int)sh/2 - 6, 8, 12, COLOR_TEXT);
        draw_rect(fb, stride, sw, sh, (int)sw/2 + 4, (int)sh/2 - 6, 8, 12, COLOR_TEXT);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Aliens + Bullets + Explosions)
 * ============================================================ */

static void draw_alien(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                        int x, int y, int row, int anim) {
    uint32_t color = alien_colors[row];

    /* Body */
    draw_rect(fb, stride, sw, sh, x + 4, y + 4, ALIEN_WIDTH - 8, ALIEN_HEIGHT - 8, color);
    /* Head */
    draw_rect(fb, stride, sw, sh, x + 6, y + 2, ALIEN_WIDTH - 12, 6, color);
    /* Eyes */
    draw_rect(fb, stride, sw, sh, x + 7, y + 5, 3, 3, 0xFF000000);
    draw_rect(fb, stride, sw, sh, x + ALIEN_WIDTH - 10, y + 5, 3, 3, 0xFF000000);

    /* Tentacles (animated) */
    if (anim == 0) {
        draw_rect(fb, stride, sw, sh, x + 2, y + ALIEN_HEIGHT - 6, 4, 4, color);
        draw_rect(fb, stride, sw, sh, x + ALIEN_WIDTH - 6, y + ALIEN_HEIGHT - 6, 4, 4, color);
    } else {
        draw_rect(fb, stride, sw, sh, x, y + ALIEN_HEIGHT - 4, 4, 4, color);
        draw_rect(fb, stride, sw, sh, x + ALIEN_WIDTH - 4, y + ALIEN_HEIGHT - 4, 4, 4, color);
    }
}

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t sw = OVERLAY_W;
    uint32_t sh = OVERLAY_H;

    /* Clear overlay to transparent */
    draw_fill(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw aliens */
    for (int r = 0; r < ALIEN_ROWS; r++) {
        for (int c = 0; c < ALIEN_COLS; c++) {
            if (!game->aliens.alive[r][c]) continue;
            int ax = (int)game->aliens.offset_x + c * ALIEN_SPACING_X;
            int ay = (int)game->aliens.offset_y + r * ALIEN_SPACING_Y;
            draw_alien(fb, stride, sw, sh, ax, ay, r, game->aliens.anim_frame);
        }
    }

    /* Draw player bullets */
    for (int i = 0; i < MAX_BULLETS; i++) {
        if (!game->bullets[i].active) continue;
        draw_rect(fb, stride, sw, sh,
                  (int)game->bullets[i].x, (int)game->bullets[i].y,
                  BULLET_WIDTH, BULLET_HEIGHT, COLOR_BULLET);
    }

    /* Draw alien bullets */
    for (int i = 0; i < MAX_ALIEN_BULLETS; i++) {
        if (!game->alien_bullets[i].active) continue;
        draw_rect(fb, stride, sw, sh,
                  (int)game->alien_bullets[i].x, (int)game->alien_bullets[i].y,
                  BULLET_WIDTH, BULLET_HEIGHT, COLOR_ALIEN_BULLET);
    }

    /* Draw explosions */
    for (int i = 0; i < 16; i++) {
        if (!game->explosions[i].active) continue;
        int radius = 12 - game->explosions[i].timer;
        uint32_t c = (game->explosions[i].timer > 6) ? COLOR_EXPLOSION : 0xFFFF6600;
        draw_circle(fb, stride, sw, sh,
                    game->explosions[i].x, game->explosions[i].y, radius, c);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Player Ship)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t sw = buf->width;
    uint32_t sh = buf->height;

    /* Clear to transparent */
    draw_fill(fb, buf->size / 4, COLOR_TRANSPARENT);

    /* Draw player ship centered in cursor buffer */
    int cx = ((int)sw - SHIP_WIDTH) / 2;
    int cy = ((int)sh - SHIP_HEIGHT) / 2;

    /* Ship body */
    draw_rect(fb, stride, sw, sh,
              cx + 5, cy + 4,
              SHIP_WIDTH - 10, SHIP_HEIGHT - 4, COLOR_SHIP);
    /* Ship nose (triangle-ish) */
    draw_rect(fb, stride, sw, sh,
              cx + SHIP_WIDTH/2 - 2, cy,
              4, 6, COLOR_SHIP);
    /* Wings */
    draw_rect(fb, stride, sw, sh,
              cx, cy + 8,
              6, SHIP_HEIGHT - 8, COLOR_SHIP_WING);
    draw_rect(fb, stride, sw, sh,
              cx + SHIP_WIDTH - 6, cy + 8,
              6, SHIP_HEIGHT - 8, COLOR_SHIP_WING);
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
    printf("  DRM Space Invaders - Microchip Demo\n");
    printf("  Built with libdrm (KMS/DRM, 3 Planes)\n");
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

    /* Draw the player ship sprite once (cursor plane content is static, just moves) */
    render_cursor(&dev.cursor_buf);

    printf("[GAME] Controls:\n");
    printf("  Left/Right / Touch : Move ship\n");
    printf("  SPACE / Tap        : Fire\n");
    printf("  P                  : Pause\n");
    printf("  ESC / Q            : Quit\n\n");

    /* Main game loop */
    uint64_t frame_start, frame_end, frame_elapsed;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        /* Poll input */
        input_process(&game);

        /* Update game state */
        game_update(&game);

        /* Render primary plane (background + shields + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (aliens + bullets + explosions) into back buffer */
        render_overlay(&dev, &game);

        /* Compute cursor plane position (player ship movement) */
        int cursor_x = game.ship.x - (CURSOR_W - SHIP_WIDTH) / 2;
        int cursor_y = game.ship.y - (CURSOR_H - SHIP_HEIGHT) / 2;

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

    printf("[GAME] Final Score: %d | Level: %d\n", game.score, game.level);
    input_cleanup();
    drm_cleanup(&dev);
    return 0;
}
