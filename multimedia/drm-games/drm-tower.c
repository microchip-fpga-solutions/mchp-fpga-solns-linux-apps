/*
 * drm-tower.c - Tower Defense using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Grid map with path, placed towers, score/wave HUD
 *   - Overlay plane:  Enemies moving along path, projectiles, hit effects
 *   - Cursor plane:   Selection cursor / tower placement indicator
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-tower drm-tower.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-tower drm-tower.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-tower [/dev/dri/cardN]
 *
 * Controls:
 *   - Arrow keys: move cursor on grid
 *   - Space: place tower at cursor position
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

#define TARGET_FPS          30
#define FRAME_TIME_NS       (1000000000 / TARGET_FPS)

/* Number of framebuffers for primary and overlay planes (triple-buffering).
 * Triple-buffering allows the GPU/CPU to render ahead by one frame without
 * stalling on vsync, reducing dropped frames when rendering is uneven. */
#define NUM_BUFFERS         3

#define SCREEN_W            1280
#define SCREEN_H            720

#define OVERLAY_W           1280
#define OVERLAY_H           720

#define CURSOR_W            64
#define CURSOR_H            64

/* Grid settings */
#define GRID_COLS           16
#define GRID_ROWS           9
#define CELL_W              (SCREEN_W / GRID_COLS)   /* 80 */
#define CELL_H              ((SCREEN_H - 40) / GRID_ROWS) /* ~75 */
#define GRID_OFFSET_Y       40

/* Game tuning */
#define MAX_ENEMIES         20
#define MAX_TOWERS          30
#define MAX_PROJECTILES     40
#define MAX_EFFECTS         15
#define TOWER_RANGE         200.0f
#define TOWER_FIRE_RATE_MS  800
#define PROJECTILE_SPEED    8.0f
#define ENEMY_BASE_SPEED    1.5f
#define TOWER_COST          50
#define ENEMY_REWARD        20
#define STARTING_GOLD       200
#define STARTING_LIVES      10
#define WAVE_INTERVAL_MS    5000

/* Path waypoints (grid coordinates) */
#define PATH_LEN            12

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_GRASS         0xFF1A4D1A
#define COLOR_GRASS2        0xFF1E5520
#define COLOR_PATH          0xFF8B7355
#define COLOR_PATH_EDGE     0xFF6B5335
#define COLOR_GRID_LINE     0xFF143314
#define COLOR_BORDER        0xFF2A6B2A
#define COLOR_HUD_BG        0xFF1A1A2E
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE_VAL     0xFF00FF80
#define COLOR_GOLD_VAL      0xFFFFDD00
#define COLOR_LIVES_VAL     0xFFFF4444
#define COLOR_WAVE_VAL      0xFF88BBFF

#define COLOR_TOWER_BASE    0xFF555588
#define COLOR_TOWER_TOP     0xFF7777BB
#define COLOR_TOWER_GUN     0xFF333355

#define COLOR_ENEMY_BODY    0xFFCC3333
#define COLOR_ENEMY_FAST    0xFFFFAA00
#define COLOR_ENEMY_TANK    0xFF6633CC
#define COLOR_ENEMY_HP_BG   0xFF333333
#define COLOR_ENEMY_HP_FG   0xFF00CC00

#define COLOR_PROJECTILE    0xFFFFFF44
#define COLOR_EFFECT_HIT    0xFFFF8800

#define COLOR_CURSOR_OK     0xFF00FF00
#define COLOR_CURSOR_BAD    0xFFFF0000

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

struct vec2 { float x, y; };

/* Path definition */
static const struct vec2 path_waypoints[PATH_LEN] = {
    {0, 2}, {2, 2}, {2, 5}, {5, 5}, {5, 1}, {8, 1},
    {8, 7}, {11, 7}, {11, 3}, {13, 3}, {13, 6}, {16, 6}
};

enum cell_type { CELL_GRASS = 0, CELL_PATH, CELL_TOWER };

struct tower {
    int grid_x, grid_y;
    float world_x, world_y;
    uint64_t last_fire_time;
    bool active;
};

enum enemy_type { ENEMY_NORMAL = 0, ENEMY_FAST, ENEMY_TANK };

struct enemy {
    float x, y;
    float speed;
    int hp, max_hp;
    int path_idx;   /* index along path segment */
    float path_t;   /* interpolation along current segment */
    enum enemy_type type;
    bool active;
};

struct projectile {
    float x, y;
    float dx, dy;
    int damage;
    bool active;
};

struct effect {
    float x, y;
    int life;
    bool active;
};

struct game_state {
    int cursor_gx, cursor_gy;
    enum cell_type grid[GRID_ROWS][GRID_COLS];
    struct tower towers[MAX_TOWERS];
    struct enemy enemies[MAX_ENEMIES];
    struct projectile projectiles[MAX_PROJECTILES];
    struct effect effects[MAX_EFFECTS];
    int tower_count;
    int gold;
    int lives;
    int score;
    int wave;
    int enemies_spawned;
    int enemies_per_wave;
    uint64_t last_spawn_time;
    uint64_t wave_start_time;
    int spawn_interval_ms;
    bool wave_active;
    bool running;
    bool game_over;
    bool cursor_valid;  /* can place tower here? */
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false, key_right = false, key_up = false, key_down = false;
static bool key_space = false, key_space_prev = false;

static void signal_handler(int sig) { (void)sig; g_quit = true; }

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

    drmModeEncoder *enc = conn->encoder_id ? drmModeGetEncoder(dev->fd, conn->encoder_id) : NULL;
    if (enc) { dev->crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    else {
        for (int i = 0; i < conn->count_encoders; i++) {
            enc = drmModeGetEncoder(dev->fd, conn->encoders[i]);
            if (enc) {
                for (int j = 0; j < res->count_crtcs; j++)
                    if (enc->possible_crtcs & (1 << j)) { dev->crtc_id = res->crtcs[j]; break; }
                drmModeFreeEncoder(enc);
                if (dev->crtc_id) break;
            }
        }
    }
    if (!dev->crtc_id) { drmModeFreeConnector(conn); drmModeFreeResources(res); return -1; }
    for (int i = 0; i < res->count_crtcs; i++)
        if (res->crtcs[i] == dev->crtc_id) { dev->crtc_idx = i; break; }
    dev->saved_crtc = drmModeGetCrtc(dev->fd, dev->crtc_id);
    drmModeFreeConnector(conn); drmModeFreeResources(res);
    return 0;
}

static int drm_find_planes(struct drm_device *dev) {
    drmModePlaneRes *pr = drmModeGetPlaneResources(dev->fd);
    if (!pr) return -1;
    for (uint32_t i = 0; i < pr->count_planes; i++) {
        drmModePlane *plane = drmModeGetPlane(dev->fd, pr->planes[i]);
        if (!plane) continue;
        if (!(plane->possible_crtcs & (1 << dev->crtc_idx))) { drmModeFreePlane(plane); continue; }
        drmModeObjectProperties *props = drmModeObjectGetProperties(dev->fd, pr->planes[i], DRM_MODE_OBJECT_PLANE);
        if (!props) { drmModeFreePlane(plane); continue; }
        int pt = -1;
        for (uint32_t j = 0; j < props->count_props; j++) {
            drmModePropertyRes *prop = drmModeGetProperty(dev->fd, props->props[j]);
            if (!prop) continue;
            if (strcmp(prop->name, "type") == 0) { pt = (int)props->prop_values[j]; drmModeFreeProperty(prop); break; }
            drmModeFreeProperty(prop);
        }
        drmModeFreeObjectProperties(props);
        switch (pt) {
        case 1: if (!dev->primary_plane_id) dev->primary_plane_id = pr->planes[i]; break;
        case 0: if (!dev->overlay_plane_id) dev->overlay_plane_id = pr->planes[i]; break;
        case 2: if (!dev->cursor_plane_id) dev->cursor_plane_id = pr->planes[i]; break;
        }
        drmModeFreePlane(plane);
    }
    drmModeFreePlaneResources(pr);
    if (!dev->primary_plane_id || !dev->overlay_plane_id || !dev->cursor_plane_id) return -1;
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
        struct drm_mode_destroy_dumb d = { .handle = buf->handle };
        drmIoctl(dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d); buf->handle = 0;
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

    /* Create cursor plane buffer (selection cursor, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at center of grid) */
    int init_cx = (int)(GRID_COLS / 2) * CELL_W + CELL_W / 2 - CURSOR_W / 2;
    int init_cy = GRID_OFFSET_Y + (int)(GRID_ROWS / 2) * CELL_H + CELL_H / 2 - CURSOR_H / 2;
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        init_cx, init_cy,
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Grid map with path, placed towers, score/wave/gold HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Enemies moving along path, projectiles, hit effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Tower placement selection cursor\n",
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
    printf("[DRM] Cleaned up\n");
}

/* ============================================================
 * Drawing Primitives
 * ============================================================ */

static inline void put_pixel(uint32_t *fb, uint32_t stride_px, uint32_t w, uint32_t h,
                             int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h) fb[y * stride_px + x] = color;
}

static void draw_rect(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                      int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++) fb[j * stride_px + i] = color;
}

static void draw_circle(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                        int cx, int cy, int r, uint32_t color) {
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r*r) put_pixel(fb, stride_px, scr_w, scr_h, cx+dx, cy+dy, color);
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++) fb[i] = color;
}

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

static void draw_char(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                      int px, int py, char ch, int scale, uint32_t color) {
    const uint8_t *g = NULL;
    if (ch >= '0' && ch <= '9') g = font_5x7[ch - '0'];
    else if (ch >= 'A' && ch <= 'Z') g = font_alpha[ch - 'A'];
    else if (ch >= 'a' && ch <= 'z') g = font_alpha[ch - 'a'];
    else return;
    for (int r = 0; r < 7; r++)
        for (int c = 0; c < 5; c++)
            if (g[r] & (0x10 >> c))
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        put_pixel(fb, stride_px, sw, sh, px+c*scale+sx, py+r*scale+sy, color);
}

static void draw_string(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                        int px, int py, const char *str, int scale, uint32_t color) {
    int x = px;
    for (int i = 0; str[i]; i++) {
        if (str[i] == ' ') { x += 4*scale; continue; }
        if (str[i] == ':') {
            put_pixel(fb, stride_px, sw, sh, x+scale, py+2*scale, color);
            put_pixel(fb, stride_px, sw, sh, x+scale, py+4*scale, color);
            x += 3*scale; continue;
        }
        draw_char(fb, stride_px, sw, sh, x, py, str[i], scale, color);
        x += 6*scale;
    }
}

static void draw_number(uint32_t *fb, uint32_t stride_px, uint32_t sw, uint32_t sh,
                        int px, int py, int num, int scale, uint32_t color) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int x = px;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_char(fb, stride_px, sw, sh, x, py, buf[i], scale, color);
        x += 6*scale;
    }
}

/* ============================================================
 * Path Helpers
 * ============================================================ */

/* Convert grid coord to pixel center */
static float grid_to_px(int gx) { return gx * CELL_W + CELL_W / 2; }
static float grid_to_py(int gy) { return GRID_OFFSET_Y + gy * CELL_H + CELL_H / 2; }

/* Check if a grid cell is on the path */
static bool is_path_cell(int gx, int gy) {
    /* Rasterize path segments onto grid */
    for (int seg = 0; seg < PATH_LEN - 1; seg++) {
        int x0 = (int)path_waypoints[seg].x;
        int y0 = (int)path_waypoints[seg].y;
        int x1 = (int)path_waypoints[seg + 1].x;
        int y1 = (int)path_waypoints[seg + 1].y;

        if (x0 == x1) { /* vertical */
            if (gx != x0) continue;
            int miny = y0 < y1 ? y0 : y1;
            int maxy = y0 > y1 ? y0 : y1;
            if (gy >= miny && gy <= maxy) return true;
        } else { /* horizontal */
            if (gy != y0) continue;
            int minx = x0 < x1 ? x0 : x1;
            int maxx = x0 > x1 ? x0 : x1;
            if (gx >= minx && gx <= maxx) return true;
        }
    }
    return false;
}

/* ============================================================
 * Drawing Game Elements
 * ============================================================ */

/* Draw cursor - updates each frame based on validity */
static void draw_cursor(struct drm_buffer *buf, bool valid) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width, h = buf->height;

    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    uint32_t color = valid ? COLOR_CURSOR_OK : COLOR_CURSOR_BAD;
    int cx = w / 2, cy = h / 2;
    int size = 26;

    /* Draw selection box outline */
    for (int i = 0; i < size; i++) {
        /* Top edge */
        put_pixel(fb, stride_px, w, h, cx - size + i, cy - size, color);
        put_pixel(fb, stride_px, w, h, cx + i, cy - size, color);
        /* Bottom edge */
        put_pixel(fb, stride_px, w, h, cx - size + i, cy + size, color);
        put_pixel(fb, stride_px, w, h, cx + i, cy + size, color);
        /* Left edge */
        put_pixel(fb, stride_px, w, h, cx - size, cy - size + i, color);
        put_pixel(fb, stride_px, w, h, cx - size, cy + i, color);
        /* Right edge */
        put_pixel(fb, stride_px, w, h, cx + size, cy - size + i, color);
        put_pixel(fb, stride_px, w, h, cx + size, cy + i, color);
    }

    /* Corner accents */
    for (int i = 0; i < 8; i++) {
        draw_rect(fb, stride_px, w, h, cx - size - 1, cy - size - 1, i + 2, 3, color);
        draw_rect(fb, stride_px, w, h, cx + size - i - 1, cy - size - 1, i + 2, 3, color);
        draw_rect(fb, stride_px, w, h, cx - size - 1, cy + size - 1, i + 2, 3, color);
        draw_rect(fb, stride_px, w, h, cx + size - i - 1, cy + size - 1, i + 2, 3, color);
    }

    /* Crosshair center */
    put_pixel(fb, stride_px, w, h, cx, cy, color);
    put_pixel(fb, stride_px, w, h, cx - 1, cy, color);
    put_pixel(fb, stride_px, w, h, cx + 1, cy, color);
    put_pixel(fb, stride_px, w, h, cx, cy - 1, color);
    put_pixel(fb, stride_px, w, h, cx, cy + 1, color);
}

/* Draw background (primary) - map + towers + HUD */
static void draw_background(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width, h = dev->height;

    /* HUD bar at top */
    draw_rect(fb, stride_px, w, h, 0, 0, w, GRID_OFFSET_Y, COLOR_HUD_BG);

    /* Draw grid */
    for (int gy = 0; gy < GRID_ROWS; gy++) {
        for (int gx = 0; gx < GRID_COLS; gx++) {
            int px = gx * CELL_W;
            int py = GRID_OFFSET_Y + gy * CELL_H;
            uint32_t color;

            if (game->grid[gy][gx] == CELL_PATH)
                color = COLOR_PATH;
            else
                color = ((gx + gy) % 2) ? COLOR_GRASS : COLOR_GRASS2;

            draw_rect(fb, stride_px, w, h, px, py, CELL_W, CELL_H, color);

            /* Grid lines */
            draw_rect(fb, stride_px, w, h, px, py, CELL_W, 1, COLOR_GRID_LINE);
            draw_rect(fb, stride_px, w, h, px, py, 1, CELL_H, COLOR_GRID_LINE);
        }
    }

    /* Path edges */
    for (int gy = 0; gy < GRID_ROWS; gy++) {
        for (int gx = 0; gx < GRID_COLS; gx++) {
            if (game->grid[gy][gx] != CELL_PATH) continue;
            int px = gx * CELL_W;
            int py = GRID_OFFSET_Y + gy * CELL_H;
            draw_rect(fb, stride_px, w, h, px, py, CELL_W, 2, COLOR_PATH_EDGE);
            draw_rect(fb, stride_px, w, h, px, py + CELL_H - 2, CELL_W, 2, COLOR_PATH_EDGE);
        }
    }

    /* Draw towers */
    for (int i = 0; i < MAX_TOWERS; i++) {
        if (!game->towers[i].active) continue;
        int tx = (int)game->towers[i].world_x;
        int ty = (int)game->towers[i].world_y;

        /* Tower base */
        draw_rect(fb, stride_px, w, h, tx - 20, ty - 15, 40, 30, COLOR_TOWER_BASE);
        /* Tower top */
        draw_rect(fb, stride_px, w, h, tx - 12, ty - 25, 24, 15, COLOR_TOWER_TOP);
        /* Gun barrel */
        draw_rect(fb, stride_px, w, h, tx - 2, ty - 30, 4, 12, COLOR_TOWER_GUN);
        /* Range indicator (subtle) */
        /* Just corners to hint at range */
    }

    /* HUD text */
    draw_string(fb, stride_px, w, h, 10, 12, "GOLD:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 75, 12, game->gold, 2, COLOR_GOLD_VAL);

    draw_string(fb, stride_px, w, h, 200, 12, "LIVES:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 280, 12, game->lives, 2, COLOR_LIVES_VAL);

    draw_string(fb, stride_px, w, h, 420, 12, "WAVE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 490, 12, game->wave, 2, COLOR_WAVE_VAL);

    draw_string(fb, stride_px, w, h, 620, 12, "SCORE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 700, 12, game->score, 2, COLOR_SCORE_VAL);

    draw_string(fb, stride_px, w, h, w - 250, 12, "COST:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w - 180, 12, TOWER_COST, 2, COLOR_GOLD_VAL);

    if (game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 - 20, "GAME OVER", 4, 0xFFFF0000);
        draw_string(fb, stride_px, w, h, w / 2 - 80, h / 2 + 30, "PRESS Q", 3, COLOR_HUD_TEXT);
    }
}

/* Draw overlay (enemies + projectiles + effects) */
static void draw_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W, h = OVERLAY_H;

    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw enemies */
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) continue;
        int ex = (int)game->enemies[i].x;
        int ey = (int)game->enemies[i].y;
        int size = 12;

        uint32_t color;
        switch (game->enemies[i].type) {
        case ENEMY_FAST: color = COLOR_ENEMY_FAST; size = 10; break;
        case ENEMY_TANK: color = COLOR_ENEMY_TANK; size = 16; break;
        default:         color = COLOR_ENEMY_BODY; break;
        }

        draw_circle(fb, stride_px, w, h, ex, ey, size, color);

        /* HP bar */
        int bar_w = 24;
        int bar_h = 4;
        int bar_x = ex - bar_w / 2;
        int bar_y = ey - size - 6;
        float hp_pct = (float)game->enemies[i].hp / game->enemies[i].max_hp;
        draw_rect(fb, stride_px, w, h, bar_x, bar_y, bar_w, bar_h, COLOR_ENEMY_HP_BG);
        draw_rect(fb, stride_px, w, h, bar_x, bar_y, (int)(bar_w * hp_pct), bar_h, COLOR_ENEMY_HP_FG);
    }

    /* Draw projectiles */
    for (int i = 0; i < MAX_PROJECTILES; i++) {
        if (!game->projectiles[i].active) continue;
        int px = (int)game->projectiles[i].x;
        int py = (int)game->projectiles[i].y;
        draw_circle(fb, stride_px, w, h, px, py, 3, COLOR_PROJECTILE);
    }

    /* Draw effects */
    for (int i = 0; i < MAX_EFFECTS; i++) {
        if (!game->effects[i].active) continue;
        int ex = (int)game->effects[i].x;
        int ey = (int)game->effects[i].y;
        int r = 4 + (10 - game->effects[i].life);
        draw_circle(fb, stride_px, w, h, ex, ey, r, COLOR_EFFECT_HIT);
    }
}

/* ============================================================
 * Input
 * ============================================================ */

static void input_init(void) {
    char path[64];
    for (int i = 0; i < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) input_fds[input_count++] = fd;
    }
}
static void input_cleanup(void) { for (int i = 0; i < input_count; i++) close(input_fds[i]); input_count = 0; }

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
                case KEY_UP:    key_up = pressed; break;
                case KEY_DOWN:  key_down = pressed; break;
                case KEY_SPACE: key_space = pressed; break;
                case KEY_ESC: case KEY_Q: if (ev.value == 1) g_quit = true; break;
                }
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void init_grid(struct game_state *game) {
    for (int gy = 0; gy < GRID_ROWS; gy++)
        for (int gx = 0; gx < GRID_COLS; gx++)
            game->grid[gy][gx] = is_path_cell(gx, gy) ? CELL_PATH : CELL_GRASS;
}

static void game_init(struct game_state *game) {
    memset(game, 0, sizeof(*game));
    game->cursor_gx = GRID_COLS / 2;
    game->cursor_gy = GRID_ROWS / 2;
    game->gold = STARTING_GOLD;
    game->lives = STARTING_LIVES;
    game->wave = 1;
    game->enemies_per_wave = 5;
    game->spawn_interval_ms = 1500;
    game->running = true;
    game->wave_active = true;
    game->wave_start_time = get_time_ms();
    game->last_spawn_time = get_time_ms();
    srand(time(NULL));
    init_grid(game);
}

static bool place_tower(struct game_state *game) {
    int gx = game->cursor_gx, gy = game->cursor_gy;
    if (game->grid[gy][gx] != CELL_GRASS) return false;
    if (game->gold < TOWER_COST) return false;

    for (int i = 0; i < MAX_TOWERS; i++) {
        if (!game->towers[i].active) {
            game->towers[i].active = true;
            game->towers[i].grid_x = gx;
            game->towers[i].grid_y = gy;
            game->towers[i].world_x = grid_to_px(gx);
            game->towers[i].world_y = grid_to_py(gy);
            game->towers[i].last_fire_time = 0;
            game->tower_count++;
            game->grid[gy][gx] = CELL_TOWER;
            game->gold -= TOWER_COST;
            return true;
        }
    }
    return false;
}

static void spawn_enemy(struct game_state *game) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) {
            game->enemies[i].active = true;
            game->enemies[i].path_idx = 0;
            game->enemies[i].path_t = 0;
            game->enemies[i].x = grid_to_px((int)path_waypoints[0].x);
            game->enemies[i].y = grid_to_py((int)path_waypoints[0].y);

            /* Random type based on wave */
            int r = rand() % 100;
            if (game->wave >= 3 && r < 20) {
                game->enemies[i].type = ENEMY_TANK;
                game->enemies[i].hp = 4 + game->wave;
                game->enemies[i].speed = ENEMY_BASE_SPEED * 0.6f;
            } else if (game->wave >= 2 && r < 40) {
                game->enemies[i].type = ENEMY_FAST;
                game->enemies[i].hp = 1;
                game->enemies[i].speed = ENEMY_BASE_SPEED * 1.8f;
            } else {
                game->enemies[i].type = ENEMY_NORMAL;
                game->enemies[i].hp = 2 + game->wave / 2;
                game->enemies[i].speed = ENEMY_BASE_SPEED + game->wave * 0.1f;
            }
            game->enemies[i].max_hp = game->enemies[i].hp;
            return;
        }
    }
}

static void fire_at_enemy(struct game_state *game, struct tower *tower, struct enemy *enemy) {
    for (int i = 0; i < MAX_PROJECTILES; i++) {
        if (!game->projectiles[i].active) {
            game->projectiles[i].active = true;
            game->projectiles[i].x = tower->world_x;
            game->projectiles[i].y = tower->world_y - 30;
            float dx = enemy->x - game->projectiles[i].x;
            float dy = enemy->y - game->projectiles[i].y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < 1) dist = 1;
            game->projectiles[i].dx = (dx / dist) * PROJECTILE_SPEED;
            game->projectiles[i].dy = (dy / dist) * PROJECTILE_SPEED;
            game->projectiles[i].damage = 1;
            tower->last_fire_time = get_time_ms();
            return;
        }
    }
}

static void spawn_effect(struct game_state *game, float x, float y) {
    for (int i = 0; i < MAX_EFFECTS; i++) {
        if (!game->effects[i].active) {
            game->effects[i].active = true;
            game->effects[i].x = x;
            game->effects[i].y = y;
            game->effects[i].life = 10;
            return;
        }
    }
}

static void game_update(struct game_state *game) {
    if (game->game_over) return;

    /* Move cursor (with key repeat delay) */
    static uint64_t last_move = 0;
    uint64_t now = get_time_ms();
    if (now - last_move > 120) {
        if (key_left && game->cursor_gx > 0) { game->cursor_gx--; last_move = now; }
        if (key_right && game->cursor_gx < GRID_COLS - 1) { game->cursor_gx++; last_move = now; }
        if (key_up && game->cursor_gy > 0) { game->cursor_gy--; last_move = now; }
        if (key_down && game->cursor_gy < GRID_ROWS - 1) { game->cursor_gy++; last_move = now; }
    }

    /* Check cursor validity */
    game->cursor_valid = (game->grid[game->cursor_gy][game->cursor_gx] == CELL_GRASS &&
                          game->gold >= TOWER_COST);

    /* Place tower */
    if (key_space && !key_space_prev) place_tower(game);

    /* Spawn enemies in wave */
    if (game->wave_active) {
        if (now - game->last_spawn_time > (uint64_t)game->spawn_interval_ms &&
            game->enemies_spawned < game->enemies_per_wave) {
            spawn_enemy(game);
            game->enemies_spawned++;
            game->last_spawn_time = now;
        }
    }

    /* Update enemies along path */
    int active_enemies = 0;
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) continue;
        active_enemies++;

        int seg = game->enemies[i].path_idx;
        if (seg >= PATH_LEN - 1) {
            /* Enemy reached the end */
            game->enemies[i].active = false;
            game->lives--;
            if (game->lives <= 0) { game->lives = 0; game->game_over = true; }
            continue;
        }

        /* Move along path */
        float sx = grid_to_px((int)path_waypoints[seg].x);
        float sy = grid_to_py((int)path_waypoints[seg].y);
        float ex = grid_to_px((int)path_waypoints[seg + 1].x);
        float ey = grid_to_py((int)path_waypoints[seg + 1].y);
        float seg_len = sqrtf((ex - sx) * (ex - sx) + (ey - sy) * (ey - sy));

        game->enemies[i].path_t += game->enemies[i].speed / seg_len;
        if (game->enemies[i].path_t >= 1.0f) {
            game->enemies[i].path_t -= 1.0f;
            game->enemies[i].path_idx++;
            seg = game->enemies[i].path_idx;
            if (seg >= PATH_LEN - 1) {
                game->enemies[i].active = false;
                game->lives--;
                if (game->lives <= 0) { game->lives = 0; game->game_over = true; }
                continue;
            }
            sx = grid_to_px((int)path_waypoints[seg].x);
            sy = grid_to_py((int)path_waypoints[seg].y);
            ex = grid_to_px((int)path_waypoints[seg + 1].x);
            ey = grid_to_py((int)path_waypoints[seg + 1].y);
        }

        float t = game->enemies[i].path_t;
        game->enemies[i].x = sx + (ex - sx) * t;
        game->enemies[i].y = sy + (ey - sy) * t;
    }

    /* Tower firing */
    for (int i = 0; i < MAX_TOWERS; i++) {
        if (!game->towers[i].active) continue;
        if (now - game->towers[i].last_fire_time < TOWER_FIRE_RATE_MS) continue;

        /* Find closest enemy in range */
        struct enemy *target = NULL;
        float best_dist = TOWER_RANGE + 1;
        for (int j = 0; j < MAX_ENEMIES; j++) {
            if (!game->enemies[j].active) continue;
            float dx = game->enemies[j].x - game->towers[i].world_x;
            float dy = game->enemies[j].y - game->towers[i].world_y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < TOWER_RANGE && dist < best_dist) {
                best_dist = dist;
                target = &game->enemies[j];
            }
        }
        if (target) fire_at_enemy(game, &game->towers[i], target);
    }

    /* Update projectiles */
    for (int i = 0; i < MAX_PROJECTILES; i++) {
        if (!game->projectiles[i].active) continue;
        game->projectiles[i].x += game->projectiles[i].dx;
        game->projectiles[i].y += game->projectiles[i].dy;

        /* Off screen? */
        if (game->projectiles[i].x < 0 || game->projectiles[i].x > SCREEN_W ||
            game->projectiles[i].y < 0 || game->projectiles[i].y > SCREEN_H) {
            game->projectiles[i].active = false;
            continue;
        }

        /* Hit enemy? */
        for (int j = 0; j < MAX_ENEMIES; j++) {
            if (!game->enemies[j].active) continue;
            float dx = game->projectiles[i].x - game->enemies[j].x;
            float dy = game->projectiles[i].y - game->enemies[j].y;
            if (dx * dx + dy * dy < 15 * 15) {
                game->projectiles[i].active = false;
                game->enemies[j].hp -= game->projectiles[i].damage;
                spawn_effect(game, game->enemies[j].x, game->enemies[j].y);
                if (game->enemies[j].hp <= 0) {
                    game->enemies[j].active = false;
                    game->gold += ENEMY_REWARD;
                    game->score += ENEMY_REWARD;
                }
                break;
            }
        }
    }

    /* Update effects */
    for (int i = 0; i < MAX_EFFECTS; i++) {
        if (!game->effects[i].active) continue;
        game->effects[i].life--;
        if (game->effects[i].life <= 0) game->effects[i].active = false;
    }

    /* Wave management */
    if (game->enemies_spawned >= game->enemies_per_wave && active_enemies == 0) {
        /* Wave complete - start next */
        game->wave++;
        game->enemies_spawned = 0;
        game->enemies_per_wave = 5 + game->wave * 2;
        game->spawn_interval_ms = 1500 - game->wave * 50;
        if (game->spawn_interval_ms < 500) game->spawn_interval_ms = 500;
        game->wave_start_time = now;
        game->last_spawn_time = now + 2000; /* brief pause between waves */
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

    printf("=== DRM Tower Defense (Overlay + Cursor Planes) ===\n");
    printf("Controls: Arrows to move, Space to place tower, Q/ESC to quit\n\n");

    if (drm_init(&dev, card_path) < 0) { fprintf(stderr, "Failed to initialize DRM\n"); return 1; }

    input_init();
    game_init(&game);

    struct timespec frame_start, frame_end;
    bool last_valid = true;
    draw_cursor(&dev.cursor_buf, true);

    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        input_poll();
        game_update(&game);

        /* Redraw cursor if validity changed */
        if (game.cursor_valid != last_valid) {
            draw_cursor(&dev.cursor_buf, game.cursor_valid);
            last_valid = game.cursor_valid;
        }

        draw_background(&dev, &game);
        draw_overlay(&dev, &game);

        /* Position cursor on grid cell */
        int cx = game.cursor_gx * CELL_W + CELL_W / 2 - CURSOR_W / 2;
        int cy = GRID_OFFSET_Y + game.cursor_gy * CELL_H + CELL_H / 2 - CURSOR_H / 2;

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
        drm_atomic_flip(&dev, cx, cy);

        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        uint64_t elapsed_ns = (frame_end.tv_sec - frame_start.tv_sec) * 1000000000ULL +
                              (frame_end.tv_nsec - frame_start.tv_nsec);
        if (elapsed_ns < FRAME_TIME_NS) sleep_ns(FRAME_TIME_NS - elapsed_ns);
    }

    if (game.game_over) {
        printf("[GAME] Game Over! Score: %d, Wave: %d\n", game.score, game.wave);
        while (!g_quit) { input_poll(); sleep_ns(50000000); }
    }

    input_cleanup();
    drm_cleanup(&dev);
    printf("Final Score: %d | Wave: %d\n", game.score, game.wave);
    return 0;
}
