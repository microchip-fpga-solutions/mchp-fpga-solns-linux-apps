/*
 * drm-ninja.c - Fruit Ninja Style Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Wooden board background + score/combo HUD
 *   - Overlay plane:  Flying fruits, slash trails, splatter effects
 *   - Cursor plane:   Blade/crosshair indicator
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-ninja drm-ninja.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-ninja drm-ninja.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-ninja [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right/Up/Down arrow keys: move blade
 *   - Space: slash (cut fruits within blade range)
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

#define CURSOR_W            64
#define CURSOR_H            64

/* Game tuning */
#define BLADE_SPEED         10
#define MAX_FRUITS          10
#define MAX_SPLATTERS       20
#define MAX_SLASH_TRAIL     12
#define FRUIT_RADIUS        22
#define SLASH_RADIUS        50
#define SPAWN_INTERVAL_MS   1000
#define GRAVITY             0.15f
#define MAX_MISSES          5

#define FRUIT_SCORE         10
#define COMBO_BONUS         5

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG_WOOD1      0xFF3D2817
#define COLOR_BG_WOOD2      0xFF4A3020
#define COLOR_BG_GRAIN      0xFF2A1B0F
#define COLOR_BORDER        0xFF5C3A1E
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE_VAL     0xFF00FF80
#define COLOR_COMBO_VAL     0xFFFFDD00
#define COLOR_MISS_VAL      0xFFFF4444

#define COLOR_WATERMELON    0xFF22BB22
#define COLOR_WATERMELON_IN 0xFFFF3344
#define COLOR_ORANGE        0xFFFF8800
#define COLOR_ORANGE_IN     0xFFFFBB44
#define COLOR_GRAPE         0xFF6622CC
#define COLOR_GRAPE_IN      0xFFAA66FF
#define COLOR_APPLE         0xFFDD1111
#define COLOR_APPLE_IN      0xFFFF8888
#define COLOR_BOMB          0xFF222222

#define COLOR_BLADE         0xFFCCCCFF
#define COLOR_BLADE_EDGE    0xFFFFFFFF
#define COLOR_BLADE_GLOW    0xFF6666FF

#define COLOR_SLASH         0xFFAABBFF

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

enum fruit_type {
    FRUIT_WATERMELON = 0,
    FRUIT_ORANGE,
    FRUIT_GRAPE,
    FRUIT_APPLE,
    FRUIT_BOMB,
    FRUIT_TYPE_COUNT
};

struct fruit {
    float x, y;
    float vx, vy;
    enum fruit_type type;
    bool active;
    bool sliced;
    int slice_frame;
};

struct splatter {
    float x, y;
    float dx, dy;
    int life;
    uint32_t color;
    bool active;
};

struct slash_point {
    int x, y;
    int life;
};

struct game_state {
    int blade_x, blade_y;
    struct fruit fruits[MAX_FRUITS];
    struct splatter splatters[MAX_SPLATTERS];
    struct slash_point trail[MAX_SLASH_TRAIL];
    int trail_head;
    int score;
    int combo;
    int misses;
    int best_combo;
    bool running;
    bool game_over;
    bool slashing;
    uint64_t last_spawn_time;
    int spawn_interval_ms;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false;
static bool key_right = false;
static bool key_up = false;
static bool key_down = false;
static bool key_space = false;

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

        drmModeObjectProperties *props = drmModeObjectGetProperties(dev->fd, plane_res->planes[i], DRM_MODE_OBJECT_PLANE);
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

    /* Create cursor plane buffer (blade sprite, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at center - blade start position) */
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

    printf("[DRM] Initialized (atomic, %d buffers): primary=%ux%u, overlay=%ux%u, cursor=%ux%u\n",
           NUM_BUFFERS, dev->width, dev->height, OVERLAY_W, OVERLAY_H, CURSOR_W, CURSOR_H);

    printf("\n");
    printf("[PLANE INFO] This game uses 3 DRM/KMS planes (atomic commit):\n");
    printf("[PLANE INFO]   1. Primary plane (id=%u): Wooden board background + score/combo HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Flying fruits, slash trails, splatter effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Blade/crosshair indicator\n",
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

/* Font (same as drm-catch) */
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

/* Draw blade cursor */
static void draw_blade_cursor(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    int cx = w / 2, cy = h / 2;

    /* Crosshair */
    for (int i = -20; i <= 20; i++) {
        if (abs(i) > 4) {
            put_pixel(fb, stride_px, w, h, cx + i, cy, COLOR_BLADE_EDGE);
            put_pixel(fb, stride_px, w, h, cx, cy + i, COLOR_BLADE_EDGE);
        }
    }

    /* Center glow */
    draw_circle(fb, stride_px, w, h, cx, cy, 4, COLOR_BLADE_GLOW);
    draw_circle(fb, stride_px, w, h, cx, cy, 2, COLOR_BLADE_EDGE);

    /* Diagonal slash marks */
    for (int i = 6; i < 16; i++) {
        put_pixel(fb, stride_px, w, h, cx + i, cy + i, COLOR_BLADE);
        put_pixel(fb, stride_px, w, h, cx - i, cy - i, COLOR_BLADE);
        put_pixel(fb, stride_px, w, h, cx + i, cy - i, COLOR_BLADE);
        put_pixel(fb, stride_px, w, h, cx - i, cy + i, COLOR_BLADE);
    }
}

/* Draw background (primary) */
static void draw_background(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Wood texture background */
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            /* Simple wood grain pattern */
            int grain = ((x * 7 + y * 3) % 40);
            uint32_t color;
            if (grain < 2)
                color = COLOR_BG_GRAIN;
            else if ((y % 120) < 2)
                color = COLOR_BG_GRAIN;
            else
                color = ((x + y) % 2) ? COLOR_BG_WOOD1 : COLOR_BG_WOOD2;
            fb[y * stride_px + x] = color;
        }
    }

    /* Border */
    draw_rect(fb, stride_px, w, h, 0, 0, w, 4, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, h - 4, w, 4, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, 0, 4, h, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, w - 4, 0, 4, h, COLOR_BORDER);

    /* HUD */
    draw_string(fb, stride_px, w, h, 20, 15, "SCORE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 100, 15, game->score, 2, COLOR_SCORE_VAL);

    draw_string(fb, stride_px, w, h, 250, 15, "COMBO:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 340, 15, game->combo, 2, COLOR_COMBO_VAL);

    draw_string(fb, stride_px, w, h, w - 200, 15, "MISS:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w - 130, 15, game->misses, 2, COLOR_MISS_VAL);

    draw_string(fb, stride_px, w, h, w / 2 - 60, 15, "BEST:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w / 2 + 10, 15, game->best_combo, 2, COLOR_COMBO_VAL);

    if (game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 - 20, "GAME OVER", 4, 0xFFFF0000);
        draw_string(fb, stride_px, w, h, w / 2 - 80, h / 2 + 30, "PRESS Q", 3, COLOR_HUD_TEXT);
    }
}

/* Get fruit colors */
static void get_fruit_colors(enum fruit_type type, uint32_t *outer, uint32_t *inner) {
    switch (type) {
    case FRUIT_WATERMELON: *outer = COLOR_WATERMELON; *inner = COLOR_WATERMELON_IN; break;
    case FRUIT_ORANGE:     *outer = COLOR_ORANGE;     *inner = COLOR_ORANGE_IN;     break;
    case FRUIT_GRAPE:      *outer = COLOR_GRAPE;      *inner = COLOR_GRAPE_IN;      break;
    case FRUIT_APPLE:      *outer = COLOR_APPLE;      *inner = COLOR_APPLE_IN;      break;
    default:               *outer = COLOR_BOMB;       *inner = 0xFF444444;          break;
    }
}

/* Draw overlay (fruits + effects) */
static void draw_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw fruits */
    for (int i = 0; i < MAX_FRUITS; i++) {
        if (!game->fruits[i].active) continue;
        int fx = (int)game->fruits[i].x;
        int fy = (int)game->fruits[i].y;

        uint32_t outer, inner;
        get_fruit_colors(game->fruits[i].type, &outer, &inner);

        if (game->fruits[i].sliced) {
            /* Draw two halves separating */
            int offset = game->fruits[i].slice_frame * 2;
            draw_circle(fb, stride_px, w, h, fx - offset, fy, FRUIT_RADIUS / 2 + 2, outer);
            draw_circle(fb, stride_px, w, h, fx - offset, fy, FRUIT_RADIUS / 2 - 2, inner);
            draw_circle(fb, stride_px, w, h, fx + offset, fy, FRUIT_RADIUS / 2 + 2, outer);
            draw_circle(fb, stride_px, w, h, fx + offset, fy, FRUIT_RADIUS / 2 - 2, inner);
        } else {
            /* Whole fruit */
            draw_circle(fb, stride_px, w, h, fx, fy, FRUIT_RADIUS, outer);
            draw_circle(fb, stride_px, w, h, fx, fy, FRUIT_RADIUS - 5, inner);
            /* Highlight */
            draw_circle(fb, stride_px, w, h, fx - 5, fy - 5, 4, 0x44FFFFFF);
        }
    }

    /* Draw slash trail */
    for (int i = 0; i < MAX_SLASH_TRAIL; i++) {
        if (game->trail[i].life <= 0) continue;
        int size = game->trail[i].life / 2;
        if (size < 1) size = 1;
        uint32_t alpha = (game->trail[i].life * 255 / 15) & 0xFF;
        uint32_t color = (alpha << 24) | (COLOR_SLASH & 0x00FFFFFF);
        draw_circle(fb, stride_px, w, h, game->trail[i].x, game->trail[i].y, size, color);
    }

    /* Draw splatters */
    for (int i = 0; i < MAX_SPLATTERS; i++) {
        if (!game->splatters[i].active) continue;
        int sx = (int)game->splatters[i].x;
        int sy = (int)game->splatters[i].y;
        int size = game->splatters[i].life > 10 ? 4 : 2;
        draw_rect(fb, stride_px, w, h, sx - size / 2, sy - size / 2, size, size,
                  game->splatters[i].color);
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

static void input_cleanup(void) {
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static void input_poll(void) {
    struct input_event ev;
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

static void game_init(struct game_state *game) {
    memset(game, 0, sizeof(*game));
    game->blade_x = SCREEN_W / 2;
    game->blade_y = SCREEN_H / 2;
    game->running = true;
    game->last_spawn_time = get_time_ms();
    game->spawn_interval_ms = SPAWN_INTERVAL_MS;
    srand(time(NULL));
}

static void spawn_fruit(struct game_state *game) {
    for (int i = 0; i < MAX_FRUITS; i++) {
        if (!game->fruits[i].active) {
            game->fruits[i].active = true;
            game->fruits[i].sliced = false;
            game->fruits[i].slice_frame = 0;

            /* Launch from bottom of screen at various angles */
            game->fruits[i].x = 100 + rand() % (SCREEN_W - 200);
            game->fruits[i].y = SCREEN_H + FRUIT_RADIUS;
            game->fruits[i].vx = ((rand() % 100) - 50) / 15.0f;
            game->fruits[i].vy = -(8.0f + (rand() % 50) / 10.0f);

            /* 15% chance of bomb */
            if (rand() % 100 < 15)
                game->fruits[i].type = FRUIT_BOMB;
            else
                game->fruits[i].type = rand() % 4;
            return;
        }
    }
}

static void spawn_splatter(struct game_state *game, float x, float y, uint32_t color) {
    for (int n = 0; n < 5; n++) {
        for (int i = 0; i < MAX_SPLATTERS; i++) {
            if (!game->splatters[i].active) {
                game->splatters[i].active = true;
                game->splatters[i].x = x;
                game->splatters[i].y = y;
                game->splatters[i].dx = ((rand() % 100) - 50) / 10.0f;
                game->splatters[i].dy = ((rand() % 100) - 50) / 10.0f;
                game->splatters[i].life = 20 + rand() % 15;
                game->splatters[i].color = color;
                break;
            }
        }
    }
}

static void game_update(struct game_state *game) {
    if (game->game_over) return;

    /* Move blade */
    if (key_left) game->blade_x -= BLADE_SPEED;
    if (key_right) game->blade_x += BLADE_SPEED;
    if (key_up) game->blade_y -= BLADE_SPEED;
    if (key_down) game->blade_y += BLADE_SPEED;

    if (game->blade_x < CURSOR_W / 2) game->blade_x = CURSOR_W / 2;
    if (game->blade_x > SCREEN_W - CURSOR_W / 2) game->blade_x = SCREEN_W - CURSOR_W / 2;
    if (game->blade_y < CURSOR_H / 2) game->blade_y = CURSOR_H / 2;
    if (game->blade_y > SCREEN_H - CURSOR_H / 2) game->blade_y = SCREEN_H - CURSOR_H / 2;

    /* Add to slash trail when slashing */
    if (key_space) {
        game->slashing = true;
        game->trail[game->trail_head].x = game->blade_x;
        game->trail[game->trail_head].y = game->blade_y;
        game->trail[game->trail_head].life = 15;
        game->trail_head = (game->trail_head + 1) % MAX_SLASH_TRAIL;
    } else {
        game->slashing = false;
        if (game->combo > game->best_combo) game->best_combo = game->combo;
        game->combo = 0;
    }

    /* Spawn fruits */
    uint64_t now = get_time_ms();
    if (now - game->last_spawn_time > (uint64_t)game->spawn_interval_ms) {
        spawn_fruit(game);
        /* Sometimes spawn 2 at once */
        if (rand() % 3 == 0) spawn_fruit(game);
        game->last_spawn_time = now;
    }

    /* Update fruits */
    for (int i = 0; i < MAX_FRUITS; i++) {
        if (!game->fruits[i].active) continue;

        if (game->fruits[i].sliced) {
            game->fruits[i].slice_frame++;
            if (game->fruits[i].slice_frame > 20)
                game->fruits[i].active = false;
            continue;
        }

        game->fruits[i].x += game->fruits[i].vx;
        game->fruits[i].y += game->fruits[i].vy;
        game->fruits[i].vy += GRAVITY;

        /* Check if slash hits fruit */
        if (game->slashing) {
            float dx = game->fruits[i].x - game->blade_x;
            float dy = game->fruits[i].y - game->blade_y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < SLASH_RADIUS) {
                if (game->fruits[i].type == FRUIT_BOMB) {
                    /* Hit a bomb - penalty */
                    game->misses += 2;
                    spawn_splatter(game, game->fruits[i].x, game->fruits[i].y, 0xFF444444);
                } else {
                    /* Sliced a fruit! */
                    game->score += FRUIT_SCORE + game->combo * COMBO_BONUS;
                    game->combo++;
                    uint32_t outer, inner;
                    get_fruit_colors(game->fruits[i].type, &outer, &inner);
                    spawn_splatter(game, game->fruits[i].x, game->fruits[i].y, inner);
                }
                game->fruits[i].sliced = true;
                game->fruits[i].slice_frame = 0;
            }
        }

        /* Fruit fell off screen without being sliced */
        if (game->fruits[i].y > SCREEN_H + FRUIT_RADIUS * 2 && game->fruits[i].vy > 0) {
            if (game->fruits[i].type != FRUIT_BOMB) {
                game->misses++;
            }
            game->fruits[i].active = false;
        }
    }

    /* Decay trail */
    for (int i = 0; i < MAX_SLASH_TRAIL; i++) {
        if (game->trail[i].life > 0) game->trail[i].life--;
    }

    /* Update splatters */
    for (int i = 0; i < MAX_SPLATTERS; i++) {
        if (!game->splatters[i].active) continue;
        game->splatters[i].x += game->splatters[i].dx;
        game->splatters[i].y += game->splatters[i].dy;
        game->splatters[i].dy += 0.15f;
        game->splatters[i].life--;
        if (game->splatters[i].life <= 0) game->splatters[i].active = false;
    }

    /* Game over check */
    if (game->misses >= MAX_MISSES) {
        game->game_over = true;
        if (game->combo > game->best_combo) game->best_combo = game->combo;
    }

    /* Difficulty increase */
    game->spawn_interval_ms = SPAWN_INTERVAL_MS - (game->score / 50) * 50;
    if (game->spawn_interval_ms < 400) game->spawn_interval_ms = 400;
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

    printf("=== DRM Fruit Ninja (Overlay + Cursor Planes) ===\n");
    printf("Controls: Arrows to move blade, Space to slash, Q/ESC to quit\n\n");

    if (drm_init(&dev, card_path) < 0) { fprintf(stderr, "Failed to initialize DRM\n"); return 1; }

    input_init();
    game_init(&game);
    draw_blade_cursor(&dev.cursor_buf);

    struct timespec frame_start, frame_end;
    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        input_poll();
        game_update(&game);

        /* Render primary plane (background + HUD) into back buffer */
        draw_background(&dev, &game);

        /* Render overlay plane (fruits + effects) into back buffer */
        draw_overlay(&dev, &game);

        /* Compute cursor plane position (blade movement) */
        int cx = game.blade_x - CURSOR_W / 2;
        int cy = game.blade_y - CURSOR_H / 2;

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
        printf("[GAME] Game Over! Score: %d, Best Combo: %d\n", game.score, game.best_combo);
        while (!g_quit) { input_poll(); sleep_ns(50000000); }
    }

    input_cleanup();
    drm_cleanup(&dev);
    printf("Final Score: %d | Best Combo: %d\n", game.score, game.best_combo);
    return 0;
}
