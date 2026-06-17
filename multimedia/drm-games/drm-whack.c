/*
 * drm-whack.c - Whack-a-Mole using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Green field with mole holes, score/time HUD
 *   - Overlay plane:  Moles popping up, whack animations, score popups
 *   - Cursor plane:   Hammer/mallet (moves with player input)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-whack drm-whack.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-whack drm-whack.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-whack [/dev/dri/cardN]
 *
 * Controls:
 *   - Arrow keys: move hammer
 *   - Space: whack!
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

#define CURSOR_W            96
#define CURSOR_H            96

/* Grid of mole holes: 4 cols x 3 rows */
#define HOLE_COLS           4
#define HOLE_ROWS           3
#define HOLE_COUNT          (HOLE_COLS * HOLE_ROWS)
#define HOLE_SPACING_X      (SCREEN_W / (HOLE_COLS + 1))
#define HOLE_SPACING_Y      ((SCREEN_H - 80) / (HOLE_ROWS + 1))
#define HOLE_OFFSET_Y       120
#define HOLE_RADIUS         45

/* Game tuning */
#define HAMMER_SPEED        12
#define MOLE_UP_TIME_MS     1500
#define MOLE_DOWN_TIME_MS   800
#define WHACK_RADIUS        60
#define GAME_TIME_SEC       60
#define MOLE_SCORE          10
#define GOLDEN_MOLE_SCORE   50
#define MAX_POPUP_TEXTS     8

/* Colors */
#define COLOR_TRANSPARENT   0x00000000
#define COLOR_GRASS_LIGHT   0xFF228B22
#define COLOR_GRASS_DARK    0xFF1E7A1E
#define COLOR_DIRT          0xFF4A3520
#define COLOR_HOLE          0xFF1A1008
#define COLOR_HOLE_RIM      0xFF3D2817
#define COLOR_BORDER        0xFF114411
#define COLOR_HUD_BG        0xFF1A2A1A
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE_VAL     0xFF00FF80
#define COLOR_TIME_VAL      0xFFFFDD00
#define COLOR_COMBO_VAL     0xFFFF8800

#define COLOR_MOLE_BODY     0xFE8B6914
#define COLOR_MOLE_FACE     0xFEAA8833
#define COLOR_MOLE_NOSE     0xFEFF6688
#define COLOR_MOLE_EYE      0xFE111111
#define COLOR_GOLDEN_BODY   0xFEFFDD00
#define COLOR_GOLDEN_FACE   0xFEFFEE66

#define COLOR_HAMMER_HEAD   0xFE666666
#define COLOR_HAMMER_FACE   0xFE888888
#define COLOR_HAMMER_HANDLE 0xFE553311
#define COLOR_HAMMER_BAND   0xFEAA2222

#define COLOR_WHACK_STAR    0xFEFFFF00
#define COLOR_POPUP_GOOD    0xFE00FF00
#define COLOR_POPUP_GREAT   0xFEFFDD00

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

enum mole_state {
    MOLE_HIDDEN = 0,
    MOLE_RISING,
    MOLE_UP,
    MOLE_FALLING,
    MOLE_WHACKED
};

struct mole {
    int hole_x, hole_y;    /* pixel position of hole center */
    enum mole_state state;
    int anim_frame;
    int rise_height;       /* how far up the mole is (0=hidden, max=fully up) */
    uint64_t state_start;
    bool is_golden;
};

struct popup_text {
    float x, y;
    int value;
    int life;
    bool active;
};

struct whack_effect {
    float x, y;
    int frame;
    bool active;
};

#define MAX_WHACK_EFFECTS 6

struct game_state {
    int hammer_x, hammer_y;
    struct mole moles[HOLE_COUNT];
    struct popup_text popups[MAX_POPUP_TEXTS];
    struct whack_effect whacks[MAX_WHACK_EFFECTS];
    int score;
    int combo;
    int best_combo;
    int hits;
    int misses;
    uint64_t game_start_time;
    int time_remaining;
    bool running;
    bool game_over;
    bool hammer_down;    /* is hammer in striking position this frame */
    int hammer_anim;     /* animation frame for hammer strike */
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

    /* Create cursor plane buffer (hammer sprite, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at center - hammer starts at center) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (int32_t)(dev->width / 2 - CURSOR_W / 2),
                        (int32_t)(dev->height / 2 - CURSOR_H / 2),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Green field with mole holes + score/time HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Moles popping up, whack animations, score popups\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Hammer/mallet (player controlled)\n",
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

static void draw_ellipse(uint32_t *fb, uint32_t stride_px, uint32_t scr_w, uint32_t scr_h,
                         int cx, int cy, int rx, int ry, uint32_t color) {
    for (int dy = -ry; dy <= ry; dy++)
        for (int dx = -rx; dx <= rx; dx++)
            if ((dx*dx*ry*ry + dy*dy*rx*rx) <= rx*rx*ry*ry)
                put_pixel(fb, stride_px, scr_w, scr_h, cx+dx, cy+dy, color);
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
        if (str[i] == '+') {
            for (int s = 0; s < scale; s++) {
                put_pixel(fb, stride_px, sw, sh, x+2*scale+s, py+3*scale, color);
                put_pixel(fb, stride_px, sw, sh, x+scale, py+2*scale+s, color);
                put_pixel(fb, stride_px, sw, sh, x+3*scale, py+2*scale+s, color);
            }
            x += 5*scale; continue;
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
 * Drawing Game Elements
 * ============================================================ */

/* Draw hammer sprite */
static void draw_hammer(struct drm_buffer *buf, bool striking) {
    uint32_t *fb = buf->map;
    uint32_t stride_px = buf->stride / 4;
    uint32_t w = buf->width, h = buf->height;

    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    int cx = w / 2, cy = h / 2;

    if (striking) {
        /* Hammer tilted forward (striking position) */
        /* Handle (diagonal) */
        for (int i = 0; i < 30; i++) {
            draw_rect(fb, stride_px, w, h, cx + 5 + i / 2, cy + i, 6, 2, COLOR_HAMMER_HANDLE);
        }
        /* Head (rotated) */
        draw_rect(fb, stride_px, w, h, cx - 15, cy - 10, 35, 20, COLOR_HAMMER_HEAD);
        draw_rect(fb, stride_px, w, h, cx - 12, cy - 7, 29, 14, COLOR_HAMMER_FACE);
        /* Band */
        draw_rect(fb, stride_px, w, h, cx + 5, cy - 10, 4, 20, COLOR_HAMMER_BAND);
    } else {
        /* Hammer upright (ready position) */
        /* Handle */
        draw_rect(fb, stride_px, w, h, cx - 3, cy, 6, 35, COLOR_HAMMER_HANDLE);
        /* Head */
        draw_rect(fb, stride_px, w, h, cx - 18, cy - 20, 36, 22, COLOR_HAMMER_HEAD);
        draw_rect(fb, stride_px, w, h, cx - 15, cy - 17, 30, 16, COLOR_HAMMER_FACE);
        /* Band */
        draw_rect(fb, stride_px, w, h, cx - 3, cy - 20, 6, 22, COLOR_HAMMER_BAND);
    }
}

/* Draw background (primary): grass field + holes + HUD */
static void draw_background(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride_px = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width, h = dev->height;

    /* HUD bar */
    draw_rect(fb, stride_px, w, h, 0, 0, w, HOLE_OFFSET_Y - 40, COLOR_HUD_BG);

    /* Grass field */
    for (uint32_t y = HOLE_OFFSET_Y - 40; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint32_t color = ((x / 20 + y / 20) % 2) ? COLOR_GRASS_LIGHT : COLOR_GRASS_DARK;
            fb[y * stride_px + x] = color;
        }
    }

    /* Draw holes */
    for (int row = 0; row < HOLE_ROWS; row++) {
        for (int col = 0; col < HOLE_COLS; col++) {
            int hx = HOLE_SPACING_X * (col + 1);
            int hy = HOLE_OFFSET_Y + HOLE_SPACING_Y * (row + 1);

            /* Dirt mound around hole */
            draw_ellipse(fb, stride_px, w, h, hx, hy + 10, HOLE_RADIUS + 10, 20, COLOR_DIRT);
            /* Dark hole */
            draw_ellipse(fb, stride_px, w, h, hx, hy, HOLE_RADIUS, 25, COLOR_HOLE);
            /* Rim */
            for (int a = -HOLE_RADIUS; a <= HOLE_RADIUS; a++) {
                put_pixel(fb, stride_px, w, h, hx + a, hy - 24, COLOR_HOLE_RIM);
                put_pixel(fb, stride_px, w, h, hx + a, hy - 25, COLOR_HOLE_RIM);
            }
        }
    }

    /* Border */
    draw_rect(fb, stride_px, w, h, 0, 0, w, 3, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, h - 3, w, 3, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, 0, 0, 3, h, COLOR_BORDER);
    draw_rect(fb, stride_px, w, h, w - 3, 0, 3, h, COLOR_BORDER);

    /* HUD */
    draw_string(fb, stride_px, w, h, 20, 15, "SCORE:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 100, 15, game->score, 2, COLOR_SCORE_VAL);

    draw_string(fb, stride_px, w, h, 250, 15, "TIME:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 320, 15, game->time_remaining, 2, COLOR_TIME_VAL);

    draw_string(fb, stride_px, w, h, 470, 15, "COMBO:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 550, 15, game->combo, 2, COLOR_COMBO_VAL);

    draw_string(fb, stride_px, w, h, 700, 15, "BEST:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, 770, 15, game->best_combo, 2, COLOR_COMBO_VAL);

    draw_string(fb, stride_px, w, h, w - 200, 15, "HITS:", 2, COLOR_HUD_TEXT);
    draw_number(fb, stride_px, w, h, w - 140, 15, game->hits, 2, COLOR_SCORE_VAL);

    if (game->game_over) {
        draw_string(fb, stride_px, w, h, w / 2 - 100, h / 2 - 30, "TIME UP", 4, 0xFFFF4400);
        draw_string(fb, stride_px, w, h, w / 2 - 80, h / 2 + 25, "PRESS Q", 3, COLOR_HUD_TEXT);
    }
}

/* Draw mole at given position with given rise height */
static void draw_mole(uint32_t *fb, uint32_t stride_px, uint32_t w, uint32_t h,
                      int cx, int cy, int rise, bool golden, bool whacked) {
    /* Mole appears from below the hole - only draw what's above the "surface" */
    int mole_top = cy - rise;
    (void)whacked;

    uint32_t body_color = golden ? COLOR_GOLDEN_BODY : COLOR_MOLE_BODY;
    uint32_t face_color = golden ? COLOR_GOLDEN_FACE : COLOR_MOLE_FACE;

    /* Body (oval) - clip to above hole line */
    for (int dy = -30; dy <= 10; dy++) {
        int actual_y = mole_top + 15 + dy;
        if (actual_y > cy - 5) continue; /* below hole surface, clip */
        int rx = 20 - abs(dy) / 3;
        if (rx < 5) rx = 5;
        for (int dx = -rx; dx <= rx; dx++)
            put_pixel(fb, stride_px, w, h, cx + dx, actual_y, body_color);
    }

    /* Face (lighter circle) */
    if (rise > 15) {
        int face_y = mole_top + 8;
        if (face_y < cy - 10) {
            draw_circle(fb, stride_px, w, h, cx, face_y, 12, face_color);
            /* Eyes */
            if (whacked) {
                /* X eyes when whacked */
                for (int i = -2; i <= 2; i++) {
                    put_pixel(fb, stride_px, w, h, cx - 6 + i, face_y - 2 + i, COLOR_MOLE_EYE);
                    put_pixel(fb, stride_px, w, h, cx - 6 + i, face_y + 2 - i, COLOR_MOLE_EYE);
                    put_pixel(fb, stride_px, w, h, cx + 6 + i, face_y - 2 + i, COLOR_MOLE_EYE);
                    put_pixel(fb, stride_px, w, h, cx + 6 + i, face_y + 2 - i, COLOR_MOLE_EYE);
                }
            } else {
                draw_circle(fb, stride_px, w, h, cx - 6, face_y - 2, 3, COLOR_MOLE_EYE);
                draw_circle(fb, stride_px, w, h, cx + 6, face_y - 2, 3, COLOR_MOLE_EYE);
            }
            /* Nose */
            draw_circle(fb, stride_px, w, h, cx, face_y + 4, 3, COLOR_MOLE_NOSE);
        }
    }
}

/* Draw overlay (moles + effects) */
static void draw_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride_px = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W, h = OVERLAY_H;

    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw moles */
    for (int i = 0; i < HOLE_COUNT; i++) {
        if (game->moles[i].state == MOLE_HIDDEN) continue;

        draw_mole(fb, stride_px, w, h,
                  game->moles[i].hole_x, game->moles[i].hole_y,
                  game->moles[i].rise_height,
                  game->moles[i].is_golden,
                  game->moles[i].state == MOLE_WHACKED);
    }

    /* Draw whack effects */
    for (int i = 0; i < MAX_WHACK_EFFECTS; i++) {
        if (!game->whacks[i].active) continue;
        int ex = (int)game->whacks[i].x;
        int ey = (int)game->whacks[i].y;
        int f = game->whacks[i].frame;

        /* Star burst */
        int r = 8 + f * 4;
        for (int a = 0; a < 8; a++) {
            float angle = a * M_PI / 4 + f * 0.2f;
            int sx = ex + (int)(r * cosf(angle));
            int sy = ey + (int)(r * sinf(angle));
            draw_circle(fb, stride_px, w, h, sx, sy, 3 - f / 3, COLOR_WHACK_STAR);
        }
    }

    /* Draw popup texts */
    for (int i = 0; i < MAX_POPUP_TEXTS; i++) {
        if (!game->popups[i].active) continue;
        int px = (int)game->popups[i].x;
        int py = (int)game->popups[i].y;
        uint32_t color = game->popups[i].value >= GOLDEN_MOLE_SCORE ? COLOR_POPUP_GREAT : COLOR_POPUP_GOOD;

        char buf[8];
        snprintf(buf, sizeof(buf), "+%d", game->popups[i].value);
        int x = px;
        for (int j = 0; buf[j]; j++) {
            if (buf[j] == '+') {
                /* simple + */
                draw_rect(fb, stride_px, w, h, x + 1, py + 3, 5, 2, color);
                draw_rect(fb, stride_px, w, h, x + 2, py + 1, 2, 6, color);
                x += 8;
            } else if (buf[j] >= '0' && buf[j] <= '9') {
                draw_char(fb, stride_px, w, h, x, py, buf[j], 2, color);
                x += 12;
            }
        }
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

static void game_init(struct game_state *game) {
    memset(game, 0, sizeof(*game));
    game->hammer_x = SCREEN_W / 2;
    game->hammer_y = SCREEN_H / 2;
    game->running = true;
    game->game_start_time = get_time_ms();
    game->time_remaining = GAME_TIME_SEC;
    srand(time(NULL));

    /* Initialize mole hole positions */
    for (int row = 0; row < HOLE_ROWS; row++) {
        for (int col = 0; col < HOLE_COLS; col++) {
            int idx = row * HOLE_COLS + col;
            game->moles[idx].hole_x = HOLE_SPACING_X * (col + 1);
            game->moles[idx].hole_y = HOLE_OFFSET_Y + HOLE_SPACING_Y * (row + 1);
            game->moles[idx].state = MOLE_HIDDEN;
        }
    }
}

static void spawn_popup(struct game_state *game, float x, float y, int value) {
    for (int i = 0; i < MAX_POPUP_TEXTS; i++) {
        if (!game->popups[i].active) {
            game->popups[i].active = true;
            game->popups[i].x = x - 15;
            game->popups[i].y = y - 30;
            game->popups[i].value = value;
            game->popups[i].life = 30;
            return;
        }
    }
}

static void spawn_whack_effect(struct game_state *game, float x, float y) {
    for (int i = 0; i < MAX_WHACK_EFFECTS; i++) {
        if (!game->whacks[i].active) {
            game->whacks[i].active = true;
            game->whacks[i].x = x;
            game->whacks[i].y = y;
            game->whacks[i].frame = 0;
            return;
        }
    }
}

static void game_update(struct game_state *game) {
    if (game->game_over) return;

    uint64_t now = get_time_ms();

    /* Update time */
    int elapsed = (int)(now - game->game_start_time) / 1000;
    game->time_remaining = GAME_TIME_SEC - elapsed;
    if (game->time_remaining <= 0) {
        game->time_remaining = 0;
        game->game_over = true;
        return;
    }

    /* Move hammer */
    if (key_left) game->hammer_x -= HAMMER_SPEED;
    if (key_right) game->hammer_x += HAMMER_SPEED;
    if (key_up) game->hammer_y -= HAMMER_SPEED;
    if (key_down) game->hammer_y += HAMMER_SPEED;
    if (game->hammer_x < CURSOR_W / 2) game->hammer_x = CURSOR_W / 2;
    if (game->hammer_x > SCREEN_W - CURSOR_W / 2) game->hammer_x = SCREEN_W - CURSOR_W / 2;
    if (game->hammer_y < CURSOR_H / 2) game->hammer_y = CURSOR_H / 2;
    if (game->hammer_y > SCREEN_H - CURSOR_H / 2) game->hammer_y = SCREEN_H - CURSOR_H / 2;

    /* Whack action */
    if (key_space && !key_space_prev) {
        game->hammer_down = true;
        game->hammer_anim = 5;

        /* Check if hitting any mole */
        bool hit_any = false;
        for (int i = 0; i < HOLE_COUNT; i++) {
            if (game->moles[i].state != MOLE_UP && game->moles[i].state != MOLE_RISING)
                continue;

            float dx = game->hammer_x - game->moles[i].hole_x;
            float dy = game->hammer_y - (game->moles[i].hole_y - game->moles[i].rise_height / 2);
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist < WHACK_RADIUS) {
                game->moles[i].state = MOLE_WHACKED;
                game->moles[i].state_start = now;
                int score = game->moles[i].is_golden ? GOLDEN_MOLE_SCORE : MOLE_SCORE;
                score += game->combo * 5;
                game->score += score;
                game->hits++;
                game->combo++;
                if (game->combo > game->best_combo) game->best_combo = game->combo;
                spawn_popup(game, game->moles[i].hole_x, game->moles[i].hole_y - 40, score);
                spawn_whack_effect(game, game->moles[i].hole_x, game->moles[i].hole_y - 20);
                hit_any = true;
            }
        }

        if (!hit_any) {
            game->combo = 0;
            game->misses++;
        }
    }

    /* Hammer animation */
    if (game->hammer_anim > 0) game->hammer_anim--;
    game->hammer_down = (game->hammer_anim > 0);

    /* Randomly pop up moles */
    /* Difficulty increases over time */
    int active_moles = 0;
    for (int i = 0; i < HOLE_COUNT; i++)
        if (game->moles[i].state != MOLE_HIDDEN) active_moles++;

    int max_active = 2 + (GAME_TIME_SEC - game->time_remaining) / 15;
    if (max_active > 6) max_active = 6;

    if (active_moles < max_active && (rand() % 10) < 3) {
        /* Pick a random hidden mole */
        int start = rand() % HOLE_COUNT;
        for (int j = 0; j < HOLE_COUNT; j++) {
            int idx = (start + j) % HOLE_COUNT;
            if (game->moles[idx].state == MOLE_HIDDEN) {
                game->moles[idx].state = MOLE_RISING;
                game->moles[idx].state_start = now;
                game->moles[idx].rise_height = 0;
                game->moles[idx].is_golden = (rand() % 100) < 10;
                break;
            }
        }
    }

    /* Update mole states */
    for (int i = 0; i < HOLE_COUNT; i++) {
        uint64_t dt = now - game->moles[i].state_start;

        switch (game->moles[i].state) {
        case MOLE_HIDDEN:
            break;
        case MOLE_RISING:
            game->moles[i].rise_height = (int)(40.0f * dt / 300.0f);
            if (game->moles[i].rise_height >= 40) {
                game->moles[i].rise_height = 40;
                game->moles[i].state = MOLE_UP;
                game->moles[i].state_start = now;
            }
            break;
        case MOLE_UP:
            if (dt > MOLE_UP_TIME_MS) {
                game->moles[i].state = MOLE_FALLING;
                game->moles[i].state_start = now;
            }
            break;
        case MOLE_FALLING:
            game->moles[i].rise_height = 40 - (int)(40.0f * dt / 300.0f);
            if (game->moles[i].rise_height <= 0) {
                game->moles[i].rise_height = 0;
                game->moles[i].state = MOLE_HIDDEN;
            }
            break;
        case MOLE_WHACKED:
            /* Brief stunned animation then fall */
            if (dt > 400) {
                game->moles[i].state = MOLE_FALLING;
                game->moles[i].state_start = now;
            }
            break;
        }
    }

    /* Update popup texts */
    for (int i = 0; i < MAX_POPUP_TEXTS; i++) {
        if (!game->popups[i].active) continue;
        game->popups[i].y -= 1.5f;
        game->popups[i].life--;
        if (game->popups[i].life <= 0) game->popups[i].active = false;
    }

    /* Update whack effects */
    for (int i = 0; i < MAX_WHACK_EFFECTS; i++) {
        if (!game->whacks[i].active) continue;
        game->whacks[i].frame++;
        if (game->whacks[i].frame > 8) game->whacks[i].active = false;
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

    printf("=== DRM Whack-a-Mole (Overlay + Cursor Planes) ===\n");
    printf("Controls: Arrows to move hammer, Space to whack, Q/ESC to quit\n\n");

    if (drm_init(&dev, card_path) < 0) { fprintf(stderr, "Failed to initialize DRM\n"); return 1; }

    input_init();
    game_init(&game);

    /* Draw initial hammer */
    draw_hammer(&dev.cursor_buf, false);

    struct timespec frame_start, frame_end;
    bool last_hammer_state = false;

    while (!g_quit && game.running) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        input_poll();
        game_update(&game);

        /* Redraw hammer sprite if strike state changed */
        if (game.hammer_down != last_hammer_state) {
            draw_hammer(&dev.cursor_buf, game.hammer_down);
            last_hammer_state = game.hammer_down;
        }

        draw_background(&dev, &game);
        draw_overlay(&dev, &game);

        int cx = game.hammer_x - CURSOR_W / 2;
        int cy = game.hammer_y - CURSOR_H / 2;

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
        printf("[GAME] Time's up! Score: %d, Hits: %d, Best Combo: %d\n",
               game.score, game.hits, game.best_combo);
        while (!g_quit) { input_poll(); sleep_ns(50000000); }
    }

    input_cleanup();
    drm_cleanup(&dev);
    printf("Final Score: %d | Hits: %d | Best Combo: %d\n", game.score, game.hits, game.best_combo);
    return 0;
}
