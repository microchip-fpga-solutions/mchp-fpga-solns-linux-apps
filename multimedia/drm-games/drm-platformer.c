/*
 * drm-platformer.c - Side-Scrolling Platformer using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background (sky/ground/platforms) + score/level HUD
 *   - Overlay plane:  Enemies, collectibles, particles/effects
 *   - Cursor plane:   Player character (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-platformer drm-platformer.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-platformer drm-platformer.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-platformer [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right arrows: move
 *   - Space/Up: jump
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

/* Cursor plane dimensions (player character) */
#define CURSOR_W            64
#define CURSOR_H            64

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

/* Game tuning */
#define GRAVITY             0.5f
#define JUMP_FORCE          -10.0f
#define MOVE_SPEED          4.0f
#define MAX_FALL_SPEED      12.0f

#define PLAYER_W            20
#define PLAYER_H            32

#define MAX_PLATFORMS       20
#define MAX_COINS           15
#define MAX_PARTICLES       40
#define MAX_CLOUDS          8
#define MAX_MOUNTAINS       6
#define MAX_TREES           12

#define PLATFORM_MIN_W      60
#define PLATFORM_MAX_W      150
#define PLATFORM_H          14

#define COLOR_SKY_TOP       0xFF87CEEB
#define COLOR_SKY_BOT       0xFFE0F0FF
#define COLOR_GROUND        0xFF4A7A23
#define COLOR_GROUND_DARK   0xFF2D5016
#define COLOR_PLAYER_BODY   0xFF2255AA
#define COLOR_PLAYER_HEAD   0xFFFFCC99
#define COLOR_PLAYER_FEET   0xFF884422
#define COLOR_COIN          0xFFFFD700
#define COLOR_COIN_SHINE    0xFFFFFF88
#define COLOR_CLOUD         0xFFFFFFFF
#define COLOR_MOUNTAIN_FAR  0xFF8899BB
#define COLOR_MOUNTAIN_NEAR 0xFF556677
#define COLOR_TREE_TRUNK    0xFF664422
#define COLOR_TREE_LEAVES   0xFF228833
#define COLOR_TRANSPARENT   0x00000000

/* Platform gradient colors */
static const uint32_t plat_colors[] = {
    0xFF44AA44, 0xFF55BB55, 0xFF337733,
    0xFF8B6914, 0xFFA07828, 0xFF6B4E0A,
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

struct platform {
    float x, y;
    int w, h;
    uint32_t color;
    bool active;
};

struct coin {
    float x, y;
    float anim_offset;
    bool collected;
    bool active;
};

struct particle {
    float x, y, vx, vy;
    float life;
    uint32_t color;
    bool active;
};

struct cloud {
    float x, y;
    int w, h;
    float speed;
};

struct mountain {
    float x;
    int height;
    int width;
    uint32_t color;
};

struct tree {
    float x, y;
    int size;
};

struct player {
    float x, y;
    float vx, vy;
    bool on_ground;
    bool facing_right;
    int anim_frame;
    int anim_timer;
};

struct game_state {
    struct player player;
    struct platform platforms[MAX_PLATFORMS];
    struct coin coins[MAX_COINS];
    struct particle particles[MAX_PARTICLES];
    struct cloud clouds[MAX_CLOUDS];
    struct mountain mountains[MAX_MOUNTAINS];
    struct tree trees[MAX_TREES];
    float camera_x;
    float world_right_edge;
    int score;
    int lives;
    bool running;
    bool paused;
    bool game_over;
    int frame_counter;
};

/* ============================================================
 * Global State
 * ============================================================ */

static volatile bool g_quit = false;
static int input_fds[8];
static int input_count = 0;
static bool key_left = false;
static bool key_right = false;
static bool key_jump = false;

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

    /* Create cursor plane buffer (player character, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned where player character starts) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        100 - CURSOR_W / 4,
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background + platforms + score/level HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Enemies, collectibles, particles/effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player character (player-controlled)\n",
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

static inline void put_pixel(uint32_t *fb, uint32_t stride, uint32_t w, uint32_t h, int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h) fb[y * stride + x] = color;
}

static inline uint32_t lerp_color(uint32_t c1, uint32_t c2, float t) {
    if (t <= 0) return c1; if (t >= 1) return c2;
    uint32_t r = (uint32_t)(((c1>>16)&0xFF)*(1-t) + ((c2>>16)&0xFF)*t);
    uint32_t g = (uint32_t)(((c1>>8)&0xFF)*(1-t) + ((c2>>8)&0xFF)*t);
    uint32_t b = (uint32_t)((c1&0xFF)*(1-t) + (c2&0xFF)*t);
    return 0xFF000000 | (r<<16) | (g<<8) | b;
}

static inline uint32_t blend_alpha(uint32_t dst, uint32_t src) {
    uint32_t sa = (src >> 24) & 0xFF;
    if (sa == 0xFF) return src; if (sa == 0) return dst;
    uint32_t da = 255 - sa;
    uint32_t r = (((src>>16)&0xFF)*sa + ((dst>>16)&0xFF)*da) / 255;
    uint32_t g = (((src>>8)&0xFF)*sa + ((dst>>8)&0xFF)*da) / 255;
    uint32_t b = ((src&0xFF)*sa + (dst&0xFF)*da) / 255;
    return 0xFF000000 | (r<<16) | (g<<8) | b;
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

static void draw_rect_blend(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                              int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = (x+w) > (int)sw ? (int)sw : (x+w);
    int y1 = (y+h) > (int)sh ? (int)sh : (y+h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = blend_alpha(fb[j * stride + i], color);
}

static void draw_rect_gradient_v(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                                   int x, int y, int w, int h, uint32_t ct, uint32_t cb) {
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = (x+w) > (int)sw ? (int)sw : (x+w);
    int y1 = (y+h) > (int)sh ? (int)sh : (y+h);
    for (int j = y0; j < y1; j++) {
        float t = (h > 0) ? (float)(j - y) / (float)h : 0;
        uint32_t c = lerp_color(ct, cb, t);
        for (int i = x0; i < x1; i++) fb[j * stride + i] = c;
    }
}

static void draw_circle(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                          int cx, int cy, int r, uint32_t color) {
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r2)
                put_pixel(fb, stride, sw, sh, cx+dx, cy+dy, color);
}

static void draw_triangle(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                            int x0, int y0, int x1, int y1, int x2, int y2, uint32_t color) {
    /* Simple scanline triangle fill */
    int min_y = y0 < y1 ? (y0 < y2 ? y0 : y2) : (y1 < y2 ? y1 : y2);
    int max_y = y0 > y1 ? (y0 > y2 ? y0 : y2) : (y1 > y2 ? y1 : y2);
    if (min_y < 0) min_y = 0;
    if (max_y >= (int)sh) max_y = (int)sh - 1;

    for (int y = min_y; y <= max_y; y++) {
        int min_x = (int)sw, max_x = 0;
        /* Check each edge */
        int edges[3][4] = {{x0,y0,x1,y1},{x1,y1,x2,y2},{x2,y2,x0,y0}};
        for (int e = 0; e < 3; e++) {
            int ey0 = edges[e][1], ey1 = edges[e][3];
            int ex0 = edges[e][0], ex1 = edges[e][2];
            if ((y >= ey0 && y < ey1) || (y >= ey1 && y < ey0)) {
                int ix = ex0 + (y - ey0) * (ex1 - ex0) / (ey1 - ey0);
                if (ix < min_x) min_x = ix;
                if (ix > max_x) max_x = ix;
            }
        }
        if (min_x < 0) min_x = 0;
        if (max_x >= (int)sw) max_x = (int)sw - 1;
        for (int x = min_x; x <= max_x; x++)
            put_pixel(fb, stride, sw, sh, x, y, color);
    }
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
}

/* Font for digits */
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
                        int x, int y, int d, int s, uint32_t c) {
    if (d < 0 || d > 9) return;
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (font_5x7[d][row] & (0x10 >> col))
                draw_rect(fb, stride, sw, sh, x+col*s, y+row*s, s, s, c);
}

static void draw_number(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int x, int y, int num, int s, uint32_t c) {
    char buf[16]; snprintf(buf, sizeof(buf), "%d", num);
    int off = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride, sw, sh, x+off, y, buf[i]-'0', s, c);
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
                case KEY_UP:
                case KEY_SPACE: key_jump = pressed; break;
                case KEY_P: if (ev.value == 1) game->paused = !game->paused; break;
                case KEY_ESC: case KEY_Q: if (ev.value == 1) game->running = false; break;
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

static void spawn_particle(struct game_state *game, float x, float y, uint32_t color) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) {
            game->particles[i] = (struct particle){
                .x = x, .y = y,
                .vx = (randf() - 0.5f) * 4.0f,
                .vy = -randf() * 4.0f,
                .life = 0.5f + randf() * 0.5f,
                .color = color,
                .active = true
            };
            return;
        }
    }
}

static void generate_world(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    srand((unsigned)time(NULL));

    /* Ground platform (very wide) */
    game->platforms[0] = (struct platform){
        .x = -100, .y = (float)screen_h - 30,
        .w = (int)(screen_w * 10), .h = 30,
        .color = COLOR_GROUND, .active = true
    };

    /* Generate platforms */
    float px = 50;
    for (int i = 1; i < MAX_PLATFORMS; i++) {
        int pw = PLATFORM_MIN_W + rand() % (PLATFORM_MAX_W - PLATFORM_MIN_W);
        float py = screen_h - 80 - randf() * (screen_h * 0.5f);
        game->platforms[i] = (struct platform){
            .x = px, .y = py, .w = pw, .h = PLATFORM_H,
            .color = plat_colors[rand() % 6], .active = true
        };
        px += pw + 40 + randf() * 80;
    }
    game->world_right_edge = px + 200;

    /* Coins on platforms */
    for (int i = 0; i < MAX_COINS; i++) {
        int pi = 1 + rand() % (MAX_PLATFORMS - 1);
        if (game->platforms[pi].active) {
            game->coins[i] = (struct coin){
                .x = game->platforms[pi].x + randf() * game->platforms[pi].w,
                .y = game->platforms[pi].y - 25,
                .anim_offset = randf() * 6.28f,
                .collected = false, .active = true
            };
        }
    }

    /* Clouds */
    for (int i = 0; i < MAX_CLOUDS; i++) {
        game->clouds[i] = (struct cloud){
            .x = randf() * screen_w * 3,
            .y = 20 + randf() * (screen_h * 0.3f),
            .w = 40 + (int)(randf() * 60),
            .h = 20 + (int)(randf() * 20),
            .speed = 0.2f + randf() * 0.5f
        };
    }

    /* Mountains (background) */
    float mx = 0;
    for (int i = 0; i < MAX_MOUNTAINS; i++) {
        game->mountains[i] = (struct mountain){
            .x = mx,
            .height = 60 + (int)(randf() * 100),
            .width = 100 + (int)(randf() * 150),
            .color = (i % 2 == 0) ? COLOR_MOUNTAIN_FAR : COLOR_MOUNTAIN_NEAR
        };
        mx += 80 + randf() * 120;
    }

    /* Trees */
    float tx = 30;
    for (int i = 0; i < MAX_TREES; i++) {
        game->trees[i] = (struct tree){
            .x = tx,
            .y = (float)screen_h - 30,
            .size = 15 + (int)(randf() * 20)
        };
        tx += 50 + randf() * 100;
    }
}

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));
    game->player.x = 100;
    game->player.y = screen_h - 70;
    game->player.facing_right = true;
    game->lives = 3;
    game->running = true;
    generate_world(game, screen_w, screen_h);
}

static void game_update(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    if (game->paused || game->game_over) return;

    struct player *p = &game->player;
    game->frame_counter++;

    /* Horizontal movement */
    p->vx = 0;
    if (key_left) { p->vx = -MOVE_SPEED; p->facing_right = false; }
    if (key_right) { p->vx = MOVE_SPEED; p->facing_right = true; }

    /* Jump */
    if (key_jump && p->on_ground) {
        p->vy = JUMP_FORCE;
        p->on_ground = false;
        for (int i = 0; i < 5; i++)
            spawn_particle(game, p->x + PLAYER_W/2, p->y + PLAYER_H, 0xFFDDDDDD);
    }

    /* Gravity */
    p->vy += GRAVITY;
    if (p->vy > MAX_FALL_SPEED) p->vy = MAX_FALL_SPEED;

    /* Move */
    p->x += p->vx;
    p->y += p->vy;

    /* Platform collision */
    p->on_ground = false;
    for (int i = 0; i < MAX_PLATFORMS; i++) {
        if (!game->platforms[i].active) continue;
        struct platform *pl = &game->platforms[i];
        /* Check if player is landing on top */
        if (p->vy >= 0 &&
            p->x + PLAYER_W > pl->x && p->x < pl->x + pl->w &&
            p->y + PLAYER_H >= pl->y && p->y + PLAYER_H <= pl->y + pl->h + p->vy + 2) {
            p->y = pl->y - PLAYER_H;
            p->vy = 0;
            p->on_ground = true;
            /* Landing particles */
            if (fabsf(p->vy) > 3.0f) {
                for (int j = 0; j < 3; j++)
                    spawn_particle(game, p->x + PLAYER_W/2, p->y + PLAYER_H, 0xFF886644);
            }
        }
    }

    /* Fall death */
    if (p->y > (float)screen_h + 50) {
        game->lives--;
        if (game->lives <= 0) {
            game->game_over = true;
        } else {
            p->x = 100; p->y = screen_h - 70;
            p->vx = 0; p->vy = 0;
            game->camera_x = 0;
        }
    }

    /* Coin collection */
    for (int i = 0; i < MAX_COINS; i++) {
        if (!game->coins[i].active || game->coins[i].collected) continue;
        float dx = fabsf((p->x + PLAYER_W/2) - game->coins[i].x);
        float dy = fabsf((p->y + PLAYER_H/2) - game->coins[i].y);
        if (dx < 18 && dy < 18) {
            game->coins[i].collected = true;
            game->score += 100;
            for (int j = 0; j < 6; j++)
                spawn_particle(game, game->coins[i].x, game->coins[i].y, COLOR_COIN_SHINE);
        }
    }

    /* Animation */
    p->anim_timer++;
    if (p->anim_timer > 8) {
        p->anim_timer = 0;
        p->anim_frame = (p->anim_frame + 1) % 4;
    }

    /* Camera follow */
    float target_cam = p->x - screen_w / 3.0f;
    if (target_cam < 0) target_cam = 0;
    game->camera_x += (target_cam - game->camera_x) * 0.08f;

    /* Update particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].vx;
        game->particles[i].y += game->particles[i].vy;
        game->particles[i].vy += 0.2f;
        game->particles[i].life -= 0.03f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }

    /* Clouds drift */
    for (int i = 0; i < MAX_CLOUDS; i++)
        game->clouds[i].x += game->clouds[i].speed;

    (void)screen_w;
}

/* ============================================================
 * Rendering - Primary Plane (Background + Platforms + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;
    float cam = game->camera_x;

    /* Sky gradient */
    draw_rect_gradient_v(fb, stride, w, h, 0, 0, (int)w, (int)h, COLOR_SKY_TOP, COLOR_SKY_BOT);

    /* Mountains (parallax: 0.2x camera speed) */
    for (int i = 0; i < MAX_MOUNTAINS; i++) {
        int mx = (int)(game->mountains[i].x - cam * 0.2f);
        int mw = game->mountains[i].width;
        int mh = game->mountains[i].height;
        int my = (int)h - 30 - mh;
        draw_triangle(fb, stride, w, h,
                      mx, my + mh, mx + mw/2, my, mx + mw, my + mh,
                      game->mountains[i].color);
    }

    /* Clouds (parallax: 0.3x) */
    for (int i = 0; i < MAX_CLOUDS; i++) {
        int cx = (int)(game->clouds[i].x - cam * 0.3f) % ((int)w + 200) - 100;
        int cy = (int)game->clouds[i].y;
        int cw = game->clouds[i].w;
        int ch = game->clouds[i].h;
        /* Multi-ellipse cloud */
        draw_circle(fb, stride, w, h, cx, cy, ch/2, COLOR_CLOUD);
        draw_circle(fb, stride, w, h, cx + cw/3, cy - ch/4, ch/2 + 2, COLOR_CLOUD);
        draw_circle(fb, stride, w, h, cx + cw*2/3, cy, ch/2 - 1, COLOR_CLOUD);
    }

    /* Trees (parallax: 0.7x) */
    for (int i = 0; i < MAX_TREES; i++) {
        int tx = (int)(game->trees[i].x - cam * 0.7f);
        int ty = (int)game->trees[i].y;
        int sz = game->trees[i].size;
        /* Trunk */
        draw_rect(fb, stride, w, h, tx - 3, ty - sz, 6, sz, COLOR_TREE_TRUNK);
        /* Leaves (layered circles) */
        draw_circle(fb, stride, w, h, tx, ty - sz - sz/2, sz/2 + 4, 0xFF115522);
        draw_circle(fb, stride, w, h, tx, ty - sz - sz/2, sz/2, COLOR_TREE_LEAVES);
        draw_circle(fb, stride, w, h, tx - sz/3, ty - sz, sz/3, COLOR_TREE_LEAVES);
        draw_circle(fb, stride, w, h, tx + sz/3, ty - sz, sz/3, COLOR_TREE_LEAVES);
    }

    /* Platforms */
    for (int i = 0; i < MAX_PLATFORMS; i++) {
        if (!game->platforms[i].active) continue;
        struct platform *pl = &game->platforms[i];
        int px = (int)(pl->x - cam);
        int py = (int)pl->y;
        if (px + pl->w < 0 || px > (int)w) continue;

        if (i == 0) {
            /* Ground: gradient */
            draw_rect_gradient_v(fb, stride, w, h, px, py, pl->w, pl->h,
                                 COLOR_GROUND, COLOR_GROUND_DARK);
            /* Grass tufts on top */
            for (int g = px; g < px + pl->w && g < (int)w; g += 8) {
                draw_rect(fb, stride, w, h, g, py - 3, 3, 3, 0xFF44BB44);
            }
        } else {
            /* Platform with highlight and shadow */
            draw_rect(fb, stride, w, h, px + 2, py + 2, pl->w, pl->h, 0x66000000);
            draw_rect_gradient_v(fb, stride, w, h, px, py, pl->w, pl->h,
                                 pl->color, pl->color & 0xFF808080);
            /* Top edge highlight */
            draw_rect(fb, stride, w, h, px, py, pl->w, 3, pl->color | 0x00303030);
        }
    }

    /* HUD */
    draw_rect_blend(fb, stride, w, h, 0, 0, (int)w, 30, 0xAA000000);
    draw_number(fb, stride, w, h, 10, 6, game->score, 3, COLOR_COIN);
    /* Lives as hearts (small red squares) */
    for (int i = 0; i < game->lives; i++)
        draw_rect(fb, stride, w, h, (int)w - 25 - i * 18, 8, 12, 12, 0xFFFF3333);

    /* Game over */
    if (game->game_over) {
        draw_rect_blend(fb, stride, w, h, 0, (int)h/2 - 25, (int)w, 50, 0xCC000000);
        draw_number(fb, stride, w, h, (int)w/2 - 30, (int)h/2 - 8, game->score, 3, 0xFFFFFF00);
    }

    /* Pause */
    if (game->paused) {
        draw_rect_blend(fb, stride, w, h, (int)w/2 - 30, (int)h/2 - 12, 60, 24, 0xCC000000);
        draw_rect(fb, stride, w, h, (int)w/2 - 10, (int)h/2 - 6, 6, 12, 0xFFFFFFFF);
        draw_rect(fb, stride, w, h, (int)w/2 + 4, (int)h/2 - 6, 6, 12, 0xFFFFFFFF);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Coins + Particles/Effects)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;
    float cam = game->camera_x;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Coins */
    float anim_t = game->frame_counter * 0.1f;
    for (int i = 0; i < MAX_COINS; i++) {
        if (!game->coins[i].active || game->coins[i].collected) continue;
        int cx = (int)(game->coins[i].x - cam);
        float bob = sinf(anim_t + game->coins[i].anim_offset) * 3.0f;
        int cy = (int)(game->coins[i].y + bob);
        if (cx < -10 || cx > (int)w + 10) continue;
        /* Coin body */
        draw_circle(fb, stride, w, h, cx, cy, 7, COLOR_COIN);
        /* Shine */
        draw_circle(fb, stride, w, h, cx - 2, cy - 2, 2, COLOR_COIN_SHINE);
    }

    /* Particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        int px = (int)(game->particles[i].x - cam);
        int py = (int)game->particles[i].y;
        int sz = (int)(game->particles[i].life * 4) + 1;
        uint32_t alpha = (uint32_t)(game->particles[i].life * 255);
        uint32_t pc = (game->particles[i].color & 0x00FFFFFF) | (alpha << 24);
        draw_rect(fb, stride, w, h, px, py, sz, sz, pc);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Player Character)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf, struct game_state *game) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    if (game->game_over) return;

    struct player *p = &game->player;

    /* Draw player character centered in cursor buffer */
    int ox = ((int)w - PLAYER_W) / 2;
    int oy = ((int)h - PLAYER_H) / 2;

    /* Shadow */
    draw_rect_blend(fb, stride, w, h, ox + 2, oy + PLAYER_H - 4, PLAYER_W - 2, 4, 0x66000000);

    /* Body */
    draw_rect(fb, stride, w, h, ox + 4, oy + 10, PLAYER_W - 8, PLAYER_H - 16, COLOR_PLAYER_BODY);

    /* Head */
    draw_circle(fb, stride, w, h, ox + PLAYER_W/2, oy + 6, 6, COLOR_PLAYER_HEAD);

    /* Eyes */
    int eye_offset = p->facing_right ? 2 : -2;
    draw_rect(fb, stride, w, h, ox + PLAYER_W/2 + eye_offset - 1, oy + 4, 2, 2, 0xFF000000);

    /* Legs (animated) */
    int leg_offset = 0;
    if (fabsf(p->vx) > 0.1f) leg_offset = (p->anim_frame % 2) * 4 - 2;
    draw_rect(fb, stride, w, h, ox + 5, oy + PLAYER_H - 8 + leg_offset, 4, 8, COLOR_PLAYER_FEET);
    draw_rect(fb, stride, w, h, ox + PLAYER_W - 9, oy + PLAYER_H - 8 - leg_offset, 4, 8, COLOR_PLAYER_FEET);
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

    printf("=== DRM Platformer (Overlay + Cursor Planes) ===\n");
    printf("Controls: Left/Right=Move, Space/Up=Jump, P=Pause, ESC/Q=Quit\n\n");

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

        /* Render primary plane (background + platforms + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (coins + particles/effects) into back buffer */
        render_overlay(&dev, &game);

        /* Render cursor plane (player character) */
        render_cursor(&dev.cursor_buf, &game);

        /* Compute cursor plane position (player character movement) */
        int cursor_x = (int)(game.player.x - game.camera_x) - (CURSOR_W - PLAYER_W) / 2;
        int cursor_y = (int)game.player.y - (CURSOR_H - PLAYER_H) / 2;

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
        uint64_t elapsed = get_time_ns() - t0;
        if (elapsed < FRAME_TIME_NS) sleep_ns(FRAME_TIME_NS - elapsed);

        frame_count++;
        if (get_time_ns() - fps_timer >= 5000000000ULL) {
            printf("[GAME] FPS: %.1f | Score: %d\n", frame_count / 5.0f, game.score);
            frame_count = 0; fps_timer = get_time_ns();
        }
    }

    printf("[GAME] Final Score: %d\n", game.score);

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);
    return 0;
}
