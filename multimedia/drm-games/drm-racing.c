/*
 * drm-racing.c - Top-Down Racing Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Road/track background + speed/score/lap HUD
 *   - Overlay plane:  Other cars/obstacles, road markings, effects
 *   - Cursor plane:   Player car (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-racing drm-racing.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-racing drm-racing.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-racing [/dev/dri/cardN]
 *
 * Controls:
 *   - Arrow keys or touchscreen: steer left/right, accelerate/brake
 *   - P: pause
 *   - SPACE: restart after game over
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

/* Cursor plane dimensions (player car) */
#define CURSOR_W            64
#define CURSOR_H            128

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define ROAD_WIDTH_RATIO    0.5f   /* Road takes 50% of screen width */
#define LANE_COUNT          3
#define CAR_WIDTH           30
#define CAR_HEIGHT          50
#define PLAYER_SPEED        6
#define MAX_ENEMIES         8
#define SCROLL_SPEED_INIT   4.0f
#define SCROLL_SPEED_MAX    12.0f
#define SPEED_INCREMENT     0.002f

#define PARTICLE_COUNT      32
#define SPEED_LINE_COUNT    16

#define COLOR_TRANSPARENT   0x00000000
#define COLOR_SKY_TOP       0xFF1A0533
#define COLOR_SKY_BOT       0xFF2D1B69
#define COLOR_ROAD          0xFF333333
#define COLOR_ROAD_EDGE     0xFFFFFFFF
#define COLOR_LANE_MARK     0xFFCCCC00
#define COLOR_GRASS_LIGHT   0xFF228B22
#define COLOR_GRASS_DARK    0xFF1A6B1A
#define COLOR_PLAYER        0xFF0088FF
#define COLOR_PLAYER_LIGHT  0xFF44AAFF
#define COLOR_PLAYER_DARK   0xFF004488
#define COLOR_SHADOW        0x88000000
#define COLOR_HUD_BG        0xAA000000
#define COLOR_HUD_TEXT      0xFFFFFFFF
#define COLOR_SCORE         0xFF00FF88

/* Enemy car colors */
static const uint32_t enemy_colors[] = {
    0xFFFF2222, 0xFF22FF22, 0xFFFFAA00,
    0xFFFF44FF, 0xFF00FFFF, 0xFFFF8800,
    0xFFAA00FF, 0xFFFFFF00,
};
#define NUM_ENEMY_COLORS (sizeof(enemy_colors) / sizeof(enemy_colors[0]))

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

struct particle {
    float x, y;
    float vx, vy;
    float life;
    uint32_t color;
    bool active;
};

struct speed_line {
    float x, y;
    float length;
    float speed;
    bool active;
};

struct enemy_car {
    float x, y;
    float speed;
    uint32_t color;
    bool active;
};

struct game_state {
    float player_x;
    float player_y;
    float scroll_offset;
    float scroll_speed;
    float distance;
    int score;
    int lives;
    bool running;
    bool paused;
    bool game_over;
    struct enemy_car enemies[MAX_ENEMIES];
    struct particle particles[PARTICLE_COUNT];
    struct speed_line speed_lines[SPEED_LINE_COUNT];
    int road_x;
    int road_w;
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
    struct timespec ts = { .tv_sec = ns / 1000000000ULL, .tv_nsec = ns % 1000000000ULL };
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

    /* Create cursor plane buffer (player car, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned where player car is) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        (dev->width / 2) - (CURSOR_W / 2),
                        (dev->height * 4 / 5) - (CURSOR_H / 2),
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Road/track background + speed/score/lap HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Other cars, obstacles, road markings, effects\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Player car (player-controlled)\n",
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
 * Drawing Primitives (Enhanced)
 * ============================================================ */

static inline void put_pixel(uint32_t *fb, uint32_t stride,
                              uint32_t w, uint32_t h,
                              int x, int y, uint32_t color) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h)
        fb[y * stride + x] = color;
}

/* Alpha blend: src over dst */
static inline uint32_t blend_color(uint32_t dst, uint32_t src) {
    uint32_t sa = (src >> 24) & 0xFF;
    if (sa == 0xFF) return src;
    if (sa == 0x00) return dst;
    uint32_t da = 255 - sa;
    uint32_t r = (((src >> 16) & 0xFF) * sa + ((dst >> 16) & 0xFF) * da) / 255;
    uint32_t g = (((src >> 8) & 0xFF) * sa + ((dst >> 8) & 0xFF) * da) / 255;
    uint32_t b = ((src & 0xFF) * sa + (dst & 0xFF) * da) / 255;
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

/* Lerp between two colors */
static inline uint32_t lerp_color(uint32_t c1, uint32_t c2, float t) {
    if (t <= 0.0f) return c1;
    if (t >= 1.0f) return c2;
    uint32_t r = (uint32_t)(((c1 >> 16) & 0xFF) * (1 - t) + ((c2 >> 16) & 0xFF) * t);
    uint32_t g = (uint32_t)(((c1 >> 8) & 0xFF) * (1 - t) + ((c2 >> 8) & 0xFF) * t);
    uint32_t b = (uint32_t)((c1 & 0xFF) * (1 - t) + (c2 & 0xFF) * t);
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

static void draw_rect(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                       int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = color;
}

static void draw_rect_blend(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                             int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = blend_color(fb[j * stride + i], color);
}

/* Gradient-filled rectangle (vertical gradient) */
static void draw_rect_gradient_v(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                                  int x, int y, int w, int h,
                                  uint32_t color_top, uint32_t color_bot) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)scr_w ? (int)scr_w : (x + w);
    int y1 = (y + h) > (int)scr_h ? (int)scr_h : (y + h);
    for (int j = y0; j < y1; j++) {
        float t = (float)(j - y) / (float)h;
        uint32_t c = lerp_color(color_top, color_bot, t);
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = c;
    }
}

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
}

/* Draw a car shape (stylized rectangle with details) */
static void draw_car(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                     int cx, int cy, int w, int h, uint32_t body_color, bool is_player) {
    uint32_t dark = ((body_color >> 1) & 0x7F7F7F) | 0xFF000000;
    uint32_t light = body_color | 0x00404040;
    if (light < body_color) light = 0xFFFFFFFF; /* overflow protection */

    /* Shadow */
    draw_rect_blend(fb, stride, scr_w, scr_h, cx - w/2 + 3, cy - h/2 + 3, w, h, COLOR_SHADOW);

    /* Main body */
    draw_rect(fb, stride, scr_w, scr_h, cx - w/2, cy - h/2, w, h, body_color);

    /* Top gradient highlight */
    draw_rect(fb, stride, scr_w, scr_h, cx - w/2, cy - h/2, w, h/4, light);

    /* Bottom darker area */
    draw_rect(fb, stride, scr_w, scr_h, cx - w/2, cy + h/4, w, h/4, dark);

    /* Windshield */
    int ws_w = w * 2 / 3;
    int ws_h = h / 5;
    draw_rect(fb, stride, scr_w, scr_h, cx - ws_w/2, cy - h/4, ws_w, ws_h, 0xFF88CCFF);

    /* Rear window */
    draw_rect(fb, stride, scr_w, scr_h, cx - ws_w/2, cy + h/6, ws_w, ws_h - 2, 0xFF6699CC);

    /* Wheels */
    int wheel_w = 4, wheel_h = 10;
    draw_rect(fb, stride, scr_w, scr_h, cx - w/2 - 2, cy - h/3, wheel_w, wheel_h, 0xFF111111);
    draw_rect(fb, stride, scr_w, scr_h, cx + w/2 - 2, cy - h/3, wheel_w, wheel_h, 0xFF111111);
    draw_rect(fb, stride, scr_w, scr_h, cx - w/2 - 2, cy + h/5, wheel_w, wheel_h, 0xFF111111);
    draw_rect(fb, stride, scr_w, scr_h, cx + w/2 - 2, cy + h/5, wheel_w, wheel_h, 0xFF111111);

    if (is_player) {
        /* Tail lights */
        draw_rect(fb, stride, scr_w, scr_h, cx - w/2 + 2, cy + h/2 - 4, 6, 3, 0xFFFF0000);
        draw_rect(fb, stride, scr_w, scr_h, cx + w/2 - 8, cy + h/2 - 4, 6, 3, 0xFFFF0000);
        /* Headlights */
        draw_rect(fb, stride, scr_w, scr_h, cx - w/2 + 2, cy - h/2 + 1, 6, 3, 0xFFFFFF88);
        draw_rect(fb, stride, scr_w, scr_h, cx + w/2 - 8, cy - h/2 + 1, 6, 3, 0xFFFFFF88);
    }
}

/* Simple 5x7 font for digits */
static const uint8_t font_5x7[][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, /* 0 */
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* 1 */
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, /* 2 */
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}, /* 3 */
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, /* 4 */
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, /* 5 */
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, /* 6 */
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* 7 */
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, /* 8 */
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, /* 9 */
};

static void draw_digit(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                        int x, int y, int digit, int scale, uint32_t color) {
    if (digit < 0 || digit > 9) return;
    const uint8_t *glyph = font_5x7[digit];
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (glyph[row] & (0x10 >> col))
                draw_rect(fb, stride, scr_w, scr_h,
                          x + col * scale, y + row * scale, scale, scale, color);
}

static void draw_number(uint32_t *fb, uint32_t stride, uint32_t scr_w, uint32_t scr_h,
                         int x, int y, int number, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", number);
    int offset = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride, scr_w, scr_h, x + offset, y, buf[i] - '0', scale, color);
        offset += 6 * scale;
    }
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64];
    char name[256];
    input_count = 0;
    for (int i = 0; i < 8 && input_count < 8; i++) {
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
    for (int i = 0; i < input_count; i++) close(input_fds[i]);
    input_count = 0;
}

static void input_process(struct game_state *game, uint32_t screen_w) {
    struct input_event ev;
    (void)screen_w;

    for (int i = 0; i < input_count; i++) {
        while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
            if (ev.type == EV_ABS) {
                if (ev.code == ABS_X || ev.code == ABS_MT_POSITION_X) {
                    struct input_absinfo abs_info;
                    int max_val = 4095;
                    if (ioctl(input_fds[i], EVIOCGABS(ev.code), &abs_info) == 0)
                        max_val = abs_info.maximum > 0 ? abs_info.maximum : 4095;
                    game->player_x = (float)(ev.value * (int)screen_w / max_val);
                }
            }
            if (ev.type == EV_KEY) {
                bool pressed = (ev.value == 1 || ev.value == 2);
                switch (ev.code) {
                case KEY_LEFT:  key_left = pressed; break;
                case KEY_RIGHT: key_right = pressed; break;
                case KEY_UP:    key_up = pressed; break;
                case KEY_DOWN:  key_down = pressed; break;
                case KEY_P:
                    if (ev.value == 1) game->paused = !game->paused;
                    break;
                case KEY_SPACE:
                    if (ev.value == 1 && game->game_over) {
                        game->game_over = false;
                        game->score = 0;
                        game->lives = 3;
                        game->scroll_speed = SCROLL_SPEED_INIT;
                        game->distance = 0;
                        for (int e = 0; e < MAX_ENEMIES; e++)
                            game->enemies[e].active = false;
                    }
                    break;
                case KEY_ESC:
                case KEY_Q:
                    if (ev.value == 1) g_quit = true;
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

static float randf(void) {
    return (float)rand() / (float)RAND_MAX;
}

static void spawn_enemy(struct game_state *game, uint32_t screen_h) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) {
            int lane = rand() % LANE_COUNT;
            int lane_w = game->road_w / LANE_COUNT;
            game->enemies[i].x = game->road_x + lane * lane_w + lane_w / 2;
            game->enemies[i].y = -(float)CAR_HEIGHT;
            game->enemies[i].speed = game->scroll_speed * (0.5f + randf() * 0.5f);
            game->enemies[i].color = enemy_colors[rand() % NUM_ENEMY_COLORS];
            game->enemies[i].active = true;
            (void)screen_h;
            return;
        }
    }
}

static void update_particles(struct game_state *game) {
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].vx;
        game->particles[i].y += game->particles[i].vy;
        game->particles[i].life -= 0.02f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }
}

static void spawn_particle(struct game_state *game, float x, float y, uint32_t color) {
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        if (!game->particles[i].active) {
            game->particles[i].x = x;
            game->particles[i].y = y;
            game->particles[i].vx = (randf() - 0.5f) * 6.0f;
            game->particles[i].vy = (randf() - 0.5f) * 6.0f;
            game->particles[i].life = 1.0f;
            game->particles[i].color = color;
            game->particles[i].active = true;
            return;
        }
    }
}

static void spawn_explosion(struct game_state *game, float x, float y) {
    for (int i = 0; i < 12; i++) {
        uint32_t colors[] = {0xFFFF4400, 0xFFFFAA00, 0xFFFFFF00, 0xFFFF0000};
        spawn_particle(game, x, y, colors[i % 4]);
    }
}

static void update_speed_lines(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    for (int i = 0; i < SPEED_LINE_COUNT; i++) {
        if (!game->speed_lines[i].active) {
            if (randf() < 0.1f && game->scroll_speed > 6.0f) {
                game->speed_lines[i].x = game->road_x + randf() * game->road_w;
                game->speed_lines[i].y = 0;
                game->speed_lines[i].length = 10 + randf() * 30;
                game->speed_lines[i].speed = game->scroll_speed * 2.0f;
                game->speed_lines[i].active = true;
            }
        } else {
            game->speed_lines[i].y += game->speed_lines[i].speed;
            if (game->speed_lines[i].y > (float)screen_h)
                game->speed_lines[i].active = false;
        }
    }
    (void)screen_w;
}

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));
    game->road_w = (int)(screen_w * ROAD_WIDTH_RATIO);
    game->road_x = ((int)screen_w - game->road_w) / 2;
    game->player_x = screen_w / 2.0f;
    game->player_y = screen_h * 0.8f;
    game->scroll_speed = SCROLL_SPEED_INIT;
    game->lives = 3;
    game->running = true;
    srand((unsigned)time(NULL));
}

static void game_update(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    if (game->paused || game->game_over) return;

    /* Player movement */
    if (key_left) game->player_x -= PLAYER_SPEED;
    if (key_right) game->player_x += PLAYER_SPEED;
    if (key_up) game->scroll_speed += 0.05f;
    if (key_down) game->scroll_speed -= 0.05f;

    if (game->scroll_speed < 2.0f) game->scroll_speed = 2.0f;
    if (game->scroll_speed > SCROLL_SPEED_MAX) game->scroll_speed = SCROLL_SPEED_MAX;

    /* Clamp player to road */
    float min_x = game->road_x + CAR_WIDTH / 2.0f + 5;
    float max_x = game->road_x + game->road_w - CAR_WIDTH / 2.0f - 5;
    if (game->player_x < min_x) game->player_x = min_x;
    if (game->player_x > max_x) game->player_x = max_x;

    /* Scroll */
    game->scroll_offset += game->scroll_speed;
    game->distance += game->scroll_speed;
    game->scroll_speed += SPEED_INCREMENT;
    if (game->scroll_speed > SCROLL_SPEED_MAX) game->scroll_speed = SCROLL_SPEED_MAX;

    /* Score based on distance */
    game->score = (int)(game->distance / 10.0f);

    /* Spawn enemies */
    if (rand() % 60 < 2 + (int)(game->scroll_speed / 3.0f))
        spawn_enemy(game, screen_h);

    /* Update enemies */
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) continue;
        game->enemies[i].y += game->scroll_speed - game->enemies[i].speed;
        if (game->enemies[i].y > (float)screen_h + CAR_HEIGHT)
            game->enemies[i].active = false;

        /* Collision with player */
        float dx = fabsf(game->enemies[i].x - game->player_x);
        float dy = fabsf(game->enemies[i].y - game->player_y);
        if (dx < CAR_WIDTH && dy < CAR_HEIGHT * 0.8f) {
            spawn_explosion(game, game->player_x, game->player_y);
            game->enemies[i].active = false;
            game->lives--;
            if (game->lives <= 0) {
                game->game_over = true;
                printf("[GAME] Game Over! Score: %d\n", game->score);
            }
        }
    }

    update_particles(game);
    update_speed_lines(game, screen_w, screen_h);
}

/* ============================================================
 * Rendering - Primary Plane (Road/Track Background + HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Draw gradient background (grass) */
    for (uint32_t y = 0; y < h; y++) {
        float t = (float)y / (float)h;
        uint32_t grass = lerp_color(COLOR_GRASS_LIGHT, COLOR_GRASS_DARK, t);
        for (uint32_t x = 0; x < w; x++)
            fb[y * stride + x] = grass;
    }

    /* Grass stripes (scrolling) */
    int stripe_spacing = 40;
    int offset = (int)game->scroll_offset % stripe_spacing;
    for (uint32_t y = 0; y < h; y++) {
        if (((int)y + offset) % stripe_spacing < 3) {
            for (int x = 0; x < game->road_x - 10; x++)
                fb[y * stride + x] = lerp_color(fb[y * stride + x], 0xFF33AA33, 0.3f);
            for (int x = game->road_x + game->road_w + 10; x < (int)w; x++)
                fb[y * stride + x] = lerp_color(fb[y * stride + x], 0xFF33AA33, 0.3f);
        }
    }

    /* Road surface */
    draw_rect(fb, stride, w, h, game->road_x, 0, game->road_w, (int)h, COLOR_ROAD);

    /* Road edge lines (white) */
    draw_rect(fb, stride, w, h, game->road_x, 0, 3, (int)h, COLOR_ROAD_EDGE);
    draw_rect(fb, stride, w, h, game->road_x + game->road_w - 3, 0, 3, (int)h, COLOR_ROAD_EDGE);

    /* Road edge rumble strips */
    int rumble_h = 20;
    int rumble_offset = (int)game->scroll_offset % (rumble_h * 2);
    for (uint32_t y = 0; y < h; y++) {
        int ymod = ((int)y + rumble_offset) % (rumble_h * 2);
        uint32_t rc = ymod < rumble_h ? 0xFFFF0000 : 0xFFFFFFFF;
        for (int dx = 0; dx < 5; dx++) {
            put_pixel(fb, stride, w, h, game->road_x - 5 + dx, (int)y, rc);
            put_pixel(fb, stride, w, h, game->road_x + game->road_w + dx, (int)y, rc);
        }
    }

    /* HUD Background */
    draw_rect_blend(fb, stride, w, h, 0, 0, (int)w, 35, COLOR_HUD_BG);

    /* Score */
    draw_number(fb, stride, w, h, 10, 8, game->score, 3, COLOR_SCORE);

    /* Speed indicator */
    int speed_pct = (int)((game->scroll_speed / SCROLL_SPEED_MAX) * 100);
    draw_number(fb, stride, w, h, (int)w - 80, 8, speed_pct, 2, 0xFFFF8800);

    /* Lives */
    for (int i = 0; i < game->lives; i++) {
        draw_rect(fb, stride, w, h, (int)w / 2 - 30 + i * 18, 10, 12, 12, 0xFFFF4444);
    }

    /* Game Over overlay */
    if (game->game_over) {
        draw_rect_blend(fb, stride, w, h, 0, (int)h/2 - 30, (int)w, 60, 0xCC000000);
        /* Draw "GAME OVER" as simple blocks */
        draw_rect(fb, stride, w, h, (int)w/2 - 50, (int)h/2 - 10, 100, 4, 0xFFFF0000);
        draw_rect(fb, stride, w, h, (int)w/2 - 50, (int)h/2 + 6, 100, 4, 0xFFFF0000);
        draw_number(fb, stride, w, h, (int)w/2 - 30, (int)h/2 + 20, game->score, 2, COLOR_HUD_TEXT);
    }

    /* Pause overlay */
    if (game->paused) {
        draw_rect_blend(fb, stride, w, h, (int)w/2 - 40, (int)h/2 - 15, 80, 30, 0xCC000000);
        draw_rect(fb, stride, w, h, (int)w/2 - 12, (int)h/2 - 8, 8, 16, COLOR_HUD_TEXT);
        draw_rect(fb, stride, w, h, (int)w/2 + 4, (int)h/2 - 8, 8, 16, COLOR_HUD_TEXT);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Enemy Cars, Road Markings, Effects)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Lane markings (dashed) */
    int lane_w = game->road_w / LANE_COUNT;
    int dash_len = 30, gap_len = 20;
    int mark_offset = (int)game->scroll_offset % (dash_len + gap_len);
    for (int lane = 1; lane < LANE_COUNT; lane++) {
        int lx = game->road_x + lane * lane_w;
        for (uint32_t y = 0; y < h; y++) {
            int ymod = ((int)y + mark_offset) % (dash_len + gap_len);
            if (ymod < dash_len) {
                put_pixel(fb, stride, w, h, lx - 1, (int)y, COLOR_LANE_MARK);
                put_pixel(fb, stride, w, h, lx, (int)y, COLOR_LANE_MARK);
                put_pixel(fb, stride, w, h, lx + 1, (int)y, COLOR_LANE_MARK);
            }
        }
    }

    /* Speed lines */
    for (int i = 0; i < SPEED_LINE_COUNT; i++) {
        if (!game->speed_lines[i].active) continue;
        int lx = (int)game->speed_lines[i].x;
        int ly = (int)game->speed_lines[i].y;
        int len = (int)game->speed_lines[i].length;
        for (int d = 0; d < len; d++) {
            float alpha = 1.0f - (float)d / (float)len;
            uint32_t a = (uint32_t)(alpha * 128);
            uint32_t c = (a << 24) | 0x00FFFFFF;
            int py = ly + d;
            if (py >= 0 && py < (int)h && lx >= 0 && lx < (int)w)
                fb[py * stride + lx] = c;
        }
    }

    /* Enemy cars */
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!game->enemies[i].active) continue;
        draw_car(fb, stride, w, h,
                 (int)game->enemies[i].x, (int)game->enemies[i].y,
                 CAR_WIDTH, CAR_HEIGHT, game->enemies[i].color, false);
    }

    /* Particles */
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        if (!game->particles[i].active) continue;
        int px = (int)game->particles[i].x;
        int py = (int)game->particles[i].y;
        int sz = (int)(game->particles[i].life * 5);
        if (sz < 1) sz = 1;
        uint32_t pc = game->particles[i].color;
        uint32_t alpha = (uint32_t)(game->particles[i].life * 255);
        pc = (pc & 0x00FFFFFF) | (alpha << 24);
        draw_rect(fb, stride, w, h, px - sz/2, py - sz/2, sz, sz, pc);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Player Car)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    /* Draw player car centered in cursor buffer */
    int cx = (int)w / 2;
    int cy = (int)h / 2;

    draw_car(fb, stride, w, h, cx, cy, CAR_WIDTH, CAR_HEIGHT, COLOR_PLAYER, true);
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
    printf("  DRM Racing Game - Microchip Demo\n");
    printf("  Built with libdrm (KMS/DRM, 3 Planes)\n");
    printf("===========================================\n\n");

    /* Initialize DRM with all 3 planes */
    if (drm_init(&dev, card_path) < 0) {
        fprintf(stderr, "Failed to initialize DRM\n");
        return 1;
    }

    /* Initialize input */
    input_init();

    /* Initialize game state */
    game_init(&game, dev.width, dev.height);

    /* Draw the player car sprite once (cursor plane content is static, just moves) */
    render_cursor(&dev.cursor_buf);

    printf("[GAME] Controls:\n");
    printf("  Left/Right : Steer\n");
    printf("  Up/Down    : Accelerate/Brake\n");
    printf("  P          : Pause\n");
    printf("  SPACE      : Restart (after game over)\n");
    printf("  ESC/Q      : Quit\n\n");
    printf("[GAME] Starting main loop...\n");

    /* Main game loop */
    uint64_t frame_start, frame_end, frame_elapsed;
    uint64_t fps_timer = get_time_ns();
    int frame_count = 0;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        /* Poll input */
        input_process(&game, dev.width);

        /* Update game state */
        game_update(&game, dev.width, dev.height);

        /* Render primary plane (road/track background + HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (enemy cars, road markings, effects) into back buffer */
        render_overlay(&dev, &game);

        /* Compute cursor plane position (player car movement) */
        int cursor_x = (int)game.player_x - (CURSOR_W / 2);
        int cursor_y = (int)game.player_y - (CURSOR_H / 2);

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

        /* FPS counter (print every 5 seconds) */
        frame_count++;
        if (get_time_ns() - fps_timer >= 5000000000ULL) {
            printf("[GAME] FPS: %.1f | Score: %d | Speed: %.0f%%\n",
                   (float)frame_count / 5.0f, game.score,
                   game.scroll_speed / SCROLL_SPEED_MAX * 100);
            frame_count = 0;
            fps_timer = get_time_ns();
        }
    }

    printf("\n[GAME] Final Score: %d\n", game.score);
    printf("[GAME] Shutting down...\n");

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);

    return 0;
}
