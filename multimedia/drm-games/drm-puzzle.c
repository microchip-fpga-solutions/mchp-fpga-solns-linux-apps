/*
 * drm-puzzle.c - Match-3 Puzzle Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background gradient + puzzle grid/board + score/moves HUD
 *   - Overlay plane:  Moving/animating puzzle pieces, match effects, particles
 *   - Cursor plane:   Selection cursor/highlight (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm-puzzle drm-puzzle.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm-puzzle drm-puzzle.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm-puzzle [/dev/dri/cardN]
 *
 * Controls:
 *   - Arrow keys: move cursor
 *   - Space/Enter: select/swap gems
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

/* Cursor plane dimensions (selection highlight) */
#define CURSOR_W            64
#define CURSOR_H            64

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define GRID_W              8
#define GRID_H              8
#define GEM_TYPES           6
#define MIN_MATCH           3

#define ANIM_SWAP_FRAMES    12
#define ANIM_FALL_SPEED     6

#define MAX_PARTICLES       60
#define MAX_SPARKLES        20

/* Gem colors (main, light, dark) */
static const uint32_t gem_colors[GEM_TYPES][3] = {
    {0xFFFF2222, 0xFFFF6666, 0xFFAA0000}, /* Red */
    {0xFF22FF22, 0xFF66FF66, 0xFF00AA00}, /* Green */
    {0xFF2222FF, 0xFF6666FF, 0xFF0000AA}, /* Blue */
    {0xFFFFFF22, 0xFFFFFF88, 0xFFAAAA00}, /* Yellow */
    {0xFFFF22FF, 0xFFFF88FF, 0xFFAA00AA}, /* Purple */
    {0xFFFF8822, 0xFFFFBB66, 0xFFAA5500}, /* Orange */
};

#define COLOR_TRANSPARENT   0x00000000
#define COLOR_BG_TOP        0xFF1A0A2E
#define COLOR_BG_BOT        0xFF0A1A3E
#define COLOR_GRID_BG       0xFF0D0D2A
#define COLOR_CURSOR        0xFFFFFFFF
#define COLOR_SELECTED      0xFF00FFFF
#define COLOR_HUD           0xAAFFFFFF

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
    float x, y, vx, vy;
    float life;
    uint32_t color;
    int size;
    bool active;
};

struct sparkle {
    float x, y;
    float phase;
    float speed;
    bool active;
};

enum anim_state {
    ANIM_NONE = 0,
    ANIM_SWAPPING,
    ANIM_FALLING,
    ANIM_MATCHING,
};

struct gem {
    int type;       /* 0..GEM_TYPES-1, -1 = empty */
    float offset_x; /* animation offset */
    float offset_y;
    bool matched;
    bool falling;
};

struct game_state {
    struct gem grid[GRID_H][GRID_W];
    int cursor_x, cursor_y;
    int selected_x, selected_y;
    bool has_selection;
    enum anim_state anim;
    int anim_timer;
    int swap_x1, swap_y1, swap_x2, swap_y2;
    int score;
    int combo;
    int combo_display_timer;
    bool running;
    bool paused;
    int frame_count;
    struct particle particles[MAX_PARTICLES];
    struct sparkle sparkles[MAX_SPARKLES];
    int cell_size;
    int grid_offset_x;
    int grid_offset_y;
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

    /* Cursor plane (positioned at initial cursor location - top-left of grid) */
    add_plane_to_atomic(req, dev->cursor_plane_id, &dev->cursor_props,
                        dev->cursor_buf.fb_id, dev->crtc_id,
                        0, 0, CURSOR_W, CURSOR_H,
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background + puzzle grid + score/moves HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Puzzle pieces, match effects, particles\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Selection cursor (player-controlled)\n",
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

static inline void put_pixel(uint32_t *fb, uint32_t stride, uint32_t w, uint32_t h, int x, int y, uint32_t c) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h) fb[y * stride + x] = c;
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

static void clear_buffer(uint32_t *fb, uint32_t total_pixels, uint32_t color) {
    for (uint32_t i = 0; i < total_pixels; i++)
        fb[i] = color;
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
        float t = h > 0 ? (float)(j - y) / (float)h : 0;
        uint32_t c = lerp_color(ct, cb, t);
        for (int i = x0; i < x1; i++) fb[j * stride + i] = c;
    }
}

static void draw_rect_outline(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                                int x, int y, int w, int h, int thick, uint32_t color) {
    draw_rect(fb, stride, sw, sh, x, y, w, thick, color);
    draw_rect(fb, stride, sw, sh, x, y+h-thick, w, thick, color);
    draw_rect(fb, stride, sw, sh, x, y, thick, h, color);
    draw_rect(fb, stride, sw, sh, x+w-thick, y, thick, h, color);
}

static void draw_rect_outline_blend(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                                     int x, int y, int w, int h, int thick, uint32_t color) {
    draw_rect_blend(fb, stride, sw, sh, x, y, w, thick, color);
    draw_rect_blend(fb, stride, sw, sh, x, y+h-thick, w, thick, color);
    draw_rect_blend(fb, stride, sw, sh, x, y, thick, h, color);
    draw_rect_blend(fb, stride, sw, sh, x+w-thick, y, thick, h, color);
}

static void draw_circle(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int cx, int cy, int r, uint32_t color) {
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r*r)
                put_pixel(fb, stride, sw, sh, cx+dx, cy+dy, color);
}

/* Draw a gem (rounded rectangle with gradient and shine) */
static void draw_gem(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                     int x, int y, int size, int type) {
    if (type < 0 || type >= GEM_TYPES) return;
    int margin = 2;
    int gx = x + margin, gy = y + margin;
    int gs = size - margin * 2;

    uint32_t main_c = gem_colors[type][0];
    uint32_t light_c = gem_colors[type][1];
    uint32_t dark_c = gem_colors[type][2];

    /* Shadow */
    draw_rect_blend(fb, stride, sw, sh, gx + 2, gy + 2, gs, gs, 0x66000000);

    /* Main body with gradient */
    draw_rect_gradient_v(fb, stride, sw, sh, gx, gy, gs, gs, light_c, dark_c);

    /* Inner highlight */
    int hl_margin = gs / 4;
    draw_rect_blend(fb, stride, sw, sh, gx + hl_margin, gy + hl_margin,
                    gs - hl_margin*2, gs/3, 0x44FFFFFF);

    /* Top-left shine spot */
    int shine_x = gx + gs / 4;
    int shine_y = gy + gs / 4;
    draw_circle(fb, stride, sw, sh, shine_x, shine_y, gs/8 + 1, 0x88FFFFFF);

    /* Border */
    draw_rect_outline(fb, stride, sw, sh, gx, gy, gs, gs, 1, main_c);
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
            if (ev.type == EV_KEY && ev.value == 1) {
                switch (ev.code) {
                case KEY_LEFT:
                    if (game->anim == ANIM_NONE && game->cursor_x > 0) game->cursor_x--;
                    break;
                case KEY_RIGHT:
                    if (game->anim == ANIM_NONE && game->cursor_x < GRID_W-1) game->cursor_x++;
                    break;
                case KEY_UP:
                    if (game->anim == ANIM_NONE && game->cursor_y > 0) game->cursor_y--;
                    break;
                case KEY_DOWN:
                    if (game->anim == ANIM_NONE && game->cursor_y < GRID_H-1) game->cursor_y++;
                    break;
                case KEY_SPACE:
                case KEY_ENTER:
                    if (game->anim != ANIM_NONE) break;
                    if (!game->has_selection) {
                        game->selected_x = game->cursor_x;
                        game->selected_y = game->cursor_y;
                        game->has_selection = true;
                    } else {
                        /* Try swap */
                        int dx = abs(game->cursor_x - game->selected_x);
                        int dy = abs(game->cursor_y - game->selected_y);
                        if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1)) {
                            game->swap_x1 = game->selected_x;
                            game->swap_y1 = game->selected_y;
                            game->swap_x2 = game->cursor_x;
                            game->swap_y2 = game->cursor_y;
                            game->anim = ANIM_SWAPPING;
                            game->anim_timer = ANIM_SWAP_FRAMES;
                        }
                        game->has_selection = false;
                    }
                    break;
                case KEY_P: game->paused = !game->paused; break;
                case KEY_ESC: case KEY_Q: game->running = false; break;
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
                .vx = (randf() - 0.5f) * 6.0f,
                .vy = (randf() - 0.5f) * 6.0f - 2.0f,
                .life = 0.6f + randf() * 0.4f,
                .color = color, .size = 2 + (int)(randf() * 3),
                .active = true
            };
            return;
        }
    }
}

static void fill_grid_no_matches(struct game_state *game) {
    /* Fill with random gems ensuring no initial matches */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int type;
            int attempts = 0;
            do {
                type = rand() % GEM_TYPES;
                attempts++;
                /* Check horizontal match */
                bool h_match = (x >= 2 &&
                    game->grid[y][x-1].type == type &&
                    game->grid[y][x-2].type == type);
                /* Check vertical match */
                bool v_match = (y >= 2 &&
                    game->grid[y-1][x].type == type &&
                    game->grid[y-2][x].type == type);
                if (!h_match && !v_match) break;
            } while (attempts < 100);
            game->grid[y][x].type = type;
            game->grid[y][x].offset_x = 0;
            game->grid[y][x].offset_y = 0;
            game->grid[y][x].matched = false;
            game->grid[y][x].falling = false;
        }
    }
}

static bool check_matches(struct game_state *game) {
    bool found = false;

    /* Clear match flags */
    for (int y = 0; y < GRID_H; y++)
        for (int x = 0; x < GRID_W; x++)
            game->grid[y][x].matched = false;

    /* Horizontal matches */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x <= GRID_W - MIN_MATCH; x++) {
            int type = game->grid[y][x].type;
            if (type < 0) continue;
            int count = 1;
            while (x + count < GRID_W && game->grid[y][x + count].type == type) count++;
            if (count >= MIN_MATCH) {
                for (int k = 0; k < count; k++)
                    game->grid[y][x + k].matched = true;
                found = true;
            }
        }
    }

    /* Vertical matches */
    for (int x = 0; x < GRID_W; x++) {
        for (int y = 0; y <= GRID_H - MIN_MATCH; y++) {
            int type = game->grid[y][x].type;
            if (type < 0) continue;
            int count = 1;
            while (y + count < GRID_H && game->grid[y + count][x].type == type) count++;
            if (count >= MIN_MATCH) {
                for (int k = 0; k < count; k++)
                    game->grid[y + k][x].matched = true;
                found = true;
            }
        }
    }

    return found;
}

static void remove_matches(struct game_state *game) {
    int removed = 0;
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            if (game->grid[y][x].matched) {
                /* Spawn particles */
                int type = game->grid[y][x].type;
                float px = game->grid_offset_x + x * game->cell_size + game->cell_size / 2.0f;
                float py = game->grid_offset_y + y * game->cell_size + game->cell_size / 2.0f;
                if (type >= 0 && type < GEM_TYPES) {
                    for (int p = 0; p < 4; p++)
                        spawn_particle(game, px, py, gem_colors[type][0]);
                }
                game->grid[y][x].type = -1;
                removed++;
            }
        }
    }
    game->score += removed * 10 * (game->combo + 1);
    if (removed > 0) game->combo++;
    game->combo_display_timer = 60;
}

static bool apply_gravity(struct game_state *game) {
    bool moved = false;
    for (int x = 0; x < GRID_W; x++) {
        for (int y = GRID_H - 1; y >= 0; y--) {
            if (game->grid[y][x].type == -1) {
                /* Find gem above */
                for (int above = y - 1; above >= 0; above--) {
                    if (game->grid[above][x].type >= 0) {
                        game->grid[y][x].type = game->grid[above][x].type;
                        game->grid[above][x].type = -1;
                        game->grid[y][x].offset_y = -(float)(y - above) * game->cell_size;
                        moved = true;
                        break;
                    }
                }
            }
        }
        /* Fill empty top cells */
        for (int y = 0; y < GRID_H; y++) {
            if (game->grid[y][x].type == -1) {
                game->grid[y][x].type = rand() % GEM_TYPES;
                game->grid[y][x].offset_y = -(float)(y + 1) * game->cell_size;
                moved = true;
            }
        }
    }
    return moved;
}

static void game_init(struct game_state *game, uint32_t screen_w, uint32_t screen_h) {
    memset(game, 0, sizeof(*game));
    srand((unsigned)time(NULL));

    /* Calculate cell size to fit screen */
    int max_grid_h = (int)screen_h - 60;
    int max_grid_w = (int)screen_w - 20;
    game->cell_size = max_grid_h / GRID_H;
    if (game->cell_size > max_grid_w / GRID_W)
        game->cell_size = max_grid_w / GRID_W;
    if (game->cell_size > 50) game->cell_size = 50;

    game->grid_offset_x = ((int)screen_w - GRID_W * game->cell_size) / 2;
    game->grid_offset_y = ((int)screen_h - GRID_H * game->cell_size) / 2 + 15;

    game->running = true;
    fill_grid_no_matches(game);

    /* Init sparkles */
    for (int i = 0; i < MAX_SPARKLES; i++) {
        game->sparkles[i] = (struct sparkle){
            .x = randf() * screen_w,
            .y = randf() * screen_h,
            .phase = randf() * 6.28f,
            .speed = 0.5f + randf() * 2.0f,
            .active = true
        };
    }
}

static void game_update(struct game_state *game) {
    if (game->paused) return;
    game->frame_count++;

    /* Animation states */
    switch (game->anim) {
    case ANIM_SWAPPING:
        game->anim_timer--;
        if (game->anim_timer <= 0) {
            /* Complete the swap */
            struct gem tmp = game->grid[game->swap_y1][game->swap_x1];
            game->grid[game->swap_y1][game->swap_x1] = game->grid[game->swap_y2][game->swap_x2];
            game->grid[game->swap_y2][game->swap_x2] = tmp;
            game->grid[game->swap_y1][game->swap_x1].offset_x = 0;
            game->grid[game->swap_y1][game->swap_x1].offset_y = 0;
            game->grid[game->swap_y2][game->swap_x2].offset_x = 0;
            game->grid[game->swap_y2][game->swap_x2].offset_y = 0;

            /* Check if swap made a match */
            if (check_matches(game)) {
                game->combo = 0;
                game->anim = ANIM_MATCHING;
                game->anim_timer = 20;
            } else {
                /* Swap back */
                tmp = game->grid[game->swap_y1][game->swap_x1];
                game->grid[game->swap_y1][game->swap_x1] = game->grid[game->swap_y2][game->swap_x2];
                game->grid[game->swap_y2][game->swap_x2] = tmp;
                game->anim = ANIM_NONE;
            }
        } else {
            /* Animate swap offset */
            float progress = 1.0f - (float)game->anim_timer / ANIM_SWAP_FRAMES;
            float dx = (float)(game->swap_x2 - game->swap_x1) * game->cell_size * progress;
            float dy = (float)(game->swap_y2 - game->swap_y1) * game->cell_size * progress;
            game->grid[game->swap_y1][game->swap_x1].offset_x = dx;
            game->grid[game->swap_y1][game->swap_x1].offset_y = dy;
            game->grid[game->swap_y2][game->swap_x2].offset_x = -dx;
            game->grid[game->swap_y2][game->swap_x2].offset_y = -dy;
        }
        break;

    case ANIM_MATCHING:
        game->anim_timer--;
        if (game->anim_timer <= 0) {
            remove_matches(game);
            game->anim = ANIM_FALLING;
            game->anim_timer = 30;
            apply_gravity(game);
        }
        break;

    case ANIM_FALLING: {
        /* Animate falling gems */
        bool still_falling = false;
        for (int y = 0; y < GRID_H; y++) {
            for (int x = 0; x < GRID_W; x++) {
                if (game->grid[y][x].offset_y < -0.5f) {
                    game->grid[y][x].offset_y += ANIM_FALL_SPEED;
                    if (game->grid[y][x].offset_y > 0) game->grid[y][x].offset_y = 0;
                    still_falling = true;
                }
            }
        }
        if (!still_falling) {
            /* Check for cascading matches */
            if (check_matches(game)) {
                game->anim = ANIM_MATCHING;
                game->anim_timer = 15;
            } else {
                game->anim = ANIM_NONE;
                game->combo = 0;
            }
        }
        break;
    }

    case ANIM_NONE:
    default:
        break;
    }

    /* Update particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].vx;
        game->particles[i].y += game->particles[i].vy;
        game->particles[i].vy += 0.15f;
        game->particles[i].life -= 0.025f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }

    /* Combo display timer */
    if (game->combo_display_timer > 0) game->combo_display_timer--;
}

/* ============================================================
 * Rendering - Primary Plane (Background + Grid Board + Score/Moves HUD)
 * ============================================================ */

static void render_primary(struct drm_device *dev, struct game_state *game) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t w = dev->width;
    uint32_t h = dev->height;

    /* Background gradient */
    draw_rect_gradient_v(fb, stride, w, h, 0, 0, (int)w, (int)h, COLOR_BG_TOP, COLOR_BG_BOT);

    /* Background sparkles */
    float t = game->frame_count * 0.05f;
    for (int i = 0; i < MAX_SPARKLES; i++) {
        if (!game->sparkles[i].active) continue;
        float brightness = (sinf(t * game->sparkles[i].speed + game->sparkles[i].phase) + 1.0f) * 0.5f;
        if (brightness > 0.7f) {
            int sx = (int)game->sparkles[i].x;
            int sy = (int)game->sparkles[i].y;
            uint32_t alpha = (uint32_t)(brightness * 100);
            uint32_t sc = (alpha << 24) | 0x00FFFFFF;
            draw_rect_blend(fb, stride, w, h, sx, sy, 2, 2, sc);
        }
    }

    /* Grid background */
    int gx = game->grid_offset_x - 4;
    int gy = game->grid_offset_y - 4;
    int gw = GRID_W * game->cell_size + 8;
    int gh = GRID_H * game->cell_size + 8;
    draw_rect_blend(fb, stride, w, h, gx, gy, gw, gh, 0xCC000000);
    draw_rect_outline(fb, stride, w, h, gx - 1, gy - 1, gw + 2, gh + 2, 1, 0xFF444488);

    /* Draw static (non-animating) gems on primary plane */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            struct gem *g = &game->grid[y][x];
            if (g->type < 0) continue;

            /* Skip gems that are animating (they go on the overlay) */
            if (g->offset_x != 0.0f || g->offset_y != 0.0f) continue;
            if (g->matched && game->anim == ANIM_MATCHING) continue;

            int px = game->grid_offset_x + x * game->cell_size;
            int py = game->grid_offset_y + y * game->cell_size;

            draw_gem(fb, stride, w, h, px, py, game->cell_size, g->type);
        }
    }

    /* HUD */
    draw_rect_blend(fb, stride, w, h, 0, 0, (int)w, 30, 0xAA000022);
    draw_number(fb, stride, w, h, 10, 6, game->score, 3, 0xFF00FF88);

    /* Combo display */
    if (game->combo_display_timer > 0 && game->combo > 1) {
        float fade = (float)game->combo_display_timer / 60.0f;
        uint32_t alpha = (uint32_t)(fade * 255);
        uint32_t cc = (alpha << 24) | 0x00FFFF00;
        draw_number(fb, stride, w, h, (int)w / 2 - 10, 6, game->combo, 3, cc);
    }

    /* Pause */
    if (game->paused) {
        draw_rect_blend(fb, stride, w, h, (int)w/2 - 30, (int)h/2 - 12, 60, 24, 0xCC000022);
        draw_rect(fb, stride, w, h, (int)w/2 - 10, (int)h/2 - 6, 6, 12, 0xFFFFFFFF);
        draw_rect(fb, stride, w, h, (int)w/2 + 4, (int)h/2 - 6, 6, 12, 0xFFFFFFFF);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Animating Pieces + Match Effects + Particles)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t w = OVERLAY_W;
    uint32_t h = OVERLAY_H;

    /* Clear overlay to transparent */
    clear_buffer(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    /* Draw animating gems (those with non-zero offsets or matched/blinking) */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            struct gem *g = &game->grid[y][x];
            if (g->type < 0) continue;

            bool is_animating = (g->offset_x != 0.0f || g->offset_y != 0.0f);
            bool is_matching = (g->matched && game->anim == ANIM_MATCHING);

            if (!is_animating && !is_matching) continue;

            int px = game->grid_offset_x + x * game->cell_size + (int)g->offset_x;
            int py = game->grid_offset_y + y * game->cell_size + (int)g->offset_y;

            /* Flash matched gems (blink effect) */
            if (is_matching) {
                if ((game->frame_count / 4) % 2 == 0) continue; /* blink */
            }

            draw_gem(fb, stride, w, h, px, py, game->cell_size, g->type);
        }
    }

    /* Particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        struct particle *p = &game->particles[i];
        int px = (int)p->x, py = (int)p->y;
        float alpha = p->life;
        uint32_t a = (uint32_t)(alpha * 255);
        uint32_t pc = (p->color & 0x00FFFFFF) | (a << 24);
        draw_rect(fb, stride, w, h, px, py, p->size, p->size, pc);
    }

    /* Selected gem highlight on overlay (so it blends over) */
    if (game->has_selection && game->anim == ANIM_NONE) {
        int sx = game->grid_offset_x + game->selected_x * game->cell_size;
        int sy = game->grid_offset_y + game->selected_y * game->cell_size;
        draw_rect_outline(fb, stride, w, h, sx - 1, sy - 1,
                         game->cell_size + 2, game->cell_size + 2, 2, COLOR_SELECTED);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Selection Cursor / Highlight)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf, struct game_state *game) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t w = buf->width;
    uint32_t h = buf->height;

    /* Clear to transparent */
    clear_buffer(fb, buf->size / 4, COLOR_TRANSPARENT);

    if (game->anim != ANIM_NONE) return;

    /* Draw cursor highlight centered in cursor buffer */
    /* Pulsing glow */
    float pulse = (sinf(game->frame_count * 0.1f) + 1.0f) * 0.5f;
    uint32_t alpha = (uint32_t)(100 + pulse * 155);
    uint32_t cursor_color = (alpha << 24) | (COLOR_CURSOR & 0x00FFFFFF);

    /* Draw the outline within the cursor buffer, centered */
    int margin = 4;
    draw_rect_outline_blend(fb, stride, w, h, margin, margin,
                            (int)w - margin*2, (int)h - margin*2, 2, cursor_color);

    /* Inner glow */
    uint32_t glow_alpha = (uint32_t)(pulse * 60);
    uint32_t glow_color = (glow_alpha << 24) | 0x00FFFFFF;
    draw_rect_blend(fb, stride, w, h, margin + 2, margin + 2,
                    (int)w - margin*2 - 4, (int)h - margin*2 - 4, glow_color);
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
    printf("  DRM Puzzle (Match-3) - Microchip Demo\n");
    printf("  Built with libdrm (KMS/DRM) - 3 Planes\n");
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

    printf("[GAME] Controls: Arrows=Move, Space=Select/Swap, P=Pause, ESC=Quit\n\n");

    uint64_t fps_timer = get_time_ns();
    int frame_count = 0;

    /* Main game loop */
    while (game.running && !g_quit) {
        uint64_t t0 = get_time_ns();

        /* Poll input */
        input_process(&game);

        /* Update game state */
        game_update(&game);

        /* Render primary plane (background + grid board + score HUD) into back buffer */
        render_primary(&dev, &game);

        /* Render overlay plane (animating pieces + match effects + particles) into back buffer */
        render_overlay(&dev, &game);

        /* Render cursor plane (selection highlight) */
        render_cursor(&dev.cursor_buf, &game);

        /* Compute cursor plane position */
        int cursor_x = game.grid_offset_x + game.cursor_x * game.cell_size - (CURSOR_W - game.cell_size) / 2;
        int cursor_y = game.grid_offset_y + game.cursor_y * game.cell_size - (CURSOR_H - game.cell_size) / 2;

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
