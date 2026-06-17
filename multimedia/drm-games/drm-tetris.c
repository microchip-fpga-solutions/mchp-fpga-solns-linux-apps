/*
 * drm-tetris.c - Tetris Game using DRM Overlay + Cursor Planes
 *
 * Demonstrates usage of all three DRM/KMS plane types on PolarFire SoC:
 *   - Primary plane:  Background + game board grid (placed blocks) + score/level/lines HUD + next piece preview
 *   - Overlay plane:  Line clear effects, ghost piece, particles
 *   - Cursor plane:   Current falling tetromino (player-controlled)
 *
 * Targets: PolarFire SoC (mpfs-dpsub DRM driver)
 * Dependencies: libdrm
 *
 * Build (native):
 *   gcc -O2 -o drm_tetris drm-tetris.c $(pkg-config --cflags --libs libdrm) -lm
 *
 * Build (cross-compile for PolarFire SoC RISC-V):
 *   riscv64-linux-gnu-gcc -O2 -o drm_tetris drm-tetris.c \
 *       -I<sysroot>/usr/include/libdrm -ldrm -lm
 *
 * Run:
 *   systemctl stop weston  # if running
 *   ./drm_tetris [/dev/dri/cardN]
 *
 * Controls:
 *   - Left/Right arrows: move piece
 *   - Up arrow: rotate piece
 *   - Down arrow: soft drop
 *   - SPACE: hard drop
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

/* Cursor plane dimensions (current tetromino) */
#define CURSOR_W            128
#define CURSOR_H            128

/* Plane alpha property value: 255 disables global alpha, uses per-pixel alpha */
#define OVERLAY_ALPHA       255
#define CURSOR_ALPHA        255

#define BOARD_WIDTH         10
#define BOARD_HEIGHT        20
#define CELL_SIZE           20
#define BOARD_OFFSET_X      40
#define BOARD_OFFSET_Y      30

#define INITIAL_DROP_FRAMES 48  /* frames between drops at level 1 */
#define MIN_DROP_FRAMES     2

#define NUM_TETROMINOS      7

#define MAX_PARTICLES       60
#define LINE_CLEAR_FRAMES   20

#define COLOR_BG            0xFF0A0A1A
#define COLOR_BORDER        0xFF333355
#define COLOR_GRID          0xFF151530
#define COLOR_GHOST         0x88FFFFFF
#define COLOR_TEXT          0xFFCCCCCC
#define COLOR_GAMEOVER_BG   0xCC000000
#define COLOR_TRANSPARENT   0x00000000

/* Tetromino colors */
static const uint32_t piece_colors[NUM_TETROMINOS] = {
    0xFF00FFFF, /* I - Cyan */
    0xFF0000FF, /* J - Blue */
    0xFFFF8800, /* L - Orange */
    0xFFFFFF00, /* O - Yellow */
    0xFF00FF00, /* S - Green */
    0xFF9900FF, /* T - Purple */
    0xFFFF0000, /* Z - Red */
};

/* Tetromino shapes (4 rotations, 4x4 grid each) */
/* Encoded as 4 rows of 4 bits */
static const uint16_t tetrominos[NUM_TETROMINOS][4] = {
    /* I */
    { 0x0F00, 0x2222, 0x00F0, 0x4444 },
    /* J */
    { 0x8E00, 0x6440, 0x0E20, 0x44C0 },
    /* L */
    { 0x2E00, 0x4460, 0x0E80, 0xC440 },
    /* O */
    { 0x6600, 0x6600, 0x6600, 0x6600 },
    /* S */
    { 0x6C00, 0x4620, 0x06C0, 0x8C40 },
    /* T */
    { 0x4E00, 0x4640, 0x0E40, 0x4C40 },
    /* Z */
    { 0xC600, 0x2640, 0x0C60, 0x4C80 },
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

struct particle {
    float x, y, vx, vy;
    float life;
    uint32_t color;
    bool active;
};

struct piece {
    int type;       /* 0-6 */
    int rotation;   /* 0-3 */
    int x, y;       /* position on board (can be negative) */
};

struct game_state {
    uint8_t board[BOARD_HEIGHT][BOARD_WIDTH]; /* 0 = empty, 1-7 = piece type+1 */
    struct piece current;
    struct piece next;
    int score;
    int lines_cleared;
    int level;
    int drop_timer;
    int drop_speed;   /* frames between automatic drops */
    bool running;
    bool paused;
    bool game_over;
    /* Input state */
    int soft_drop;    /* > 0 if holding down */
    bool key_left_held;
    bool key_right_held;
    int das_timer;    /* delayed auto-shift timer */
    int das_dir;      /* -1 left, +1 right, 0 none */
    /* Line clear effect */
    int line_clear_timer;
    int clearing_lines[4];
    int clearing_count;
    /* Particles */
    struct particle particles[MAX_PARTICLES];
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

    /* Create cursor plane buffer (current tetromino, ARGB8888 for per-pixel alpha) */
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

    /* Cursor plane (positioned at 0,0 initially - will be updated each frame) */
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
    printf("[PLANE INFO]   1. Primary plane (id=%u): Background + game board + score/level/lines HUD\n",
           dev->primary_plane_id);
    printf("[PLANE INFO]   2. Overlay plane (id=%u): Line clear effects, ghost piece, particles\n",
           dev->overlay_plane_id);
    printf("[PLANE INFO]   3. Cursor  plane (id=%u): Current falling tetromino (player-controlled)\n",
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

static void draw_fill(uint32_t *fb, uint32_t total, uint32_t color) {
    for (uint32_t i = 0; i < total; i++) fb[i] = color;
}

static void draw_rect(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                       int x, int y, int w, int h, uint32_t color) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + w) > (int)sw ? (int)sw : (x + w);
    int y1 = (y + h) > (int)sh ? (int)sh : (y + h);
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            fb[j * stride + i] = color;
}

static void draw_rect_outline(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                               int x, int y, int w, int h, int t, uint32_t color) {
    draw_rect(fb, stride, sw, sh, x, y, w, t, color);
    draw_rect(fb, stride, sw, sh, x, y + h - t, w, t, color);
    draw_rect(fb, stride, sw, sh, x, y, t, h, color);
    draw_rect(fb, stride, sw, sh, x + w - t, y, t, h, color);
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
                draw_rect(fb, stride, sw, sh, x + c*scale, y + r*scale, scale, scale, color);
}

static void draw_number(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                         int x, int y, int num, int scale, uint32_t color) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", num);
    int off = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] >= '0' && buf[i] <= '9')
            draw_digit(fb, stride, sw, sh, x + off, y, buf[i] - '0', scale, color);
        off += 6 * scale;
    }
}

/* ============================================================
 * Tetromino Helpers
 * ============================================================ */

static bool get_cell(int type, int rotation, int row, int col) {
    uint16_t shape = tetrominos[type][rotation];
    int bit = (3 - row) * 4 + (3 - col);
    return (shape >> bit) & 1;
}

static bool piece_fits(struct game_state *game, int type, int rotation, int px, int py) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!get_cell(type, rotation, r, c))
                continue;
            int bx = px + c;
            int by = py + r;
            if (bx < 0 || bx >= BOARD_WIDTH || by >= BOARD_HEIGHT)
                return false;
            if (by < 0) continue; /* allow above top */
            if (game->board[by][bx] != 0)
                return false;
        }
    }
    return true;
}

static int get_ghost_y(struct game_state *game) {
    int gy = game->current.y;
    while (piece_fits(game, game->current.type, game->current.rotation,
                      game->current.x, gy + 1)) {
        gy++;
    }
    return gy;
}

/* ============================================================
 * Particle Helpers
 * ============================================================ */

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void spawn_particles(struct game_state *game, float x, float y, int count, uint32_t color) {
    for (int i = 0; i < count; i++) {
        for (int p = 0; p < MAX_PARTICLES; p++) {
            if (!game->particles[p].active) {
                float angle = randf() * 6.28f;
                float speed = 1.0f + randf() * 3.0f;
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

static void update_particles(struct game_state *game) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        game->particles[i].x += game->particles[i].vx;
        game->particles[i].y += game->particles[i].vy;
        game->particles[i].vx *= 0.95f;
        game->particles[i].vy *= 0.95f;
        game->particles[i].life -= 0.03f;
        if (game->particles[i].life <= 0) game->particles[i].active = false;
    }
}

/* ============================================================
 * Input Handling
 * ============================================================ */

static void input_init(void) {
    char path[64];
    input_count = 0;
    for (int i = 0; i < 8 && input_count < 8; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd >= 0) {
            char name[256] = "Unknown";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            printf("[INPUT] Opened %s: %s\n", path, name);
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
            if (ev.type != EV_KEY) continue;

            bool pressed = (ev.value == 1);
            bool released = (ev.value == 0);

            switch (ev.code) {
            case KEY_LEFT:
                if (pressed && !game->game_over && !game->paused) {
                    if (piece_fits(game, game->current.type, game->current.rotation,
                                   game->current.x - 1, game->current.y))
                        game->current.x--;
                    game->key_left_held = true;
                    game->das_timer = 0;
                    game->das_dir = -1;
                }
                if (released) {
                    game->key_left_held = false;
                    if (game->das_dir == -1) game->das_dir = 0;
                }
                break;

            case KEY_RIGHT:
                if (pressed && !game->game_over && !game->paused) {
                    if (piece_fits(game, game->current.type, game->current.rotation,
                                   game->current.x + 1, game->current.y))
                        game->current.x++;
                    game->key_right_held = true;
                    game->das_timer = 0;
                    game->das_dir = 1;
                }
                if (released) {
                    game->key_right_held = false;
                    if (game->das_dir == 1) game->das_dir = 0;
                }
                break;

            case KEY_UP:
                if (pressed && !game->game_over && !game->paused) {
                    int new_rot = (game->current.rotation + 1) % 4;
                    if (piece_fits(game, game->current.type, new_rot,
                                   game->current.x, game->current.y)) {
                        game->current.rotation = new_rot;
                    }
                    /* Wall kick: try left/right offset */
                    else if (piece_fits(game, game->current.type, new_rot,
                                        game->current.x - 1, game->current.y)) {
                        game->current.rotation = new_rot;
                        game->current.x--;
                    }
                    else if (piece_fits(game, game->current.type, new_rot,
                                        game->current.x + 1, game->current.y)) {
                        game->current.rotation = new_rot;
                        game->current.x++;
                    }
                }
                break;

            case KEY_DOWN:
                if (pressed) game->soft_drop = 1;
                if (released) game->soft_drop = 0;
                break;

            case KEY_SPACE:
                if (pressed && !game->game_over && !game->paused) {
                    /* Hard drop */
                    int gy = get_ghost_y(game);
                    game->score += (gy - game->current.y) * 2;
                    game->current.y = gy;
                    game->drop_timer = game->drop_speed; /* force lock next frame */
                }
                if (pressed && game->game_over) {
                    /* Restart */
                    memset(game->board, 0, sizeof(game->board));
                    game->score = 0;
                    game->lines_cleared = 0;
                    game->level = 1;
                    game->drop_speed = INITIAL_DROP_FRAMES;
                    game->game_over = false;
                    game->current.type = rand() % NUM_TETROMINOS;
                    game->current.rotation = 0;
                    game->current.x = BOARD_WIDTH / 2 - 2;
                    game->current.y = -1;
                    game->next.type = rand() % NUM_TETROMINOS;
                }
                break;

            case KEY_P:
                if (pressed && !game->game_over)
                    game->paused = !game->paused;
                break;

            case KEY_ESC:
            case KEY_Q:
                if (pressed) game->running = false;
                break;

            default:
                break;
            }
        }
    }

    /* Delayed Auto Shift (DAS) */
    if (!game->game_over && !game->paused) {
        if (game->das_dir != 0) {
            game->das_timer++;
            if (game->das_timer > 10 && game->das_timer % 3 == 0) {
                if (piece_fits(game, game->current.type, game->current.rotation,
                               game->current.x + game->das_dir, game->current.y))
                    game->current.x += game->das_dir;
            }
        }
    }
}

/* ============================================================
 * Game Logic
 * ============================================================ */

static void spawn_piece(struct game_state *game) {
    game->current = game->next;
    game->current.x = BOARD_WIDTH / 2 - 2;
    game->current.y = -1;
    game->current.rotation = 0;

    game->next.type = rand() % NUM_TETROMINOS;
    game->next.rotation = 0;

    /* Check game over */
    if (!piece_fits(game, game->current.type, game->current.rotation,
                    game->current.x, game->current.y)) {
        game->game_over = true;
        printf("[GAME] Game Over! Score: %d | Lines: %d | Level: %d\n",
               game->score, game->lines_cleared, game->level);
    }
}

static void lock_piece(struct game_state *game) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!get_cell(game->current.type, game->current.rotation, r, c))
                continue;
            int bx = game->current.x + c;
            int by = game->current.y + r;
            if (bx >= 0 && bx < BOARD_WIDTH && by >= 0 && by < BOARD_HEIGHT)
                game->board[by][bx] = game->current.type + 1;
        }
    }
}

static void clear_lines(struct game_state *game, int board_px, int board_py) {
    int lines = 0;
    game->clearing_count = 0;

    for (int r = BOARD_HEIGHT - 1; r >= 0; r--) {
        bool full = true;
        for (int c = 0; c < BOARD_WIDTH; c++) {
            if (game->board[r][c] == 0) { full = false; break; }
        }
        if (full) {
            /* Spawn particles along the cleared line */
            for (int c = 0; c < BOARD_WIDTH; c += 2) {
                float px = (float)(board_px + c * CELL_SIZE + CELL_SIZE / 2);
                float py = (float)(board_py + r * CELL_SIZE + CELL_SIZE / 2);
                spawn_particles(game, px, py, 2, piece_colors[game->board[r][c] - 1]);
            }
            if (game->clearing_count < 4)
                game->clearing_lines[game->clearing_count++] = r;
            lines++;
            /* Shift everything above down */
            for (int rr = r; rr > 0; rr--)
                memcpy(game->board[rr], game->board[rr - 1], BOARD_WIDTH);
            memset(game->board[0], 0, BOARD_WIDTH);
            r++; /* recheck this row */
        }
    }

    if (lines > 0) {
        /* Scoring: 1=100, 2=300, 3=500, 4=800 */
        static const int line_scores[] = {0, 100, 300, 500, 800};
        int idx = lines > 4 ? 4 : lines;
        game->score += line_scores[idx] * game->level;
        game->lines_cleared += lines;
        game->level = game->lines_cleared / 10 + 1;

        /* Update drop speed */
        game->drop_speed = INITIAL_DROP_FRAMES - (game->level - 1) * 4;
        if (game->drop_speed < MIN_DROP_FRAMES)
            game->drop_speed = MIN_DROP_FRAMES;

        game->line_clear_timer = LINE_CLEAR_FRAMES;
    }
}

static void game_init(struct game_state *game) {
    memset(game, 0, sizeof(*game));
    game->running = true;
    game->drop_speed = INITIAL_DROP_FRAMES;
    game->level = 1;

    srand((unsigned)time(NULL));
    game->current.type = rand() % NUM_TETROMINOS;
    game->current.rotation = 0;
    game->current.x = BOARD_WIDTH / 2 - 2;
    game->current.y = -1;
    game->next.type = rand() % NUM_TETROMINOS;
    game->next.rotation = 0;
}

static void game_update(struct game_state *game, int board_px, int board_py) {
    if (game->paused || game->game_over)
        return;

    /* Update line clear timer */
    if (game->line_clear_timer > 0)
        game->line_clear_timer--;

    /* Update particles */
    update_particles(game);

    /* Determine effective drop speed */
    int effective_speed = game->soft_drop ? 2 : game->drop_speed;

    game->drop_timer++;
    if (game->drop_timer >= effective_speed) {
        game->drop_timer = 0;

        if (piece_fits(game, game->current.type, game->current.rotation,
                       game->current.x, game->current.y + 1)) {
            game->current.y++;
            if (game->soft_drop)
                game->score += 1; /* soft drop bonus */
        } else {
            /* Lock the piece */
            lock_piece(game);
            clear_lines(game, board_px, board_py);
            spawn_piece(game);
        }
    }
}

/* ============================================================
 * Rendering - Primary Plane (Background + Board + HUD)
 * ============================================================ */

static void draw_cell(uint32_t *fb, uint32_t stride, uint32_t sw, uint32_t sh,
                       int px, int py, uint32_t color) {
    /* Filled cell with highlight and shadow */
    draw_rect(fb, stride, sw, sh, px, py, CELL_SIZE, CELL_SIZE, color);
    /* Highlight (top/left) */
    draw_rect(fb, stride, sw, sh, px, py, CELL_SIZE, 2, color | 0x00404040);
    draw_rect(fb, stride, sw, sh, px, py, 2, CELL_SIZE, color | 0x00404040);
    /* Shadow (bottom/right) */
    uint32_t shadow = (color & 0xFF000000) |
                      ((color & 0x00FF0000) >> 1 & 0x00FF0000) |
                      ((color & 0x0000FF00) >> 1 & 0x0000FF00) |
                      ((color & 0x000000FF) >> 1 & 0x000000FF);
    draw_rect(fb, stride, sw, sh, px, py + CELL_SIZE - 2, CELL_SIZE, 2, shadow);
    draw_rect(fb, stride, sw, sh, px + CELL_SIZE - 2, py, 2, CELL_SIZE, shadow);
}

static void render_primary(struct drm_device *dev, struct game_state *game,
                           int board_px, int board_py) {
    int back = (dev->primary_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->primary_buf[back].map;
    uint32_t stride = dev->primary_buf[back].stride / 4;
    uint32_t sw = dev->width;
    uint32_t sh = dev->height;

    draw_fill(fb, dev->primary_buf[back].size / 4, COLOR_BG);

    /* Draw board background */
    draw_rect(fb, stride, sw, sh,
              board_px, board_py,
              BOARD_WIDTH * CELL_SIZE, BOARD_HEIGHT * CELL_SIZE, 0xFF000005);

    /* Draw grid */
    for (int r = 0; r <= BOARD_HEIGHT; r++) {
        int y = board_py + r * CELL_SIZE;
        draw_rect(fb, stride, sw, sh, board_px, y, BOARD_WIDTH * CELL_SIZE, 1, COLOR_GRID);
    }
    for (int c = 0; c <= BOARD_WIDTH; c++) {
        int x = board_px + c * CELL_SIZE;
        draw_rect(fb, stride, sw, sh, x, board_py, 1, BOARD_HEIGHT * CELL_SIZE, COLOR_GRID);
    }

    /* Draw placed blocks */
    for (int r = 0; r < BOARD_HEIGHT; r++) {
        for (int c = 0; c < BOARD_WIDTH; c++) {
            if (game->board[r][c] == 0) continue;
            int px = board_px + c * CELL_SIZE;
            int py = board_py + r * CELL_SIZE;
            draw_cell(fb, stride, sw, sh, px, py, piece_colors[game->board[r][c] - 1]);
        }
    }

    /* Draw board border */
    draw_rect_outline(fb, stride, sw, sh,
                      board_px - 2, board_py - 2,
                      BOARD_WIDTH * CELL_SIZE + 4,
                      BOARD_HEIGHT * CELL_SIZE + 4, 2, COLOR_BORDER);

    /* Draw next piece preview */
    int next_x = board_px + BOARD_WIDTH * CELL_SIZE + 30;
    int next_y = board_py + 20;
    draw_rect_outline(fb, stride, sw, sh,
                      next_x - 5, next_y - 5, 4 * CELL_SIZE + 10, 4 * CELL_SIZE + 10,
                      1, COLOR_BORDER);
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!get_cell(game->next.type, game->next.rotation, r, c))
                continue;
            int px = next_x + c * CELL_SIZE;
            int py = next_y + r * CELL_SIZE;
            draw_cell(fb, stride, sw, sh, px, py, piece_colors[game->next.type]);
        }
    }

    /* Draw HUD (score, level, lines) */
    int hud_x = board_px - 120;
    if (hud_x < 10) hud_x = 10;

    draw_number(fb, stride, sw, sh, hud_x, board_py + 10, game->score, 3, COLOR_TEXT);
    draw_number(fb, stride, sw, sh, hud_x, board_py + 50, game->level, 2, 0xFF88AAFF);
    draw_number(fb, stride, sw, sh, hud_x, board_py + 80, game->lines_cleared, 2, 0xFF88FF88);

    /* Game over overlay */
    if (game->game_over) {
        draw_rect(fb, stride, sw, sh,
                  board_px, board_py + BOARD_HEIGHT * CELL_SIZE / 2 - 20,
                  BOARD_WIDTH * CELL_SIZE, 40, COLOR_GAMEOVER_BG);
        /* X mark */
        int cx = board_px + BOARD_WIDTH * CELL_SIZE / 2;
        int cy = board_py + BOARD_HEIGHT * CELL_SIZE / 2;
        draw_rect(fb, stride, sw, sh, cx - 20, cy - 3, 40, 6, 0xFFFF4444);
        draw_rect(fb, stride, sw, sh, cx - 3, cy - 20, 6, 40, 0xFFFF4444);
    }

    /* Paused overlay */
    if (game->paused && !game->game_over) {
        draw_rect(fb, stride, sw, sh,
                  board_px + BOARD_WIDTH * CELL_SIZE / 2 - 30,
                  board_py + BOARD_HEIGHT * CELL_SIZE / 2 - 10, 60, 20, 0xCC000000);
        draw_rect(fb, stride, sw, sh,
                  board_px + BOARD_WIDTH * CELL_SIZE / 2 - 12,
                  board_py + BOARD_HEIGHT * CELL_SIZE / 2 - 6, 8, 12, COLOR_TEXT);
        draw_rect(fb, stride, sw, sh,
                  board_px + BOARD_WIDTH * CELL_SIZE / 2 + 4,
                  board_py + BOARD_HEIGHT * CELL_SIZE / 2 - 6, 8, 12, COLOR_TEXT);
    }
}

/* ============================================================
 * Rendering - Overlay Plane (Line Clear Effects + Ghost + Particles)
 * ============================================================ */

static void render_overlay(struct drm_device *dev, struct game_state *game,
                           int board_px, int board_py) {
    int back = (dev->overlay_front + 1) % NUM_BUFFERS;
    uint32_t *fb = dev->overlay_buf[back].map;
    uint32_t stride = dev->overlay_buf[back].stride / 4;
    uint32_t sw = OVERLAY_W;
    uint32_t sh = OVERLAY_H;

    /* Clear overlay to transparent */
    draw_fill(fb, dev->overlay_buf[back].size / 4, COLOR_TRANSPARENT);

    if (game->game_over || game->paused)
        return;

    /* Draw ghost piece */
    int ghost_y = get_ghost_y(game);
    if (ghost_y != game->current.y) {
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                if (!get_cell(game->current.type, game->current.rotation, r, c))
                    continue;
                int bx = game->current.x + c;
                int by = ghost_y + r;
                if (by < 0 || by >= BOARD_HEIGHT) continue;
                int px = board_px + bx * CELL_SIZE;
                int py = board_py + by * CELL_SIZE;
                draw_rect_outline(fb, stride, sw, sh, px, py,
                                  CELL_SIZE, CELL_SIZE, 1, COLOR_GHOST);
            }
        }
    }

    /* Line clear flash effect */
    if (game->line_clear_timer > 0) {
        float flash = (float)game->line_clear_timer / LINE_CLEAR_FRAMES;
        uint32_t alpha = (uint32_t)(flash * 200);
        uint32_t flash_color = (alpha << 24) | 0x00FFFFFF;
        for (int i = 0; i < game->clearing_count; i++) {
            int row = game->clearing_lines[i];
            int py = board_py + row * CELL_SIZE;
            draw_rect(fb, stride, sw, sh, board_px, py,
                      BOARD_WIDTH * CELL_SIZE, CELL_SIZE, flash_color);
        }
    }

    /* Particles */
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!game->particles[i].active) continue;
        struct particle *p = &game->particles[i];
        int px = (int)p->x;
        int py = (int)p->y;
        int sz = (int)(p->life * 4) + 1;
        uint32_t a = (uint32_t)(p->life * 255);
        uint32_t pc = (a << 24) | (p->color & 0x00FFFFFF);
        draw_rect(fb, stride, sw, sh, px - sz / 2, py - sz / 2, sz, sz, pc);
    }
}

/* ============================================================
 * Rendering - Cursor Plane (Current Falling Tetromino)
 * ============================================================ */

static void render_cursor(struct drm_buffer *buf, struct game_state *game) {
    uint32_t *fb = buf->map;
    uint32_t stride = buf->stride / 4;
    uint32_t sw = buf->width;
    uint32_t sh = buf->height;

    /* Clear to transparent */
    draw_fill(fb, buf->size / 4, COLOR_TRANSPARENT);

    /* Draw current tetromino cells relative to cursor buffer origin */
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (!get_cell(game->current.type, game->current.rotation, r, c))
                continue;
            int px = c * CELL_SIZE;
            int py = r * CELL_SIZE;
            /* Draw cell with highlight/shadow */
            uint32_t color = piece_colors[game->current.type];
            draw_rect(fb, stride, sw, sh, px, py, CELL_SIZE, CELL_SIZE, color);
            draw_rect(fb, stride, sw, sh, px, py, CELL_SIZE, 2, color | 0x00404040);
            draw_rect(fb, stride, sw, sh, px, py, 2, CELL_SIZE, color | 0x00404040);
            uint32_t shadow = (color & 0xFF000000) |
                              ((color & 0x00FF0000) >> 1 & 0x00FF0000) |
                              ((color & 0x0000FF00) >> 1 & 0x0000FF00) |
                              ((color & 0x000000FF) >> 1 & 0x000000FF);
            draw_rect(fb, stride, sw, sh, px, py + CELL_SIZE - 2, CELL_SIZE, 2, shadow);
            draw_rect(fb, stride, sw, sh, px + CELL_SIZE - 2, py, 2, CELL_SIZE, shadow);
        }
    }
}

/* ============================================================
 * Timing
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
    printf("  DRM Tetris Game (Overlay + Cursor Planes)\n");
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
    game_init(&game);

    printf("[GAME] Controls:\n");
    printf("  Left/Right  : Move piece\n");
    printf("  Up          : Rotate\n");
    printf("  Down        : Soft drop\n");
    printf("  SPACE       : Hard drop / Restart\n");
    printf("  P           : Pause\n");
    printf("  ESC / Q     : Quit\n\n");

    /* Calculate board position (centered horizontally) */
    int board_px = (int)dev.width / 2 - (BOARD_WIDTH * CELL_SIZE) / 2;
    int board_py = BOARD_OFFSET_Y;

    uint64_t frame_start, frame_end, frame_elapsed;

    while (game.running && !g_quit) {
        frame_start = get_time_ns();

        input_process(&game);
        game_update(&game, board_px, board_py);

        /* Render primary plane (background + board + HUD) into back buffer */
        render_primary(&dev, &game, board_px, board_py);

        /* Render overlay plane (ghost piece + line clear effects + particles) into back buffer */
        render_overlay(&dev, &game, board_px, board_py);

        /* Render cursor plane (current falling tetromino) */
        render_cursor(&dev.cursor_buf, &game);

        /* Compute cursor plane position */
        int cursor_x, cursor_y;
        if (!game.game_over && !game.paused && game.current.y >= -1) {
            cursor_x = board_px + game.current.x * CELL_SIZE;
            cursor_y = board_py + game.current.y * CELL_SIZE;
        } else {
            /* Hide cursor plane when paused or game over by moving offscreen */
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

    printf("[GAME] Final Score: %d | Lines: %d | Level: %d\n",
           game.score, game.lines_cleared, game.level);

    /* Cleanup */
    input_cleanup();
    drm_cleanup(&dev);
    return 0;
}
