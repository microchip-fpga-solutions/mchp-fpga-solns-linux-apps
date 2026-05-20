/*
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: MIT
 *
 * DRM/KMS display subsystem - device open, connector enumeration,
 * dumb buffer allocation, framebuffer setup, and teardown.
 */

#define _GNU_SOURCE
#define _XOPEN_SOURCE 701

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <libdrm/drm.h>
#include "drm_display.h"

enum {
	DEPTH = 24,
	BPP = 32,
};

static int eopen(const char *path, int flag)
{
	int fd;

	if ((fd = open(path, flag)) < 0) {
		fprintf(stderr, "cannot open \"%s\"\n", path);
		error("open");
	}
	return fd;
}

static void *emmap(size_t len, int prot, int flag, int fd, off_t offset)
{
	void *fp;

	if ((fp = mmap(NULL, len, prot, flag, fd, offset)) == MAP_FAILED)
		error("mmap");
	return fp;
}

int drm_open(const char *path, int need_dumb, int need_prime)
{
	int fd, flags;
	uint64_t has_it;

	fd = eopen(path, O_RDWR);

	/* set FD_CLOEXEC flag */
	if ((flags = fcntl(fd, F_GETFD)) < 0
		|| fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0)
		fatal("fcntl FD_CLOEXEC failed");

	if (need_dumb) {
		if (drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_it) < 0)
			error("drmGetCap DRM_CAP_DUMB_BUFFER failed!");
		if (has_it == 0)
			fatal("can't give us dumb buffers");
	}

	if (need_prime) {
		/* check prime */
		if (drmGetCap(fd, DRM_CAP_PRIME, &has_it) < 0)
			error("drmGetCap DRM_CAP_PRIME failed!");
		if (!(has_it & DRM_PRIME_CAP_EXPORT))
			fatal("can't export dmabuf");
	}

	return fd;
}

struct drm_dev_t *drm_find_dev(int fd)
{
	int i, m;
	struct drm_dev_t *dev = NULL, *dev_head = NULL;
	drmModeRes *res;
	drmModeConnector *conn;
	drmModeEncoder *enc;
	drmModeModeInfo *mode = NULL, *preferred = NULL;

	if ((res = drmModeGetResources(fd)) == NULL)
		fatal("drmModeGetResources() failed");

	/* find all available connectors */
	for (i = 0; i < res->count_connectors; i++) {
		conn = drmModeGetConnector(fd, res->connectors[i]);

		if (conn != NULL && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0) {
			dev = malloc(sizeof(*dev));
			if (!dev)
				fatal("out of memory");
			memset(dev, 0, sizeof(*dev));

			preferred = NULL;

			/* find preferred mode */
			for (m = 0; m < conn->count_modes; m++) {
				mode = &conn->modes[m];
				if (mode->type & DRM_MODE_TYPE_PREFERRED)
					preferred = mode;
				fprintf(stdout, "mode: %dx%d %s\n", mode->hdisplay, mode->vdisplay, mode->type & DRM_MODE_TYPE_PREFERRED ? "*" : "");
			}

			if (!preferred)
				preferred = &conn->modes[0];

			dev->conn_id = conn->connector_id;
			dev->enc_id = conn->encoder_id;
			dev->next = NULL;

			memcpy(&dev->mode, preferred, sizeof(drmModeModeInfo));
			dev->width = preferred->hdisplay;
			dev->height = preferred->vdisplay;

			/* FIXME: use default encoder/crtc pair */
			if ((enc = drmModeGetEncoder(fd, dev->enc_id)) == NULL) {
				free(dev);
				fatal("drmModeGetEncoder() failed");
			}
			dev->crtc_id = enc->crtc_id;
			drmModeFreeEncoder(enc);

			dev->saved_crtc = NULL;

			/* create dev list */
			dev->next = dev_head;
			dev_head = dev;
		}
		drmModeFreeConnector(conn);
	}

	drmModeFreeResources(res);

	printf("selected connector(s)\n");
	for (dev = dev_head; dev != NULL; dev = dev->next) {
		printf("connector id:%d\n", dev->conn_id);
		printf("\tencoder id:%d crtc id:%d\n", dev->enc_id, dev->crtc_id);
		printf("\twidth:%d height:%d\n", dev->width, dev->height);
	}

	return dev_head;
}

static void drm_setup_buffer(int fd, struct drm_dev_t *dev,
		int width, int height,
		struct drm_buffer_t *buffer, int map, int export)
{
	struct drm_mode_create_dumb create_req;
	struct drm_mode_map_dumb map_req;

	buffer->dmabuf_fd = -1;

	memset(&create_req, 0, sizeof(struct drm_mode_create_dumb));
	create_req.width = width;
	create_req.height = height;
	create_req.bpp = BPP;

	if (drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_req) < 0)
		fatal("drmIoctl DRM_IOCTL_MODE_CREATE_DUMB failed");

	buffer->pitch = create_req.pitch;
	buffer->size = create_req.size;
	/* GEM buffer handle */
	buffer->bo_handle = create_req.handle;

	if (export) {
		int ret;

		ret = drmPrimeHandleToFD(fd, buffer->bo_handle,
			DRM_CLOEXEC | DRM_RDWR, &buffer->dmabuf_fd);
		if (ret < 0)
			fatal("could not export the dumb buffer");
	}

	if (map) {
		memset(&map_req, 0, sizeof(struct drm_mode_map_dumb));
		map_req.handle = buffer->bo_handle;

		if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map_req))
			fatal("drmIoctl DRM_IOCTL_MODE_MAP_DUMB failed");
		buffer->buf = (uint32_t *) emmap(buffer->size,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, map_req.offset);
	}
}

void drm_setup_dumb(int fd, struct drm_dev_t *dev, int map, int export)
{
	int i;

	for (i = 0; i < BUFCOUNT; i++)
		drm_setup_buffer(fd, dev, dev->width, dev->height,
				 &dev->bufs[i], map, export);

	/* Assume all buffers have the same pitch */
	dev->pitch = dev->bufs[0].pitch;
	printf("DRM: buffer pitch = %d bytes\n", dev->pitch);
}

void drm_setup_fb(int fd, struct drm_dev_t *dev, int map, int export)
{
	int i;

	for (i = 0; i < BUFCOUNT; i++) {
		int ret;

		drm_setup_buffer(fd, dev, dev->width, dev->height,
				 &dev->bufs[i], map, export);

		ret = drmModeAddFB(fd, dev->width, dev->height,
			DEPTH, BPP, dev->bufs[i].pitch,
			dev->bufs[i].bo_handle, &dev->bufs[i].fb_id);
		if (ret)
			fatal("drmModeAddFB failed");
	}

	/* Assume all buffers have the same pitch */
	dev->pitch = dev->bufs[0].pitch;
	printf("DRM: buffer pitch %d bytes\n", dev->pitch);

	dev->saved_crtc = drmModeGetCrtc(fd, dev->crtc_id); /* must store crtc data */

	/* First buffer to DRM */
	if (drmModeSetCrtc(fd, dev->crtc_id, dev->bufs[0].fb_id, 0, 0, &dev->conn_id, 1, &dev->mode))
		fatal("drmModeSetCrtc() failed");

	/* First flip */
	if (drmModePageFlip(fd, dev->crtc_id,
			    dev->bufs[0].fb_id, DRM_MODE_PAGE_FLIP_EVENT,
			    dev))
		fatal("initial drmModePageFlip() failed");
}

void drm_destroy(int fd, struct drm_dev_t *dev_head)
{
	struct drm_dev_t *devp, *devp_tmp;
	int i;

	for (devp = dev_head; devp != NULL;) {
		if (devp->saved_crtc) {
			drmModeSetCrtc(fd, devp->saved_crtc->crtc_id, devp->saved_crtc->buffer_id,
				devp->saved_crtc->x, devp->saved_crtc->y, &devp->conn_id, 1, &devp->saved_crtc->mode);
			drmModeFreeCrtc(devp->saved_crtc);
		}

		for (i = 0; i < BUFCOUNT; i++) {
			struct drm_mode_destroy_dumb dreq = { .handle = devp->bufs[i].bo_handle };

			if (devp->bufs[i].buf)
				munmap(devp->bufs[i].buf, devp->bufs[i].size);
			if (devp->bufs[i].dmabuf_fd >= 0)
				close(devp->bufs[i].dmabuf_fd);
			drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
			if (devp->bufs[i].fb_id)
				drmModeRmFB(fd, devp->bufs[i].fb_id);
		}

		devp_tmp = devp;
		devp = devp->next;
		free(devp_tmp);
	}

	close(fd);
}

