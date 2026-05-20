/*
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: MIT
 *
 * V4L2 camera capture subsystem - device open, format negotiation,
 * buffer management, and streaming control.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "videodev2.h"
#include "v4l2_capture.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define PCLEAR(x) memset(x, 0, sizeof(*x))

struct buffer *buffers;
static unsigned int n_buffers;
static enum v4l2_memory memory_type;

void v4l2_queue_buffer(int fd, int index, int dmabuf_fd)
{
	struct v4l2_buffer buf;

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = memory_type;
	buf.index = index;
	if (memory_type == V4L2_MEMORY_DMABUF)
		buf.m.fd = dmabuf_fd;
	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
		errno_print("VIDIOC_QBUF");
		exit(EXIT_FAILURE);
	}
}

int v4l2_dequeue_buffer(int fd, struct v4l2_buffer *buf)
{
	PCLEAR(buf);

	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->memory = memory_type;

	if (-1 == xioctl(fd, VIDIOC_DQBUF, buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;
		case EIO:
			/* Could ignore EIO, see spec. */
			/* fall through */
		default:
			errno_print("VIDIOC_DQBUF");
			return -1;
		}
	}

	if (buf->index >= n_buffers) {
		fprintf(stderr, "error: buffer index %d out of range (%d)\n",
			buf->index, n_buffers);
		return -1;
	}
	return 1;
}

void v4l2_stop_capturing(int fd)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
		errno_print("VIDIOC_STREAMOFF");
}

void v4l2_start_capturing_dmabuf(int fd)
{
	enum v4l2_buf_type type;
	unsigned int i;

	/* One buffer held by DRM, the rest queued to video4linux */
	for (i = 1; i < n_buffers; ++i)
		v4l2_queue_buffer(fd, i, buffers[i].dmabuf_fd);

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
		errno_print("VIDIOC_STREAMON");
		exit(EXIT_FAILURE);
	}
}

void v4l2_start_capturing_mmap(int fd)
{
	enum v4l2_buf_type type;
	unsigned int i;

	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
			errno_print("VIDIOC_QBUF");
			exit(EXIT_FAILURE);
		}
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
		errno_print("VIDIOC_STREAMON");
		exit(EXIT_FAILURE);
	}
}

void v4l2_uninit_device(void)
{
	unsigned int i;

	for (i = 0; i < n_buffers; ++i)
		if (buffers[i].start && buffers[i].start != MAP_FAILED)
			if (-1 == munmap(buffers[i].start, buffers[i].length))
				errno_print("munmap");
	free(buffers);
}

void v4l2_init_dmabuf(int fd, int *dmabufs, int count)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = count;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_DMABUF;
	memory_type = req.memory;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno)
			fprintf(stderr, "does not support dmabuf\n");
		else
			errno_print("VIDIOC_REQBUFS");
		exit(EXIT_FAILURE);
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory\n");
		exit(EXIT_FAILURE);
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_DMABUF;
		buf.index       = n_buffers;

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
			errno_print("VIDIOC_QUERYBUF");
			exit(EXIT_FAILURE);
		}
		buffers[n_buffers].index = buf.index;
		buffers[n_buffers].dmabuf_fd = dmabufs[n_buffers];
	}
}

void v4l2_init_mmap(int fd, int count)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = count;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	memory_type = req.memory;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno)
			fprintf(stderr, "does not support mmap\n");
		else
			errno_print("VIDIOC_REQBUFS");
		exit(EXIT_FAILURE);
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory\n");
		exit(EXIT_FAILURE);
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
			errno_print("VIDIOC_QUERYBUF");
			exit(EXIT_FAILURE);
		}

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap(NULL /* start anywhere */,
					buf.length,
					PROT_READ | PROT_WRITE /* required */,
					MAP_SHARED /* recommended */,
					fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start) {
			errno_print("mmap");
			exit(EXIT_FAILURE);
		}
	}
}

void v4l2_init(int fd, int width, int height, int pitch)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno)
			fprintf(stderr, "not a V4L2 device\n");
		else
			errno_print("VIDIOC_QUERYCAP");
		exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "not a video capture device\n");
		exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "does not support streaming i/o\n");
		exit(EXIT_FAILURE);
	}

	CLEAR(cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
				case EINVAL:
					/* Cropping not supported. */
					break;
				default:
					/* Errors ignored. */
					break;
			}
		}
	}

	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_XBGR32;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;
	fmt.fmt.pix.colorspace  = V4L2_COLORSPACE_RAW;
	fmt.fmt.pix.bytesperline = pitch;

	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
		errno_print("VIDIOC_S_FMT");
		exit(EXIT_FAILURE);
	}

	printf("v4l2 negotiated format: ");
	printf("size = %dx%d, ", fmt.fmt.pix.width, fmt.fmt.pix.height);
	printf("pitch = %d bytes, ", fmt.fmt.pix.bytesperline);
	printf("fmt = 0x%08x\n", fmt.fmt.pix.pixelformat);

	/* Validate that the driver accepted our requested format.
	 * A mismatch means the camera would write data in a layout
	 * incompatible with the DRM framebuffer, causing corruption.
	 */
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_XBGR32) {
		fprintf(stderr, "error: driver changed pixel format to 0x%08x\n",
			fmt.fmt.pix.pixelformat);
		exit(EXIT_FAILURE);
	}
	if (fmt.fmt.pix.width != (unsigned)width || fmt.fmt.pix.height != (unsigned)height) {
		fprintf(stderr, "error: driver changed resolution to %dx%d (expected %dx%d)\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height, width, height);
		exit(EXIT_FAILURE);
	}
	if (fmt.fmt.pix.bytesperline != (unsigned)pitch) {
		fprintf(stderr, "warning: driver pitch %d != requested %d\n",
			fmt.fmt.pix.bytesperline, pitch);
	}
}

int v4l2_open(const char *dev_name)
{
	struct stat st;
	int fd;

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	return fd;
}
