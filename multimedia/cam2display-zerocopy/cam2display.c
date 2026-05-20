/*
 * Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: MIT
 *
 * cam2display - Zero-copy V4L2-to-DRM video capture and display pipeline.
 *               Main event loop, page-flip handling, and application entry point.
 */

#include <errno.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include "videodev2.h"
#include "drm_display.h"
#include "v4l2_capture.h"

static const char *dri_path = "/dev/dri/card0";
static const char *v4l2_path = "/dev/video0";
static int next_buffer_index = -1;
static int curr_buffer_index = 0;
static volatile sig_atomic_t quit_flag = 0;

static void signal_handler(int sig)
{
	(void)sig;
	quit_flag = 1;
}

static void page_flip_handler(int fd, unsigned int frame,
			unsigned int sec, unsigned int usec,
			void *data)
{
	struct drm_dev_t *dev = data;

	(void)frame;
	(void)sec;
	(void)usec;

	/* If we have a next buffer, then let's return the current one,
	 * and grab the next one.
	 */
	if (next_buffer_index >= 0) {
		v4l2_queue_buffer(dev->v4l2_fd, curr_buffer_index, dev->bufs[curr_buffer_index].dmabuf_fd);
		curr_buffer_index = next_buffer_index;
		next_buffer_index = -1;
	}
	if (drmModePageFlip(fd, dev->crtc_id, dev->bufs[curr_buffer_index].fb_id,
			    DRM_MODE_PAGE_FLIP_EVENT, dev))
		fprintf(stderr, "warning: drmModePageFlip failed\n");
}

static void mainloop(int v4l2_fd, int drm_fd, struct drm_dev_t *dev)
{
	struct v4l2_buffer buf;
	drmEventContext ev;
	int r;

	memset(&ev, 0, sizeof ev);
	ev.version = DRM_EVENT_CONTEXT_VERSION;
	ev.vblank_handler = NULL;
	ev.page_flip_handler = page_flip_handler;

	struct pollfd fds[] = {
		{ .fd = STDIN_FILENO, .events = POLLIN },
		{ .fd = v4l2_fd, .events = POLLIN },
		{ .fd = drm_fd, .events = POLLIN },
	};

	while (!quit_flag) {
		r = poll(fds, 3, 3000);
		if (-1 == r) {
			if (EINTR == errno)
				continue;
			fprintf(stderr, "error in poll: %d\n", errno);
			return;
		}

		if (0 == r) {
			fprintf(stderr, "timeout\n");
			return;
		}

		if (fds[0].revents & POLLIN) {
			fprintf(stdout, "User requested exit\n");
			return;
		}
		if (fds[1].revents & POLLIN) {
			/* Video buffer captured, dequeue it
			 * and store it for scanout.
			 */
			int ret = v4l2_dequeue_buffer(v4l2_fd, &buf);
			if (ret < 0) {
				fprintf(stderr, "fatal: dequeue failed\n");
				return;
			}
			if (ret > 0) {
				/* Return the previous pending buffer to V4L2
				 * if we haven't displayed it yet.
				 */
				if (next_buffer_index >= 0)
					v4l2_queue_buffer(v4l2_fd, next_buffer_index,
							  dev->bufs[next_buffer_index].dmabuf_fd);
				next_buffer_index = buf.index;
			}
		}
		if (fds[2].revents & POLLIN) {
			drmHandleEvent(drm_fd, &ev);
		}
	}
}

static void usage(const char *progname)
{
	fprintf(stderr, "Usage: %s [-d drm_device] [-v video_device]\n", progname);
	fprintf(stderr, "  -d  DRM device path (default: /dev/dri/card0)\n");
	fprintf(stderr, "  -v  V4L2 device path (default: /dev/video0)\n");
	fprintf(stderr, "  -h  Show this help message\n");
}

int main(int argc, char *argv[])
{
	struct drm_dev_t *dev_head, *dev;
	int v4l2_fd, drm_fd;
	int dmabufs[BUFCOUNT];
	int opt, i;

	while ((opt = getopt(argc, argv, "d:v:h")) != -1) {
		switch (opt) {
		case 'd':
			dri_path = optarg;
			break;
		case 'v':
			v4l2_path = optarg;
			break;
		case 'h':
		default:
			usage(argv[0]);
			return (opt == 'h') ? 0 : EXIT_FAILURE;
		}
	}

	/* Install signal handlers for clean shutdown */
	struct sigaction sa;
	sa.sa_handler = signal_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	drm_fd = drm_open(dri_path, 1, 1);
	dev_head = drm_find_dev(drm_fd);

	if (dev_head == NULL) {
		fprintf(stderr, "available drm_dev not found\n");
		return EXIT_FAILURE;
	}

	dev = dev_head;
	drm_setup_fb(drm_fd, dev, 0, 1);

	for (i = 0; i < BUFCOUNT; i++)
		dmabufs[i] = dev->bufs[i].dmabuf_fd;

	v4l2_fd = v4l2_open(v4l2_path);
	v4l2_init(v4l2_fd, dev->width, dev->height, dev->pitch);
	v4l2_init_dmabuf(v4l2_fd, dmabufs, BUFCOUNT);
	v4l2_start_capturing_dmabuf(v4l2_fd);

	dev->v4l2_fd = v4l2_fd;
	dev->drm_fd = drm_fd;

	mainloop(v4l2_fd, drm_fd, dev);

	v4l2_stop_capturing(v4l2_fd);
	v4l2_uninit_device();
	close(v4l2_fd);
	drm_destroy(drm_fd, dev_head);
	return 0;
}
