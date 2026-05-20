#!/bin/sh
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

# Configure media pipeline
media-ctl -v -V '"imx334 0-001a":0 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60000000.csi2rx":1 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60002000.rgb-scaler":0 [fmt:RBG888_1X24/1920x1080  crop: (0,0)/1280x720 field:none colorspace:srgb]' -d /dev/media0
media-ctl -v -V '"6000a000.generic-video-pipeline":0 [fmt:RBG888_1X24/1280x720 field:none colorspace:srgb]' -d /dev/media0

# Set sensor controls
v4l2-ctl -d /dev/video0 --set-ctrl=analogue_gain=80
v4l2-ctl -d /dev/video0 --set-ctrl=vertical_blanking=1170

# Capture a single JPEG frame from the camera via V4L2
gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=1 ! video/x-raw, width=1280, height=720 ! videoconvert ! jpegenc ! filesink location=frame.jpeg
