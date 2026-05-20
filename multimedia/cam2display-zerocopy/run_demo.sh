#!/bin/sh
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#
# Description:
#   Demo script for MPFS Video Kit HDMI display using GStreamer.
#   Captures 1920x1080 Bayer video from the IMX334 sensor via CSI-2,
#   scales/crops to 1280x720 RGB, and renders on the primary plane
#   of the HDMI output using the mpfs-dpsub driver.
#
# Prerequisites:
#   - MPFS Video Kit with IMX334 camera module connected
#   - media-ctl, v4l2-ctl, gst-launch-1.0 available in PATH
#   - /dev/media0 and /dev/video0 present and accessible
#

media-ctl -v -V '"imx334 0-001a":0 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60000000.csi2rx":1 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60002000.rgb-scaler":0 [fmt:RBG888_1X24/1920x1080  crop: (0,0)/1280x720 field:none colorspace:srgb]' -d /dev/media0
media-ctl -v -V '"6000a000.generic-video-pipeline":0 [fmt:RBG888_1X24/1280x720 field:none colorspace:srgb]' -d /dev/media0
v4l2-ctl -d /dev/video0 --set-ctrl=analogue_gain=80
v4l2-ctl -d /dev/video0 --set-ctrl=vertical_blanking=300
v4l2-ctl --device /dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=XR24

./cam2display

