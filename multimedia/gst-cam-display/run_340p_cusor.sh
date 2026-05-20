#!/bin/sh
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#
# Description:
#   Demo script for MPFS Video Kit HDMI display using GStreamer.
#   Captures 1920x1080 Bayer video from the IMX334 sensor via CSI-2,
#   scales/crops to 640x480 RGB, and renders as a 432x340 overlay
#   at position (100,100) on the HDMI output using the mpfs-dpsub driver.
#
# Prerequisites:
#   - MPFS Video Kit with IMX334 camera module connected
#   - media-ctl, v4l2-ctl, gst-launch-1.0 available in PATH
#   - /dev/media0 and /dev/video0 present and accessible
#

# Find the cursor plane ID for the mpfs-dpsub driver.
# In modetest output, the "type" property has value: 2 for Cursor
# (enums: Overlay=0 Primary=1 Cursor=2).
CURSOR_PLANE_ID=$(modetest -M mpfs-dpsub -p 2>/dev/null | \
    awk '
        /^Planes:/ { in_planes=1; next }
        /^Frame buffers:/ { in_planes=0 }
        in_planes && /^[0-9]/ { plane_id=$1 }
        in_planes && /[0-9]+ type:/ { in_type=1; next }
        in_type && /value:/ { if ($2 == 2) { print plane_id; exit }; in_type=0 }
    ')

if [ -z "$CURSOR_PLANE_ID" ]; then
    echo "Error: Could not find cursor plane for mpfs-dpsub driver" >&2
    exit 1
fi

echo "Using cursor plane ID: $CURSOR_PLANE_ID"

media-ctl -v -V '"imx334 0-001a":0 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60000000.csi2rx":1 [fmt:SRGGB10_1X10/1920x1080 field:none colorspace:srgb xfer:none]' -d /dev/media0
media-ctl -v -V '"60002000.rgb-scaler":0 [fmt:RBG888_1X24/1920x1080  crop: (0,0)/640x480 field:none colorspace:srgb]' -d /dev/media0
media-ctl -v -V '"6000a000.generic-video-pipeline":0 [fmt:RBG888_1X24/640x480 field:none colorspace:srgb]' -d /dev/media0
v4l2-ctl -d /dev/video0 --set-ctrl=analogue_gain=80
v4l2-ctl -d /dev/video0 --set-ctrl=vertical_blanking=300

v4l2-ctl --device /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=XR24

gst-launch-1.0 -v v4l2src device=/dev/video0  ! video/x-raw, width=640, height=480 ! videoconvert ! kmssink driver-name=mpfs-dpsub plane-id=$CURSOR_PLANE_ID skip-vsync=true plane-properties="s,alpha=254" render-rectangle="<100, 100, 432,340>"

