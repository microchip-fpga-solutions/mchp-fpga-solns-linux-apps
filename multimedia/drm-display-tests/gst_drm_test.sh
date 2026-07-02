#!/bin/bash
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

export LIBPLANES_DEBUG=1

# Find the primary plane ID for the mpfs-dpsub driver.
# In modetest output, planes are listed with their properties. The "type"
# property has value: 1 for Primary (enums: Overlay=0 Primary=1 Cursor=2).
PRIMARY_PLANE_ID=$(modetest -M mpfs-dpsub -p 2>/dev/null | \
    awk '
        /^Planes:/ { in_planes=1; next }
        /^Frame buffers:/ { in_planes=0 }
        in_planes && /^[0-9]/ { plane_id=$1 }
        in_planes && /[0-9]+ type:/ { in_type=1; next }
        in_type && /value:/ { if ($2 == 1) { print plane_id; exit }; in_type=0 }
    ')

if [ -z "$PRIMARY_PLANE_ID" ]; then
    echo "Error: Could not find primary plane for mpfs-dpsub driver" >&2
    exit 1
fi

echo "Using primary plane ID: $PRIMARY_PLANE_ID"

gst-launch-1.0 -v videotestsrc pattern=10 ! video/x-raw, width=1280, height=720, interlace-mode=progressive ! videoconvert ! kmssink driver-name=mpfs-dpsub plane-id="$PRIMARY_PLANE_ID" skip-vsync=true
