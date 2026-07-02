#!/bin/sh
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

# Display a JPEG image on the DRM/KMS display
# Usage: ./display_jpeg.sh [image_file] [duration_seconds]
# Default image: frame.jpeg
# Default duration: indefinite (Ctrl+C to stop)

IMAGE="${1:-frame.jpeg}"
DURATION="${2:-}"

if [ ! -f "$IMAGE" ]; then
    echo "Error: Image file '$IMAGE' not found."
    echo "Usage: $0 [image_file] [duration_seconds]"
    exit 1
fi

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

echo "Displaying '$IMAGE' on DRM display..."

if [ -n "$DURATION" ]; then
    echo "Will display for $DURATION seconds."
    NUM_BUFFERS=$((DURATION * 30))
    gst-launch-1.0 filesrc location="$IMAGE" ! jpegdec ! imagefreeze num-buffers="$NUM_BUFFERS" ! videoconvert ! kmssink driver-name=mpfs-dpsub plane-id="$PRIMARY_PLANE_ID" skip-vsync=true
else
    echo "Press Ctrl+C to stop."
    gst-launch-1.0 filesrc location="$IMAGE" ! jpegdec ! imagefreeze ! videoconvert ! kmssink driver-name=mpfs-dpsub plane-id="$PRIMARY_PLANE_ID" skip-vsync=true
fi
