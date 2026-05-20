#!/bin/bash
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

export LIBPLANES_DEBUG=1

# Run modetest and capture output
echo "Querying DRM device..."
MODETEST_OUTPUT=$(modetest -M mpfs-dpsub)

# Extract Connector IDs
echo "Searching for Connector IDs..."
CONNECTORS=$(echo "$MODETEST_OUTPUT" | sed -n '/^Connectors:/,/^[A-Z]/{/^[0-9]/p}' | awk '{print $1}')

if [ -z "$CONNECTORS" ]; then
    echo "No connectors found."
    exit 1
fi

echo "Found Connector IDs:"
echo "$CONNECTORS"

# Extract CRTC IDs
CRTC=$(echo "$MODETEST_OUTPUT" | sed -n '/^CRTCs:/,/^[A-Z]/{/^[0-9]/p}' | awk '{print $1}' | head -1)

if [ -z "$CRTC" ]; then
    echo "No CRTCs found."
    exit 1
else
    echo "CRTC ID: $CRTC"
fi

# Extract plane IDs by type from the Planes section
# Type is determined by the "value:" field under the "type" property:
#   Overlay=0, Primary=1, Cursor=2
OVERLAY_PLANE=""
CURSOR_PLANE=""
CURRENT_PLANE=""
IN_TYPE_PROP=0

while IFS= read -r line; do
    # Match plane ID lines (start with a number followed by whitespace)
    if echo "$line" | grep -qE '^[0-9]+\s'; then
        CURRENT_PLANE=$(echo "$line" | awk '{print $1}')
        IN_TYPE_PROP=0
    fi
    # Detect the "type:" property
    if echo "$line" | grep -qE '^\s+[0-9]+ type:'; then
        IN_TYPE_PROP=1
    fi
    # Match the value line within the type property
    if [ "$IN_TYPE_PROP" = "1" ] && echo "$line" | grep -qE '^\s+value:'; then
        TYPE_VALUE=$(echo "$line" | awk '{print $2}')
        if [ "$TYPE_VALUE" = "0" ]; then
            OVERLAY_PLANE=$CURRENT_PLANE
        elif [ "$TYPE_VALUE" = "2" ]; then
            CURSOR_PLANE=$CURRENT_PLANE
        fi
        IN_TYPE_PROP=0
    fi
done <<< "$(echo "$MODETEST_OUTPUT" | sed -n '/^Planes:/,/^Frame buffers:/p')"

if [ -z "$OVERLAY_PLANE" ]; then
    echo "No Overlay plane found."
    exit 1
else
    echo "Overlay Plane ID: $OVERLAY_PLANE"
fi

if [ -z "$CURSOR_PLANE" ]; then
    echo "No Cursor plane found."
    exit 1
else
    echo "Cursor Plane ID: $CURSOR_PLANE"
fi

modetest -M mpfs-dpsub -D 0 -a -s ${CONNECTORS}@${CRTC}:1280x720 -P ${OVERLAY_PLANE}@${CRTC}:1280x720 -w ${OVERLAY_PLANE}:alpha:128 -P ${CURSOR_PLANE}@${CRTC}:1280x720 -w ${CURSOR_PLANE}:alpha:128

