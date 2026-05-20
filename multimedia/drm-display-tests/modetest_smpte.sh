#!/bin/bash
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

export LIBPLANES_DEBUG=1

# Run modetest and extract Connector IDs
echo "Searching for Connector IDs..."
CONNECTORS=$(modetest -M mpfs-dpsub | sed -n '/^Connectors:/,/^[A-Z]/{/^[0-9]/p}' | awk '{print $1}')

if [ -z "$CONNECTORS" ]; then
    echo "No connectors found."
    exit 1
fi

echo "Found Connector IDs:"
echo "$CONNECTORS"

modetest -M mpfs-dpsub -s ${CONNECTORS}:1280x720 -Fsmpte

