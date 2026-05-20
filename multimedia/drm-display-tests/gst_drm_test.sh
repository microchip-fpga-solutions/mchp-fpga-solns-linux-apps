#!/bin/bash
#
# Copyright (c) 2026 Microchip Technology Inc. and its subsidiaries.
#
# SPDX-License-Identifier: MIT
#

export LIBPLANES_DEBUG=1

gst-launch-1.0 -v videotestsrc pattern=10 ! video/x-raw, width=1280, height=720, interlace-mode=progressive ! videoconvert ! kmssink driver-name=mpfs-dpsub skip-vsync=true
