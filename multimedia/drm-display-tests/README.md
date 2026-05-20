# PolarFire SoC Video Kit DRM Test Scripts

Test scripts for verifying the DRM/KMS display subsystem (`mpfs-dpsub`) on the
PolarFire SoC FPGA (MPFS) Video Kit. These scripts exercise the DRM driver's
mode-setting and plane capabilities via standard Linux DRM utilities.

## Overview

The scripts use the `mpfs-dpsub` DRM/KMS driver to test the PolarFire SoC
Display Subsystem (DPSUB) HDMI output. They enable debug output from
`libplanes` to aid in diagnosing display issues.

## Prerequisites

### Hardware

- PolarFire SoC FPGA Video Kit (MPFS250TS)
- HDMI display connected to the board's HDMI output

### Software

- `modetest` utility (from `libdrm-tests` / `libdrm`)
- `planes` utility (from `libplanes`)
- GStreamer 1.0 with `gst-launch-1.0`, `videotestsrc`, `videoconvert`, and `kmssink` plugins
- DRM driver `mpfs-dpsub` loaded and functional

### Device Nodes

- `/dev/dri/card0` (or equivalent DRM device node)

## Available Scripts

| Script | Description |
|--------|-------------|
| `modetest.sh` | Runs `modetest` to query and display DRM resources (connectors, CRTCs, planes, modes) for the `mpfs-dpsub` driver |
| `modetest_tiles.sh` | Auto-detects the connector ID and displays a 1280x720 tiles test pattern |
| `modetest_alpha.sh` | Auto-detects connector, CRTC, overlay, and cursor plane IDs, then runs an alpha blending test on overlay and cursor planes |
| `gst_drm_test.sh` | Uses GStreamer to render a video test pattern via `kmssink` to the DRM display |
| `planes_test.sh` | Runs `planes` in verbose mode to test DRM plane configuration on the `mpfs-dpsub` driver |

## Usage

### Mode Test

Displays DRM connector, CRTC, and plane information for the display subsystem:

```sh
./modetest.sh
```

This executes:

```sh
export LIBPLANES_DEBUG=1
modetest -M mpfs-dpsub
```

### Tiles Test

Auto-detects the connector ID and displays a tiles pattern at 1280x720:

```sh
./modetest_tiles.sh
```

The script queries `modetest` output to find the connector ID, then runs:

```sh
modetest -M mpfs-dpsub -s <connector_id>:1280x720 -Ftiles
```

### Alpha Blending Test

Auto-detects connector, CRTC, overlay plane, and cursor plane IDs, then runs
an alpha blending test with both overlay and cursor planes at 50% alpha:

```sh
./modetest_alpha.sh
```

The script parses the `modetest` output to identify plane types based on the
`type` property value (`Overlay=0`, `Primary=1`, `Cursor=2`), then runs:

```sh
modetest -M mpfs-dpsub -D 0 -a -s <connector>@<crtc>:1280x720 \
  -P <overlay_plane>@<crtc>:1280x720 -w <overlay_plane>:alpha:128 \
  -P <cursor_plane>@<crtc>:1280x720 -w <cursor_plane>:alpha:128
```

### GStreamer DRM Test

Uses GStreamer to render a video test pattern (pattern 10: circular) directly
to the DRM display via `kmssink`:

```sh
./gst_drm_test.sh
```

This executes:

```sh
gst-launch-1.0 -v videotestsrc pattern=10 ! video/x-raw, width=1280, height=720, interlace-mode=progressive ! videoconvert ! kmssink driver-name=mpfs-dpsub skip-vsync=true
```

### Planes Test

Tests DRM plane configuration with verbose output:

```sh
./planes_test.sh
```

This executes:

```sh
export LIBPLANES_DEBUG=1
planes -v -d mpfs-dpsub
```

## Environment Variables

| Variable | Value | Description |
|----------|-------|-------------|
| `LIBPLANES_DEBUG` | `1` | Enables debug output from the libplanes library |

## Troubleshooting

### No display output

- Verify the HDMI cable is connected and the display is powered on
- Check that the `mpfs-dpsub` DRM driver is loaded: `cat /sys/class/drm/card*/device/driver/module/drivers`
- Verify the DRM device node exists: `ls /dev/dri/`

### modetest or planes not found

- Ensure `libdrm-tests` and `libplanes` packages are installed on the target
- Verify the tools are in your `PATH`

## License

MIT
