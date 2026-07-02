# V4L2-DRM GStreamer Demo Scripts

Demo scripts for the MPFS Video Kit that capture video from an IMX334 sensor
via CSI-2 and display it on HDMI output using the mpfs-dpsub DRM/KMS driver
with GStreamer.

## Overview

These scripts demonstrate live camera video capture and display on the
PolarFire SoC FPGA (MPFS) Video Kit. The video pipeline is:

```
IMX334 Sensor (1920x1080 Bayer)
    |
    v
CSI-2 RX (60000000.csi2rx)
    |
    v
RGB Scaler (60002000.rgb-scaler) - Bayer to RGB conversion + crop/scale
    |
    v
Generic Video Pipeline (6000a000.generic-video-pipeline)
    |
    v
V4L2 video device (/dev/video0) - XRGB8888 format
    |
    v
GStreamer (v4l2src ! videoconvert ! kmssink) - Display via mpfs-dpsub
```

## Prerequisites

### Hardware

- DRM design programmed to the PolarFire SoC Video Kit — a FlashPro Express programming job file is available in the [Video Kit reference design](https://github.com/microchip-fpga-solutions/mpfs250-video-kit-drm)
- PolarFire SoC FPGA (MPFS) Video Kit
- IMX334 camera sensor module connected via CSI-2
- HDMI display connected to the board's HDMI output

### Software

- Yocto WIC image (`mchp-base-image-mpfs-video-kit-drm.rootfs-xxxx.wic.gz`) with V4L2 and DRM/KMS support — available on the [releases page](https://github.com/microchip-fpga-solutions/meta-mchp-fpga-solns/releases)
- `media-ctl` - Media device control utility
- `v4l2-ctl` - V4L2 device control utility
- `gst-launch-1.0` - GStreamer pipeline launcher
- GStreamer plugins: `v4l2src`, `videoconvert`, `kmssink`

### Device Nodes

The following device nodes must be present and accessible:

- `/dev/media0` - Media controller device
- `/dev/video0` - V4L2 video capture device

## Available Scripts

| Script | Resolution | Display Mode | Description |
|--------|-----------|--------------|-------------|
| `run_720p_primary.sh` | 1280x720 | Primary plane | Full-screen 720p display on the primary DRM plane |
| `run_480p_overlay.sh` | 640x480 | Overlay plane | 480p display on the overlay plane at position (0,0) |
| `run_340p_overlay.sh` | 640x480 | Overlay plane | 480p capture rendered as 432x340 overlay at position (100,100) |

## Usage

Run any script directly on the target board:

```sh
# 720p on primary plane (full-screen)
./run_720p_primary.sh

# 480p on overlay plane
./run_480p_overlay.sh

# 340p window on overlay plane (offset from top-left corner)
./run_340p_overlay.sh
```

To stop the video stream, press `Ctrl+C`.

## Pipeline Details

### Media Pipeline Configuration

Each script configures the media pipeline using `media-ctl`:

1. **IMX334 sensor** (`imx334 0-001a`) - Configured for 1920x1080 SRGGB10 Bayer output
2. **CSI-2 RX** (`60000000.csi2rx`) - Receives 1920x1080 SRGGB10 from the sensor
3. **RGB Scaler** (`60002000.rgb-scaler`) - Converts Bayer to RGB888 and crops to target resolution
4. **Generic Video Pipeline** (`6000a000.generic-video-pipeline`) - Outputs RGB888 at the final resolution

### V4L2 Controls

| Control | Value | Purpose |
|---------|-------|---------|
| `analogue_gain` | 80 | Sensor exposure gain |
| `vertical_blanking` | 550 (480p) / 1170 (720p) | Frame timing / effective frame rate |

### GStreamer Sink Properties

The `kmssink` element uses:

- `driver-name=mpfs-dpsub` - Selects the PolarFire SoC display subsystem
- `skip-vsync=true` - Disables vsync for lower latency
- `plane-properties="s,alpha=254"` - Sets overlay plane alpha (overlay scripts only)
- `render-rectangle="<x,y,w,h>"` - Defines the display position and size (overlay scripts only)

## Troubleshooting

### No video output

- Verify the camera module is properly connected
- Check that `/dev/media0` and `/dev/video0` exist
- Ensure the HDMI display is connected and powered on
- Run `media-ctl --print-topology -d /dev/media0` to inspect the pipeline state

### Dark or overexposed image

- Adjust `analogue_gain` value in the script (range depends on sensor configuration)
- Modify `vertical_blanking` to change the effective exposure time

### GStreamer errors

- Ensure all required GStreamer plugins are installed:
  ```sh
  gst-inspect-1.0 v4l2src
  gst-inspect-1.0 videoconvert
  gst-inspect-1.0 kmssink
  ```
- Check for DRM permission issues (may require running as root)

### Pipeline format mismatch

- Use `v4l2-ctl --device /dev/video0 --list-formats-ext` to verify supported formats
- Ensure the resolution set in `v4l2-ctl --set-fmt-video` matches the media pipeline output

## License

This project is licensed under the MIT License. See the file headers for details.
