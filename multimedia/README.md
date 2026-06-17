# Multimedia Applications

Linux multimedia applications and demo scripts for the PolarFire SoC FPGA (MPFS) Video Kit. These examples demonstrate video capture, display, and processing using the IMX334 camera sensor, V4L2, and the DRM/KMS display subsystem.

## Directory Overview

| Directory | Description |
|-----------|-------------|
| [raw-bayer-capture/](raw-bayer-capture/) | Bayer pipeline frame capture scripts (raw and JPEG) using `fswebcam` and `v4l2-ctl` at 720p and 1080p. |
| [gst-cam-display/](gst-cam-display/) | GStreamer demo scripts for live IMX334 camera-to-HDMI display at various resolutions and planes (primary and overlay). |
| [cam2display-zerocopy/](cam2display-zerocopy/) | Zero-copy V4L2-to-DRM video display application using DMA-BUF shared buffers (no CPU memcpy). |
| [rgb-jpeg-capture/](rgb-jpeg-capture/) | Single JPEG frame capture from the RGB video pipeline (CSI-2 RX, RGB scaler) via GStreamer. |
| [drm-display-tests/](drm-display-tests/) | Shell scripts for testing DRM/KMS display modes, planes, alpha blending, and GStreamer `kmssink`. |
| [auto-gain-osd-h264/](auto-gain-osd-h264/) | C application for automatic camera gain control based on image enhancement feedback, with H.264 compression ratio displayed on OSD. |
| [drm-games/](drm-games/) | 16 DRM/KMS arcade game demos using atomic multi-plane rendering (primary, overlay, and cursor planes). |

## Hardware Requirements

- PolarFire SoC FPGA (MPFS) Video Kit
- IMX334 camera sensor module connected via CSI-2
- HDMI display connected to the board's HDMI output

## License

MIT
