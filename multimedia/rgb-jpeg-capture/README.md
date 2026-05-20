# PolarFire SoC Video Kit V4L2 RGB Capture

Script for capturing JPEG frames from the IMX334 camera sensor via the V4L2 RGB
pipeline on the PolarFire SoC FPGA Video Kit.

## Overview

The script configures the media pipeline (CSI-2 receiver, RGB scaler, and
generic video pipeline) and sensor controls, then captures a single JPEG frame
using GStreamer.

## Prerequisites

### Hardware

- PolarFire SoC FPGA Video Kit (MPFS250TS)
- IMX334 camera sensor connected via CSI-2

### Software

- `media-ctl` utility (from `v4l-utils`)
- `v4l2-ctl` utility (from `v4l-utils`)
- GStreamer 1.0 with `gst-launch-1.0`, `v4l2src`, `videoconvert`, and `jpegenc` plugins
- V4L2/media device nodes available (`/dev/video0`, `/dev/media0`)

## Available Scripts

| Script | Description |
|--------|-------------|
| `capture_jpeg.sh` | Configures the media pipeline and captures a single 1280x720 JPEG frame from the camera |

## Usage

```sh
./capture_jpeg.sh
```

The script performs the following steps:

1. Configures the IMX334 sensor output format (SRGGB10_1X10 at 1920x1080)
2. Configures the CSI-2 receiver format
3. Configures the RGB scaler with a crop to 1280x720
4. Configures the generic video pipeline output format
5. Sets sensor analogue gain to 80
6. Sets sensor vertical blanking to 1170
7. Captures a single frame and saves it as `frame.jpeg`

## Pipeline Topology

```
IMX334 (1920x1080 SRGGB10)
    |
CSI-2 RX (60000000.csi2rx)
    |
RGB Scaler (60002000.rgb-scaler) - crop to 1280x720
    |
Generic Video Pipeline (6000a000.generic-video-pipeline)
    |
/dev/video0 (1280x720 RGB)
```

## Output

- `frame.jpeg` - captured JPEG image at 1280x720 resolution

## Troubleshooting

### No frame captured

- Verify the camera sensor is connected and detected: `media-ctl -d /dev/media0 -p`
- Check that `/dev/video0` and `/dev/media0` exist
- Ensure the sensor is streaming: `v4l2-ctl -d /dev/video0 --stream-count=1`

### Image too dark or bright

- Adjust the analogue gain: `v4l2-ctl -d /dev/video0 --set-ctrl=analogue_gain=<value>`
- Adjust the vertical blanking: `v4l2-ctl -d /dev/video0 --set-ctrl=vertical_blanking=<value>`

## License

MIT
