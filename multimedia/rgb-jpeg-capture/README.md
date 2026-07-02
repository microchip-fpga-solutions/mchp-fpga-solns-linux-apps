# PolarFire SoC Video Kit V4L2 RGB Capture

Script for capturing JPEG frames from the IMX334 camera sensor via the V4L2 RGB
pipeline on the PolarFire SoC FPGA Video Kit.

## Overview

The script configures the media pipeline (CSI-2 receiver, RGB scaler, and
generic video pipeline) and sensor controls, then captures a single JPEG frame
using GStreamer.

## Prerequisites

### Hardware

- DRM design programmed to the PolarFire SoC Video Kit — a FlashPro Express programming job file is available in the [Video Kit reference design](https://github.com/microchip-fpga-solutions/mpfs250-video-kit-drm)
- PolarFire SoC FPGA Video Kit (MPFS250TS)
- IMX334 camera sensor connected via CSI-2

### Software

- Yocto WIC image (`mchp-base-image-mpfs-video-kit-drm.rootfs-xxxx.wic.gz`) with V4L2 and DRM/KMS support — available on the [releases page](https://github.com/microchip-fpga-solutions/meta-mchp-fpga-solns/releases)
- `media-ctl` utility (from `v4l-utils`)
- `v4l2-ctl` utility (from `v4l-utils`)
- GStreamer 1.0 with `gst-launch-1.0`, `v4l2src`, `videoconvert`, and `jpegenc` plugins
- V4L2/media device nodes available (`/dev/video0`, `/dev/media0`)

## Available Scripts

| Script | Description |
|--------|-------------|
| `capture_jpeg.sh` | Configures the media pipeline and captures a single 1280x720 JPEG frame from the camera |
| `display_jpeg.sh` | Displays a JPEG image on the DRM/KMS connected display |

## Usage

### Capture a JPEG frame

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

### Display a JPEG on the DRM screen

```sh
./display_jpeg.sh [image_file] [duration_seconds]
```

- Defaults to `frame.jpeg` if no argument is provided.
- Displays the image on the DRM/KMS connected monitor using GStreamer's `kmssink`.
- If `duration_seconds` is specified, the display stops automatically after that time.
- Otherwise, press `Ctrl+C` to stop the display.

Example:

```sh
# Display the default captured frame (indefinite)
./display_jpeg.sh

# Display a specific image for 10 seconds
./display_jpeg.sh /path/to/image.jpeg 10

# Display the default frame for 5 seconds (useful in scripts)
./display_jpeg.sh frame.jpeg 5
```

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

## Expected Output

After running `capture_jpeg.sh`, the script produces `frame.jpeg` in the current
working directory. To verify the image was captured correctly, use one of the
following methods:

### Option 1: Display on the connected monitor

Use the provided `display_jpeg.sh` script to flash the captured frame on the DRM
display:

```sh
./display_jpeg.sh frame.jpeg
```

### Option 2: Copy to host PC and view

Use `scp` to transfer the image from the board to your host PC for inspection:

```sh
# Run from host PC (replace <board-ip> with the board's IP address)
scp root@<board-ip>:/path/to/frame.jpeg .
```

Then open `frame.jpeg` with any image viewer on the host PC.

### Verifying a valid capture

A successful capture should produce:

- A JPEG file of non-zero size (typically 100 KB - 500 KB depending on scene
  complexity)
- Image resolution of 1280x720 pixels
- A recognizable image of whatever the camera sensor is pointing at

You can quickly verify the file size and type on the board:

```sh
ls -la frame.jpeg
file frame.jpeg
```

Expected output:

```
-rw-r--r-- 1 root root 245760 Jan  1 00:00 frame.jpeg
frame.jpeg: JPEG image data, JFIF standard 1.01, resolution (DPI), 72 x 72, segment length 16, baseline, precision 8, 1280x720, components 3
```

If `frame.jpeg` is 0 bytes or the `file` command does not identify it as JPEG
image data, refer to the [Troubleshooting](#troubleshooting) section below.

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
