# cam2display

A zero-copy V4L2-to-DRM video capture and display pipeline for Linux.

Captures video frames from a camera device and displays them directly on screen
using DMA-BUF buffer sharing -- no CPU memcpy required.

## Features

- Zero-copy frame path using DMA-BUF shared buffers between V4L2 and DRM
- Page-flip synchronised display (vsync, no tearing)
- Poll-based event loop with 3-second timeout watchdog
- Graceful shutdown via Enter key, SIGINT, or SIGTERM
- Configurable DRM and V4L2 device paths via command-line options
- 4-buffer rotation for smooth capture-to-display pipelining

## Prerequisites

- DRM design programmed to the PolarFire SoC Video Kit — a FlashPro Express programming job file is available in the [Video Kit reference design](https://github.com/microchip-fpga-solutions/mpfs250-video-kit-drm)
- Yocto WIC image (`mchp-base-image-mpfs-video-kit-drm.rootfs-xxxx.wic.gz`) with V4L2 and DRM/KMS support — available on the [releases page](https://github.com/microchip-fpga-solutions/meta-mchp-fpga-solns/releases)
- Linux kernel with V4L2 and DRM/KMS support
- `libdrm` development headers and pkg-config file
- GCC toolchain (native or cross-compilation)
- A V4L2-compatible camera device (e.g. IMX334 on MPFS Video Kit)
- A DRM/KMS-capable display output (e.g. HDMI via mpfs-dpsub)

## Build

Native build:

```
make
```

Cross-compilation (e.g. for RISC-V target):

```
make CROSS_COMPILE=riscv64-unknown-linux-gnu- SYSROOT=/path/to/sysroot
```

Clean build artifacts:

```
make clean
```

## Usage

**Note:** The video pipeline must be configured before running `cam2display`.
Use `media-ctl` and `v4l2-ctl` to set up the sensor format, CSI-2 receiver,
RGB scaler, and video pipeline (see `run_demo.sh` for a complete example on the
MPFS Video Kit).

```
./cam2display [-d drm_device] [-v video_device] [-h]
```

### Options

| Option | Description                         | Default           |
|--------|-------------------------------------|-------------------|
| `-d`   | DRM device path                     | `/dev/dri/card0`  |
| `-v`   | V4L2 video device path              | `/dev/video0`     |
| `-h`   | Show help message                   |                   |

### Stopping the application

- Press **Enter** on stdin
- Send **SIGINT** (Ctrl+C) or **SIGTERM**

### Demo script (MPFS Video Kit)

A ready-to-run demo script is provided for the MPFS Video Kit with an IMX334
camera module and HDMI display output:

```
./run_demo.sh
```

This script configures the media pipeline (sensor format, CSI-2 receiver,
RGB scaler, video pipeline) to produce 1280x720 XR24 frames, then launches
`cam2display`.

## Architecture

- **cam2display.c** - Main event loop, page-flip handler, and application entry point
- **drm_display.c / drm_display.h** - DRM/KMS display subsystem (device open, connector/CRTC discovery, dumb buffer allocation, DMA-BUF export, framebuffer creation, page flips, teardown)
- **v4l2_capture.c / v4l2_capture.h** - V4L2 camera capture subsystem (device init, format negotiation, DMA-BUF import, buffer queue/dequeue, stream start/stop)
- **videodev2.h** - Local copy of V4L2 userspace header

## How it works

1. DRM subsystem opens the GPU device, finds a connected display, and allocates
   4 dumb buffers exported as DMA-BUF file descriptors.
2. V4L2 subsystem opens the camera, negotiates the capture format to match the
   display resolution, and imports the DMA-BUF FDs as DMABUF-type V4L2 buffers.
3. All buffers are queued to V4L2 and streaming begins.
4. The main loop uses `poll()` on stdin, the V4L2 device, and the DRM device:
   - V4L2 readable: a frame has been captured -- dequeue it and mark it for display.
   - DRM readable: a page-flip completed -- swap the displayed buffer and return
     the previous one to V4L2 for reuse.
   - stdin readable: the user pressed Enter -- exit gracefully.
5. On exit, streaming is stopped, devices are closed, and the original CRTC state
   is restored.

## License

MIT -- see source file headers for details.
