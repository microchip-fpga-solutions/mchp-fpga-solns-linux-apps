# DRM/KMS Game Demos

16 arcade-style game demos for PolarFire SoC FPGA (MPFS) that demonstrate atomic multi-plane rendering using the Linux DRM/KMS subsystem. Each game uses all three hardware plane types exposed by the `mpfs-dpsub` DRM driver:

- **Primary plane** -- Static/scrolling background, HUD (score, lives, level)
- **Overlay plane** -- Dynamic game objects (enemies, projectiles, effects, particles)
- **Cursor plane** -- Player-controlled element (paddle, ship, character, cursor)

All rendering is done entirely in software (CPU-drawn ARGB8888 framebuffers) with triple-buffered atomic commits synchronized to vsync at 60 FPS.

## Games

| Binary | Description |
|--------|-------------|
| `drm-game` | Breakout -- paddle, ball, and bricks |
| `drm-snake` | Classic Snake with growing body segments |
| `drm-tetris` | Tetris with ghost piece and line-clear effects |
| `drm-invaders` | Space Invaders with alien waves and bullets |
| `drm-flappy` | Flappy Bird with scrolling pipes |
| `drm-racing` | Top-down racing with obstacles and laps |
| `drm-platformer` | Side-scrolling platformer with enemies and collectibles |
| `drm-asteroids` | Asteroids with vector-style rotation and splitting rocks |
| `drm-puzzle` | Match-3 puzzle with cascading gems |
| `drm-pong` | Pong against an AI opponent |
| `drm-catch` | Catch falling fruits, dodge bombs |
| `drm-shooter` | Vertical space shooter with enemy waves |
| `drm-breaker` | Brick Breaker with power-ups |
| `drm-ninja` | Fruit Ninja style slicing game |
| `drm-tower` | Tower Defense with path-following enemies |
| `drm-whack` | Whack-a-Mole with timed rounds |

## Dependencies

- `libdrm` (development headers and library)
- `libm` (math library, part of glibc)
- Linux kernel with DRM/KMS support (`mpfs-dpsub` driver)

## Building

### Native (on target)

```sh
make
```

### Cross-compile (RISC-V 64-bit)

```sh
make CC=riscv64-linux-gnu-gcc
```

Or build a single game manually:

```sh
gcc -O2 -o drm-snake drm-snake.c $(pkg-config --cflags --libs libdrm) -lm
```

### Clean

```sh
make clean
```

## Running

Stop any display compositor first, then run any game binary:

```sh
./drm-snake [/dev/dri/cardN]
```

The optional `/dev/dri/cardN` argument specifies which DRM device to use. If omitted, the game scans available DRM devices automatically.

## Controls

All games support keyboard and touchscreen input:

| Key | Action |
|-----|--------|
| Arrow keys | Move player element |
| Space | Primary action (fire, jump, launch, select) |
| P | Pause/unpause |
| ESC / Q | Quit |

Individual games may have additional controls; see the header comment in each source file for details.

## Hardware Requirements

- PolarFire SoC FPGA (MPFS) Video Kit
- HDMI display connected to the board's HDMI output

## License

MIT
