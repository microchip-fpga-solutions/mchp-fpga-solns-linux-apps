# Motor Control BLDC Application

Python API Documentation for BLDC/Stepper Motor Control

*Microchip PolarFire SoC FPGA Motor Control Solution*

---

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Software Architecture](#software-architecture)
- [Linux IIO Drivers](#linux-iio-drivers)
- [API Reference](#api-reference)
- [Web Dashboard](#web-dashboard)
- [Configuration](#configuration)

---

## Overview

The Motor Control BLDC Application provides a Python API for controlling BLDC (Brushless DC) and Stepper motors using custom IIO kernel drivers on PolarFire SoC FPGA platforms.

### Key Features

- **Python API** - Simple motor control via `motor_lib_py` module
- **Web Dashboard** - Real-time motor control and monitoring via Bokeh
- **IIO Subsystem** - Linux Industrial I/O drivers for hardware access

### Supported Motor Types

| Motor Type | Speed Range | Direction |
|------------|-------------|-----------|
| BLDC (Sensorless) | 10 - 4000 RPM | CLOCKWISE / COUNTER CLOCKWISE |
| Stepper (Microstepping) | 1 - 100 RPM | Forward / Reverse |

---

## Quick Start

### Basic BLDC Motor Control

```python
import motor_lib_py as motor

# Create BLDC motor instance
bldc = motor.Motor(motor.MotorType.BLDC)

# Check if hardware is available
if bldc.is_valid():
    # Initialize motor with default parameters
    bldc.init()

    # Set speed to 1500 RPM
    bldc.set_speed(1500)

    # Set direction to clockwise
    bldc.set_direction("CLOCKWISE")

    # Start the motor
    bldc.start()

    # ... motor is running ...

    # Stop the motor
    bldc.stop()
```

### Changing Direction

```python
import motor_lib_py as motor

bldc = motor.Motor(motor.MotorType.BLDC)

if bldc.is_valid():
    bldc.init()
    bldc.set_speed(1000)

    # Set direction using string
    bldc.set_direction("CLOCKWISE")
    bldc.start()

    # Change to counter-clockwise
    bldc.stop()
    bldc.set_direction("COUNTER CLOCKWISE")
    bldc.start()

    # Or use integer values: 0 = CW, 1 = CCW
    bldc.stop()
    bldc.set_direction(0)  # Clockwise
    bldc.start()
```

### Clearing Faults

```python
import motor_lib_py as motor

bldc = motor.Motor(motor.MotorType.BLDC)

if bldc.is_valid():
    # Clear any existing faults before starting
    bldc.clear_fault()

    # Now initialize and start
    bldc.init()
    bldc.set_speed(1500)
    bldc.start()
```

### Stepper Motor Control

```python
import motor_lib_py as motor

stepper = motor.Motor(motor.MotorType.STEPPER)

if stepper.is_valid():
    stepper.init()

    # Set speed in RPM (1-100 RPM)
    stepper.set_speed(50)

    # Set direction
    stepper.set_direction(0)  # Forward

    stepper.start()
```

---

## Software Architecture

```
┌─────────────────────────────────────────────┐
│         Web Dashboard (Bokeh)               │
│   gui.py - Real-time plots, motor controls  │
└───────────────────────┬─────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│              Python API                     │
│      motor_lib_py - Motor control module    │
└───────────────────────┬─────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│           Linux IIO Drivers                 │
│  /sys/bus/iio/devices/ - mpfs_mc_* drivers  │
└───────────────────────┬─────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│         FPGA Motor Control IP               │
│  PolarFire SoC FPGA Fabric - Hardware IP    │
└─────────────────────────────────────────────┘
```

### Directory Structure

```
motor-control-bldc/
├── lib/                          # Motor control library
│   ├── motor.cpp                 # Motor control implementation
│   ├── motor.h                   # API declarations
│   ├── motorclass_def.h          # Hardware abstraction classes
│   └── bindings.cpp              # Python module bindings
├── app/                          # Python applications
│   ├── gui.py                    # Bokeh web dashboard
│   ├── sub.py                    # motor command subscriber
│   ├── motor_control_startup.sh  # Startup script
│   └── motor_control_stop.sh     # Stop script
└── scripts/                      # System scripts
    ├── load-motor-modules.sh     # Kernel module loader
    └── motor-modules.service     # systemd service file
```

---

## Linux IIO Drivers

The motor control system uses custom Linux IIO drivers located in `drivers/iio/mpfs-mc/`.

### Driver Modules

| Module | Compatible String | Purpose |
|--------|-------------------|---------|
| mpfs_mc_adc | microchip,adc-scaling-rtl-v4.3 | ADC scaling with current sensing and buffered acquisition |
| mpfs_mc_picon | microchip,speed-id-iq-pi-rtl-v4.2 | Speed/Id/Iq PI controller configuration |
| mpfs_mc_pwm | microchip,pwm3ph-rtl-v4.2 | 3-phase PWM configuration |
| mpfs_mc_ratelim | microchip,rate-limiter-rtl-v4.2 | Rate limiter and direction control |
| mpfs_mc_sqmng | microchip,seq-controller-rtl-v4.2 | Sequencer manager for start/stop/fault |
| mpfs_mc_stptheta | microchip,stepper-theta-rtl-v4.2 | Stepper theta position control |

### Loading Drivers

```bash
# Load required kernel modules
modprobe industrialio
modprobe kfifo_buf
modprobe mpfs_mc_adc
modprobe mpfs_mc_picon
modprobe mpfs_mc_pwm
modprobe mpfs_mc_ratelim
modprobe mpfs_mc_sqmng
modprobe mpfs_mc_stptheta
```

### Verifying Drivers

```bash
# Check IIO devices
ls /sys/bus/iio/devices/

# Check device names
cat /sys/bus/iio/devices/iio:device0/name
cat /sys/bus/iio/devices/iio:device1/name
...
```

---

## API Reference

### Module Import

```python
import motor_lib_py as motor
```

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| motor.MotorType.BLDC | - | BLDC motor type |
| motor.MotorType.STEPPER | - | Stepper motor type |
| motor.DIRECTION_CLOCKWISE | 0 | Clockwise direction |
| motor.DIRECTION_COUNTER_CLOCKWISE | 1 | Counter-clockwise direction |
| motor.CLOCKWISE | "CLOCKWISE" | Clockwise direction string |
| motor.COUNTER_CLOCKWISE | "COUNTER CLOCKWISE" | Counter-clockwise direction string |

### Motor Control Functions

#### `motor.Motor(type: MotorType)`

Creates a Motor instance of the specified type.

**Parameters:**
- `type` - `motor.MotorType.BLDC` or `motor.MotorType.STEPPER`

**Returns:** Motor object

```python
bldc = motor.Motor(motor.MotorType.BLDC)
stepper = motor.Motor(motor.MotorType.STEPPER)
```

---

#### `is_valid() -> bool`

Checks if all required IIO devices are available on the system.

**Returns:** `True` if all devices found, `False` otherwise

```python
bldc = motor.Motor(motor.MotorType.BLDC)
if bldc.is_valid():
    print("Hardware ready")
else:
    print("Hardware not found - check if drivers are loaded")
```

---

#### `init()`

Initializes motor with default parameters. Writes proven working values to all hardware registers.

**Returns:** None

```python
bldc = motor.Motor(motor.MotorType.BLDC)
if bldc.is_valid():
    bldc.init()  # Must call before start()
```

---

#### `start()`

Starts the motor. Motor must be initialized first with `init()`.

**Returns:** None

```python
bldc.init()
bldc.set_speed(1500)
bldc.start()  # Motor starts running
```

---

#### `stop()`

Stops the motor.

**Returns:** None

```python
bldc.stop()  # Motor stops
```

---

#### `set_speed(speed: int) -> int`

Sets motor speed.

**Parameters:**
- `speed` (BLDC) - 10 - 4000 RPM
- `speed` (Stepper) - 1 - 100 RPM

**Returns:** 0 on success, -1 on error (out of range)

```python
# BLDC: Set speed in RPM
result = bldc.set_speed(2000)  # 2000 RPM
if result == -1:
    print("Speed out of range")

# Stepper: Set speed in RPM
stepper.set_speed(50)   # 50 RPM
stepper.set_speed(100)  # 100 RPM (maximum)
```

---

#### `set_direction(dir)`

Sets motor direction.

**Parameters:**
- `dir` (int) - 0 = Clockwise, 1 = Counter-Clockwise
- `dir` (string) - "CLOCKWISE", "CW" or "COUNTER CLOCKWISE", "CCW"

**Returns:** None

```python
# Using string
bldc.set_direction("CLOCKWISE")
bldc.set_direction("COUNTER CLOCKWISE")

# Using integer
bldc.set_direction(0)  # Clockwise
bldc.set_direction(1)  # Counter-Clockwise

# Using constants
bldc.set_direction(motor.DIRECTION_CLOCKWISE)
bldc.set_direction(motor.DIRECTION_COUNTER_CLOCKWISE)
```

---

#### `clear_fault()`

Clears motor fault registers. Call this to recover from fault conditions.

**Returns:** None

```python
# Clear faults before starting
bldc.clear_fault()
bldc.init()
bldc.start()
```

---

#### `get_state() -> int`

Gets sequencer FSM (Finite State Machine) state.

**Returns:** State value (integer), or -1 on error

```python
state = bldc.get_state()
print(f"FSM State: {state}")
```

---

#### `get_bldc_speed() -> Optional[int]`

Gets current BLDC speed in RPM. Only valid for BLDC motors.

**Returns:** Speed in RPM, or `None` on error

```python
speed = bldc.get_bldc_speed()
if speed is not None:
    print(f"Current Speed: {speed} RPM")
```

---

#### `get_bldc_direction() -> Optional[str]`

Gets current BLDC direction. Only valid for BLDC motors.

**Returns:** "CLOCKWISE" or "COUNTER CLOCKWISE", or `None` on error

```python
direction = bldc.get_bldc_direction()
if direction is not None:
    print(f"Direction: {direction}")
```

---

#### `get_bldc_iq() -> Optional[int]`

Gets current BLDC torque current (IQ) value in per-unit (pu). Only valid for BLDC motors.

**Returns:** IQ value in pu, or `None` on error

```python
iq = bldc.get_bldc_iq()
if iq is not None:
    print(f"Torque (IQ): {iq} pu")
```

---

#### `get_type() -> MotorType`

Gets the motor type.

**Returns:** `MotorType.BLDC` or `MotorType.STEPPER`

```python
motor_type = bldc.get_type()
if motor_type == motor.MotorType.BLDC:
    print("This is a BLDC motor")
```

---

### Complete Example

```python
#!/usr/bin/env python3
"""Complete BLDC motor control example"""

import motor_lib_py as motor
import time

def main():
    # Create BLDC motor
    bldc = motor.Motor(motor.MotorType.BLDC)

    # Verify hardware
    if not bldc.is_valid():
        print("Error: Motor hardware not found")
        print("Check if IIO drivers are loaded")
        return

    # Clear any existing faults
    bldc.clear_fault()

    # Initialize motor
    bldc.init()
    print("Motor initialized")

    # Set speed and direction
    bldc.set_speed(1500)
    bldc.set_direction("CLOCKWISE")

    # Start motor
    bldc.start()
    print("Motor started at 1500 RPM, Clockwise")

    # Monitor for 5 seconds
    for i in range(5):
        speed = bldc.get_bldc_speed()
        direction = bldc.get_bldc_direction()
        iq = bldc.get_bldc_iq()
        state = bldc.get_state()

        print(f"[{i+1}s] Speed: {speed} RPM, Dir: {direction}, IQ: {iq} pu, State: {state}")
        time.sleep(1)

    # Change speed
    print("Changing speed to 3000 RPM...")
    bldc.set_speed(3000)
    time.sleep(2)

    speed = bldc.get_bldc_speed()
    print(f"New speed: {speed} RPM")

    # Stop motor
    bldc.stop()
    print("Motor stopped")

if __name__ == "__main__":
    main()
```

---

## Web Dashboard

The Bokeh-based web dashboard provides a real-time interface for motor control and monitoring.

### Features

- **Motor Controls** - Start/Stop buttons, speed slider (10-4000 RPM), direction toggle
- **Real-time Plots (BLDC)** - Speed, Torque (IQ in pu), and Phase Current visualization
- **Configuration Panel** - PI controller tuning, motor specifications
- **BLDC/Stepper Tabs** - Separate controls for each motor type

### BLDC Real-time Plots

| Plot | Data Source | Units | Description |
|------|-------------|-------|-------------|
| Speed Plot | IIO Buffer Channel 0 | RPM | Real-time motor speed from BLDC estimator |
| Torque Plot | IIO Buffer Channel 1 | pu (per-unit) | IQ current representing motor torque |
| Phase Current Plot | IIO Buffer Channel 2 | mV | Current IB converted to millivolts |

### Starting the Dashboard

```bash
# On target device
cd /opt/motor-control-bldc/app
./motor_control_startup.sh

# Access from browser
http://192.168.0.2:5006/gui
```

> **Startup Script Actions:**
> - Configures eth0 to 192.168.0.2
> - Enables IIO buffer channels (in_current0, in_current1, in_current2)
> - Starts subscriber (sub.py)
> - Starts Bokeh server on port 5006

---

## Configuration

### BLDC Speed Limits

| Parameter | Value |
|-----------|-------|
| Minimum Speed | 10 RPM |
| Maximum Speed | 4000 RPM |

### Stepper Speed Limits

| Parameter | Value |
|-----------|-------|
| Minimum Speed | 1 RPM |
| Maximum Speed | 100 RPM |

### Running on Target

```bash
# 1. Load kernel modules (done by systemd service or manually)
modprobe industrialio
modprobe kfifo_buf
modprobe mpfs_mc_adc
modprobe mpfs_mc_picon
modprobe mpfs_mc_pwm
modprobe mpfs_mc_ratelim
modprobe mpfs_mc_sqmng
modprobe mpfs_mc_stptheta

# 2. Start the application
cd /opt/motor-control-bldc/app
./motor_control_startup.sh

# 3. Access web dashboard
# http://<target-ip>:5006/gui
```

---

## License

Microchip Technology Inc. - PolarFire SoC FPGA Solutions

---

*Generated: June 2026*
