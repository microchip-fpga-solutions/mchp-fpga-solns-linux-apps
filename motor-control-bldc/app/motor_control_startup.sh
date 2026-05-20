#!/bin/bash
# Motor Control BLDC Startup Script
# Configures IIO buffers and starts the Bokeh application

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_FILE="/var/log/motor_control.log"

# Suppress kernel messages from console
dmesg -n 1

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

cleanup_and_exit() {
    log_msg "Error: $1"
    # Kill sub.py if it was started
    if [ -n "$SUB_PID" ] && kill -0 "$SUB_PID" 2>/dev/null; then
        log_msg "Killing subscriber (PID: $SUB_PID)..."
        kill "$SUB_PID" 2>/dev/null || true
        rm -f /tmp/motor_control_sub.pid
    fi
    exit 1
}

log_msg "Starting Motor Control BLDC initialization..."

# Source ROS2 environment
log_msg "Sourcing ROS2 Humble environment..."
source /opt/ros/humble/setup.sh

# Configure network interface for Bokeh web access
log_msg "Configuring eth0 to 192.168.0.2..."
if ip link show eth0 > /dev/null 2>&1; then
    ip addr flush dev eth0 2>/dev/null || true
    ip addr add 192.168.0.2/24 dev eth0 2>/dev/null || true
    ip link set eth0 up
    log_msg "eth0 configured with IP 192.168.0.2"
else
    log_msg "Warning: eth0 interface not found"
fi

# Enable IIO buffer channels for current sensing
log_msg "Enabling IIO buffer channels..."
echo 1 > /sys/bus/iio/devices/iio:device0/buffer0/in_current0_en
echo 1 > /sys/bus/iio/devices/iio:device0/buffer0/in_current1_en
echo 1 > /sys/bus/iio/devices/iio:device0/buffer0/in_current2_en
echo 1 > /sys/bus/iio/devices/iio:device0/buffer/enable
log_msg "IIO buffer channels enabled"

# Start subscriber application
log_msg "Starting subscriber application..."
cd "$SCRIPT_DIR"
python3 sub.py &
SUB_PID=$!
sleep 2

# Check if sub.py started successfully
if ! kill -0 "$SUB_PID" 2>/dev/null; then
    cleanup_and_exit "Failed to start sub.py"
fi
log_msg "Subscriber started with PID: $SUB_PID"
echo "$SUB_PID" > /tmp/motor_control_sub.pid
disown "$SUB_PID"

# Start Bokeh motor control application
log_msg "Starting Motor Control Bokeh application..."
bokeh serve --address 0.0.0.0 --allow-websocket-origin=* gui.py &
BOKEH_PID=$!
sleep 2

# Check if bokeh started successfully
if ! kill -0 "$BOKEH_PID" 2>/dev/null; then
    cleanup_and_exit "Failed to start Bokeh server"
fi
log_msg "Bokeh server started with PID: $BOKEH_PID"
echo "$BOKEH_PID" > /tmp/motor_control_bokeh.pid
disown "$BOKEH_PID"

log_msg "Motor Control startup complete"
log_msg "Access the web dashboard at http://192.168.0.2:5006"
