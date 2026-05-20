#!/bin/bash
# Motor Control BLDC Stop Script
# Stops Python applications and disables IIO buffers

LOG_FILE="/var/log/motor_control.log"

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log_msg "Stopping Motor Control BLDC..."

# Stop Bokeh server
if [ -f /tmp/motor_control_bokeh.pid ]; then
    BOKEH_PID=$(cat /tmp/motor_control_bokeh.pid)
    if kill -0 "$BOKEH_PID" 2>/dev/null; then
        log_msg "Stopping Bokeh server (PID: $BOKEH_PID)..."
        kill "$BOKEH_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$BOKEH_PID" 2>/dev/null || true
    fi
    rm -f /tmp/motor_control_bokeh.pid
fi

# Stop subscriber
if [ -f /tmp/motor_control_sub.pid ]; then
    SUB_PID=$(cat /tmp/motor_control_sub.pid)
    if kill -0 "$SUB_PID" 2>/dev/null; then
        log_msg "Stopping subscriber (PID: $SUB_PID)..."
        kill "$SUB_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$SUB_PID" 2>/dev/null || true
    fi
    rm -f /tmp/motor_control_sub.pid
fi

# Kill any remaining bokeh/sub.py processes
log_msg "Cleaning up remaining processes..."
pkill -f "bokeh serve.*gui.py" 2>/dev/null || true
pkill -f "python.*sub.py" 2>/dev/null || true

# Disable IIO buffer first (must be done before disabling channels)
log_msg "Disabling IIO buffers..."
echo 0 > /sys/bus/iio/devices/iio:device0/buffer/enable 2>/dev/null || true

# Disable IIO buffer channels
echo 0 > /sys/bus/iio/devices/iio:device0/buffer0/in_current0_en 2>/dev/null || true
echo 0 > /sys/bus/iio/devices/iio:device0/buffer0/in_current1_en 2>/dev/null || true
echo 0 > /sys/bus/iio/devices/iio:device0/buffer0/in_current2_en 2>/dev/null || true
log_msg "IIO buffer channels disabled"

# Restore default kernel log level
dmesg -n 7

log_msg "Motor Control BLDC stopped"
