#!/bin/sh
# Load motor control IIO kernel modules in required order
# This script ensures proper initialization sequence for motor control hardware

set -e

MODULES="mpfs_mc_adc mpfs_mc_picon mpfs_mc_pwm mpfs_mc_ratelim mpfs_mc_sqmng mpfs_mc_stptheta"

load_modules() {
    for mod in $MODULES; do
        if ! lsmod | grep -q "^${mod}"; then
            echo "Loading $mod..."
            modprobe "$mod"
        else
            echo "$mod already loaded"
        fi
    done
    echo "All motor control modules loaded"
}

unload_modules() {
    # Unload in reverse order
    for mod in mpfs_mc_stptheta mpfs_mc_sqmng mpfs_mc_ratelim mpfs_mc_pwm mpfs_mc_picon mpfs_mc_adc; do
        if lsmod | grep -q "^${mod}"; then
            echo "Unloading $mod..."
            rmmod "$mod"
        fi
    done
    echo "All motor control modules unloaded"
}

status_modules() {
    echo "Motor control module status:"
    for mod in $MODULES; do
        if lsmod | grep -q "^${mod}"; then
            echo "  $mod: loaded"
        else
            echo "  $mod: not loaded"
        fi
    done
}

case "${1:-load}" in
    load)
        load_modules
        ;;
    unload)
        unload_modules
        ;;
    reload)
        unload_modules
        sleep 1
        load_modules
        ;;
    status)
        status_modules
        ;;
    *)
        echo "Usage: $0 {load|unload|reload|status}"
        exit 1
        ;;
esac
