#!/bin/bash

# Disable IPv6 for new interfaces at the default level before overlay probe,
# so that when CoreTSE interfaces appear they won't generate a link-local
# address from the initial (duplicate) MAC.
sysctl -qw net.ipv6.conf.default.disable_ipv6=1

if [ "$1" = "mc" ]; then
  overlay_file="mpfs_motor_control_bldc_tsn.dtbo"
else
  overlay_file="mpfs_coretse.dtbo"
fi

cd /boot/

if ! /opt/microchip/dt-overlays/overlay.sh "$overlay_file"; then
  echo "Error: Failed to apply overlay '$overlay_file' — network interfaces may not be available" >&2
  exit 1
fi

# Restore default IPv6 behavior for future interfaces
sysctl -qw net.ipv6.conf.default.disable_ipv6=0

# Derive CoreTSE MAC addresses from the GEM base MAC (serial number based).
# U-Boot sets eth0 (GEM ethernet@20112000) MAC to 00:04:A3:serial[2]:serial[1]:serial[0].
# CoreTSE ports get serial[0]+2, +3, +4 as the last octet.
set_coretse_mac() {
  local base_mac
  base_mac=$(cat /sys/class/net/eth0/address 2>/dev/null)
  if [ -z "$base_mac" ]; then
    echo "Warning: Cannot read eth0 MAC address, skipping CoreTSE MAC fixup"
    return
  fi

  # Split MAC into octets
  local o1 o2 o3 o4 o5 o6
  IFS=':' read -r o1 o2 o3 o4 o5 o6 <<< "$base_mac"

  # Convert last octet to decimal and compute offsets
  local last_dec
  last_dec=$((16#$o6))

  local ifaces="eth1 eth2 eth3"
  local offset=2
  for iface in $ifaces; do
    if [ -d "/sys/class/net/$iface" ]; then
      local new_last
      new_last=$(printf "%02x" $(( (last_dec + offset) & 0xFF )))
      local new_mac="${o1}:${o2}:${o3}:${o4}:${o5}:${new_last}"
      # Disable IPv6 before MAC change to prevent duplicate link-local address
      sysctl -qw "net.ipv6.conf.${iface}.disable_ipv6=1"
      ip link set dev "$iface" down 2>/dev/null
      ip link set dev "$iface" address "$new_mac"
      # Re-enable IPv6 so link-local is derived from the new MAC
      sysctl -qw "net.ipv6.conf.${iface}.disable_ipv6=0"
      ip link set dev "$iface" up
      echo "Set $iface MAC to $new_mac"
    fi
    offset=$((offset + 1))
  done
}

# Set MAC addresses for CoreTSE interfaces based on device serial number
set_coretse_mac
 
# Function to simulate progress
simulate_progress() {
  local task_name=$1
  echo -n "$task_name..."
  for i in {1..10}; do
    sleep 0.3
    printf "\r%s [%02d%%]" "$task_name..." $((i * 10))
  done
  echo
}
 
# Simulating configurations
simulate_progress "Configuring CoreTSE"
simulate_progress "Configuring TSN"
if [ "$1" != "mc" ]; then
  simulate_progress "Configuring Mikrobus Controller"
fi
 
# TSN Demo configuration PCP 5 for VLAN 13 (not needed for Motor Control Kit)
if [ "$1" != "mc" ]; then
  ip link add link eth1 name eth1.13 type vlan id 13 egress-qos-map 0:5
  ip link set dev eth1.13 up
  ip addr add 192.168.13.2/24 dev eth1.13
  ifconfig eth1.13 mtu 1400
  cd /opt/microchip/japll-pi-controller/ && ./japll-pi > /dev/null 2>&1 &
fi
