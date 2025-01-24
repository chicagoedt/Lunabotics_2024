#!/bin/bash

# Bring up the CAN interface can0 at 1 Mbps bitrate
interface="can0"

# Check if the interface exists
if ! ip link show "$interface" &> /dev/null; then
    echo "Interface $interface does not exist."
    echo "Ensure your physical CAN adapter is connected."
    exit 1
fi

# Set the interface type and bitrate
if sudo ip link set "$interface" type can bitrate 1000000; then
    echo "Set $interface to CAN with 1 Mbps bitrate."
else
    echo "Failed to set $interface type and bitrate."
    exit 1
fi

# Bring up the interface
if sudo ip link set "$interface" up; then
    echo "Brought up $interface successfully."
else
    echo "Failed to bring up $interface."
    exit 1
fi

echo "CAN interface $interface setup complete."
