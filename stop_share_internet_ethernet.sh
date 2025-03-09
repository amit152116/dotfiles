#!/bin/bash

ETH_INTERFACE=$(ip -o link show | awk -F': ' '$2 ~ /^e/{print $2; exit}')

# Validate interface detection
if [ -z "$ETH_INTERFACE" ]; then
    echo "No Ethernet interface detected. Exiting..."
    exit 1
fi

echo "Detected Ethernet Interface: $ETH_INTERFACE"

# Disable IP forwarding
echo "Disabling IP forwarding..."
sudo sysctl -w net.ipv4.ip_forward=0

# Flush NAT and forwarding rules
echo "Clearing IP tables..."
sudo iptables -t nat -F
sudo iptables -F

echo "Internet sharing disabled."

