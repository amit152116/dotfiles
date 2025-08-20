#!/bin/bash

# Script to stop internet connection sharing
# Usage: ./stop_share_internet_ethernet.sh [source_interface] [target_interface]

# Error handling function
handle_error() {
    echo "ERROR: $1" >&2
    exit 1
}

# Check if running with sudo/root privileges
if [ "$EUID" -ne 0 ]; then
    handle_error "This script requires root privileges. Please run with sudo."
fi

# Get interfaces from command line arguments or detect automatically
SOURCE_INTERFACE=$1
TARGET_INTERFACE=$2
# TARGET_INTERFACE=${2:-$(ip -o link show | awk -F': ' '$2 ~ /^e/{print $2; exit}')}

# Check if interfaces exist
if [ -n "$SOURCE_INTERFACE" ] && ! ip link show "$SOURCE_INTERFACE" &>/dev/null; then
    handle_error "Source interface $SOURCE_INTERFACE does not exist."
fi

if [ -n "$TARGET_INTERFACE" ] && ! ip link show "$TARGET_INTERFACE" &>/dev/null; then
    handle_error "Target interface $TARGET_INTERFACE does not exist."
fi

# Log the interfaces we're working with
echo "Source interface (internet-connected): $SOURCE_INTERFACE"
echo "Target interface (shared internet): $TARGET_INTERFACE"

# Disable IP forwarding
echo "Disabling IP forwarding..."
if ! sudo sysctl -w net.ipv4.ip_forward=0 &>/dev/null; then
    echo "WARNING: Failed to disable IP forwarding."
fi

# Flush NAT and forwarding rules
echo "Clearing IP tables rules..."
sudo iptables -t nat -D POSTROUTING -o $SOURCE_INTERFACE -j MASQUERADE 2>/dev/null || echo "Note: NAT masquerading rule may not have been present."
sudo iptables -D FORWARD -i $SOURCE_INTERFACE -o $TARGET_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || echo "Note: Forward rule (source to target) may not have been present."
sudo iptables -D FORWARD -i $TARGET_INTERFACE -o $SOURCE_INTERFACE -j ACCEPT 2>/dev/null || echo "Note: Forward rule (target to source) may not have been present."

echo "Internet sharing disabled."
