#!/bin/bash

# Script to share internet connection from one interface to another
# Usage: ./share_internet_ethernet.sh [source_interface] [target_interface]
# Example: ./share_internet_ethernet.sh wlan0 eth0

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

# Validate interface detection
if [ -z "$SOURCE_INTERFACE" ]; then
    handle_error "No source interface (internet-connected) detected."
fi

# Check if source interface exists
if ! ip link show "$SOURCE_INTERFACE" &>/dev/null; then
    handle_error "Source interface $SOURCE_INTERFACE does not exist."
fi

if [ -z "$TARGET_INTERFACE" ]; then
    handle_error "No target interface (to share internet to) detected."
fi

# Check if target interface exists
if ! ip link show "$TARGET_INTERFACE" &>/dev/null; then
    handle_error "Target interface $TARGET_INTERFACE does not exist."
fi

echo "Source interface (internet-connected): $SOURCE_INTERFACE"
echo "Target interface (to share internet to): $TARGET_INTERFACE"

# Check if source interface is connected to internet
if ! ping -c 1 -W 2 -I "$SOURCE_INTERFACE" 8.8.8.8 &>/dev/null; then
    echo "WARNING: Source interface $SOURCE_INTERFACE may not have internet connectivity."
fi

# Find the target interface IP dynamically
TARGET_IP=$(ip -4 addr show $TARGET_INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

# Check if target interface has an IP
if [ -z "$TARGET_IP" ]; then
    handle_error "Target interface ($TARGET_INTERFACE) is not configured with an IP."
fi

echo "Target interface IP: $TARGET_IP"

# Enable IP forwarding (temporary)
echo "Enabling IP forwarding..."
if ! sudo sysctl -w net.ipv4.ip_forward=1 &>/dev/null; then
    handle_error "Failed to enable IP forwarding."
fi

# Set up NAT (Masquerading)
echo "Setting up NAT..."
if ! sudo iptables -t nat -A POSTROUTING -o $SOURCE_INTERFACE -j MASQUERADE; then
    handle_error "Failed to set up NAT masquerading rule."
fi

if ! sudo iptables -A FORWARD -i $SOURCE_INTERFACE -o $TARGET_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT; then
    handle_error "Failed to set up forward rule from source to target."
fi

if ! sudo iptables -A FORWARD -i $TARGET_INTERFACE -o $SOURCE_INTERFACE -j ACCEPT; then
    handle_error "Failed to set up forward rule from target to source."
fi

echo "Internet sharing enabled!"

# Instructions for devices connecting to the target interface
echo "Run the following command on devices connecting to $TARGET_INTERFACE to set the default gateway:"
echo " sudo route add default gw $TARGET_IP"
echo "If needed, set the DNS:"
echo " echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf"
