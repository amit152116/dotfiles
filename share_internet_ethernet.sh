
#!/bin/bash

# Automatically detect Ethernet and WiFi interfaces
ETH_INTERFACE=$(ip -o link show | awk -F': ' '$2 ~ /^e/{print $2; exit}')
WIFI_INTERFACE=$(ip -o link show | awk -F': ' '$2 ~ /^w/{print $2; exit}')

# Validate interface detection
if [ -z "$ETH_INTERFACE" ]; then
    echo "No Ethernet interface detected. Exiting..."
    exit 1
fi

if [ -z "$WIFI_INTERFACE" ]; then
    echo "No WiFi interface detected. Exiting..."
    exit 1
fi

echo "Detected Ethernet Interface: $ETH_INTERFACE"
echo "Detected WiFi Interface: $WIFI_INTERFACE"

# Find the laptop's Ethernet IP dynamically
ETH_IP=$(ip -4 addr show $ETH_INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

# Check if Ethernet is connected
if [ -z "$ETH_IP" ]; then
    echo "Ethernet ($ETH_INTERFACE) is not connected. Exiting..."
    exit 1
fi

echo "Ethernet IP: $ETH_IP"

# Enable IP forwarding (temporary)
echo "Enabling IP forwarding..."
sudo sysctl -w net.ipv4.ip_forward=1

# Set up NAT (Masquerading)
echo "Setting up NAT..."
sudo iptables -t nat -A POSTROUTING -o $WIFI_INTERFACE -j MASQUERADE
sudo iptables -A FORWARD -i $WIFI_INTERFACE -o $ETH_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i $ETH_INTERFACE -o $WIFI_INTERFACE -j ACCEPT

echo "Internet sharing enabled for Raspberry Pi!"

# Instructions for Raspberry Pi
echo "Run the following command on your Raspberry Pi to set the default gateway:\n"
echo " sudo route add default gw $ETH_IP eth0\n"
echo "If needed, set the DNS: \n"
echo " echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf\n"
