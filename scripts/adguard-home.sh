#!/bin/bash

set -e

# --- CONFIG ---
# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADGUARD_FILENAME="AdGuardHome.yaml"
ADGUARD_CONFIG="$SCRIPT_DIR/$ADGUARD_FILENAME"

ADGUARD_DIR="/opt/AdGuardHome"
DOWNLOAD_URL="https://raw.githubusercontent.com/AdguardTeam/AdGuardHome/master/scripts/install.sh"

# Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
	echo "❌ Please run this script as root (use sudo)"
	exit 1
fi

# Check if AdGuardHome directory exists
if [ ! -d "$ADGUARD_DIR" ]; then
	echo "🔽 Downloading and installing AdGuard Home..."
	curl -sSL "$DOWNLOAD_URL" | sh -s -- -v
fi

echo "🔧 Ensuring dotfiles config exists..."
if [ ! -f "$ADGUARD_CONFIG" ]; then
	echo "❌ ERROR: $ADGUARD_CONFIG not found!"
	exit 1
fi

cp $ADGUARD_CONFIG "$ADGUARD_DIR/$ADGUARD_FILENAME"

systemctl stop systemd-resolved
systemctl disable systemd-resolved

rm /etc/resolv.conf
echo "nameserver 127.0.0.1" | tee /etc/resolv.conf

sleep 1

systemctl restart AdGuardHome.service

echo "✅ Done: AdGuardHome now uses your custom config!"
