#!/bin/bash
### firstboot.sh – Run once to set up unique drone identity

MARKER="/etc/firstboot_done"

if [ -f "$MARKER" ]; then
    exit 0
fi

echo "[First Boot] Running unique setup..."

# --- Get Pi Serial Number ---
SERIAL=$(awk '/Serial/ {print $3}' /proc/cpuinfo)

# --- Create unique hostname ---
NEW_HOSTNAME="drone-${SERIAL: -6}" # last 6 chars of serial
echo "[First Boot] Setting hostname to $NEW_HOSTNAME"
echo "$NEW_HOSTNAME" >/etc/hostname
sed -i "s/127.0.1.1.*/127.0.1.1\t$NEW_HOSTNAME/" /etc/hosts
hostnamectl set-hostname "$NEW_HOSTNAME"

# --- Regenerate SSH keys ---
echo "[First Boot] Regenerating SSH keys..."
rm -f /etc/ssh/ssh_host_*
dpkg-reconfigure openssh-server

# --- Assign drone ID ---
echo "[First Boot] Assigning drone ID..."
echo "$SERIAL" >/etc/drone_id

# --- Marker so this doesn’t rerun ---
touch "$MARKER"

echo "[First Boot] Setup complete. Rebooting..."
reboot
