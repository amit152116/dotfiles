#!/bin/bash

set -e

echo "Downloading No-IP DUC..."
cd /usr/local/src
sudo wget --content-disposition https://www.noip.com/download/linux/latest
sudo tar xf noip-duc_3.3.0.tar.gz
cd /usr/local/src/noip-duc-3.3.0/binaries && sudo apt install ./noip-duc_3.3.0_amd64.deb
sudo rm -rf noip-duc_3.3.0.tar.gz
sudo rm -rf /usr/local/src/noip-duc-3.3.0
echo "No-IP DUC installed successfully."

echo "Creating systemd service for auto-start..."
sudo bash -c 'cat > /etc/systemd/system/noip-duc-custom.service <<EOF
[Unit]
Description=No-IP DUC Client (Custom CLI)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/bin/noip-duc \
  -g all.ddnskey.com \
  --username <DDNS key Username> \
  --password <DDNS key Password> 
Restart=on-failure
User=root

[Install]
WantedBy=multi-user.target
EOF'

sudo systemctl daemon-reload
sudo systemctl enable --now noip-duc-custom.service
sudo systemctl status noip-duc-custom.service

echo
echo "✅ No-IP DDNS client installed and running."
echo "🔁 It will auto-update your public IP to your No-IP hostname."
