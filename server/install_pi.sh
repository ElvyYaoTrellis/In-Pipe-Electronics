#!/bin/bash
# Install all dependencies for rtsp_dual.py + 2Dac2Motor_network.py on Raspberry Pi Zero 2W
set -e

echo "==> Updating apt..."
sudo apt-get update

echo "==> Installing GStreamer + RTSP server..."
sudo apt-get install -y \
    python3-gi \
    python3-gst-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-rtsp \
    gir1.2-gst-rtsp-server-1.0 \
    libgstrtspserver-1.0-dev

echo "==> Installing Python packages..."
sudo apt-get install -y \
    python3-pip \
    python3-smbus \
    i2c-tools

pip3 install --break-system-packages \
    aiohttp \
    smbus2

echo "==> Enabling I2C and SPI..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0

echo "==> Adding user to i2c + video groups..."
sudo usermod -aG i2c,video,gpio "$USER"

echo ""
echo "Done. Reboot for group changes to take effect."
echo "Run the server with: python3 rtsp_dual.py"
