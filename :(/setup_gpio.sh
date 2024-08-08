#!/bin/bash

# Script to set up GPIO access on Raspberry Pi 3B+ running Ubuntu 22.04 server

# Exit on any error
set -e

echo "Setting up GPIO access for Raspberry Pi 3B+ on Ubuntu 22.04 server"

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Install necessary packages
sudo apt install -y python3-pip python3-dev python3-setuptools

# Install RPi.GPIO from pip
sudo pip3 install RPi.GPIO

# Add user to gpio group
sudo usermod -a -G gpio $USER

# Create a udev rule to allow gpio group to access /dev/gpiomem
echo 'SUBSYSTEM=="bcm2835-gpiomem", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/90-gpio.rules

# Load the gpio module
sudo modprobe gpio

# Add gpio to modules to load at boot
echo "gpio" | sudo tee -a /etc/modules

# Create a script to set GPIO permissions at boot
cat << EOF | sudo tee /etc/rc.local
#!/bin/sh -e
# Give gpio group access to /dev/gpiomem
chown root:gpio /dev/gpiomem
chmod g+rw /dev/gpiomem
exit 0
EOF

# Make rc.local executable
sudo chmod +x /etc/rc.local

# Enable rc-local service
sudo systemctl enable rc-local

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Setup complete. Please reboot your Raspberry Pi for changes to take effect."
echo "After reboot, try running your GPIO scripts again."
