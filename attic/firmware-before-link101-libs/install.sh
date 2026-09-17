#!/bin/bash

# Installs the Axon udev rules on the ROS host (Raspberry Pi / Jetson).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Axon udev rules..."

sudo cp "$SCRIPT_DIR/99-axon-devices.rules" /etc/udev/rules.d/
echo "✓ Installed udev rules"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "✓ Reloaded udev rules"

echo ""
echo "Installation complete!"
echo ""
echo "A connected Axon board will appear as:"
echo "  /dev/axon-zenoh  - zenoh serial transport (point zenohd here)"
echo "  /dev/axon-lidar  - lidar UART passthrough (point rplidar_ros here)"
echo "  /dev/axon-debug  - debug log (init info, discovered IDs, status)"
