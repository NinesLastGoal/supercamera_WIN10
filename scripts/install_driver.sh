#!/bin/bash
#
# SuperCamera Driver Installation Script
# Installs the USB kernel driver for UseePlus/SuperCamera borescope cameras
#
# Supported: Ubuntu 18.04, 20.04, 22.04, 24.04
#

set -e

echo "=== SuperCamera Driver Installation ==="
echo ""

# Check for root
if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo $0"
    exit 1
fi

# Get script directory (works even if called from different location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVER_DIR="$SCRIPT_DIR/../driver"

# Check if driver source exists
if [ ! -f "$DRIVER_DIR/supercamera.c" ]; then
    # Try to find it relative to ROS package
    DRIVER_DIR="$(ros2 pkg prefix supercamera_ros 2>/dev/null)/share/supercamera_ros/driver" || true
    if [ ! -f "$DRIVER_DIR/supercamera.c" ]; then
        echo "ERROR: Cannot find driver source files"
        echo "Make sure supercamera_ros package is built: colcon build"
        exit 1
    fi
fi

KERNEL_VER=$(uname -r)
DRIVER_NAME="supercamera"
DRIVER_VERSION="1.0"

echo "[1/5] Installing build dependencies..."
apt-get update -qq
apt-get install -y -qq build-essential linux-headers-${KERNEL_VER} dkms

echo "[2/5] Removing old driver (if present)..."
modprobe -r supercamera 2>/dev/null || true
modprobe -r supercamera_simple 2>/dev/null || true
dkms remove -m ${DRIVER_NAME} -v ${DRIVER_VERSION} --all 2>/dev/null || true
rm -rf /usr/src/${DRIVER_NAME}-* 2>/dev/null || true

echo "[3/5] Installing driver via DKMS..."
SRC_DIR="/usr/src/${DRIVER_NAME}-${DRIVER_VERSION}"
mkdir -p "$SRC_DIR"
cp "$DRIVER_DIR/supercamera.c" "$SRC_DIR/"
cp "$DRIVER_DIR/Makefile" "$SRC_DIR/"

# Create DKMS config
cat > "$SRC_DIR/dkms.conf" << EOF
PACKAGE_NAME="${DRIVER_NAME}"
PACKAGE_VERSION="${DRIVER_VERSION}"
BUILT_MODULE_NAME[0]="${DRIVER_NAME}"
DEST_MODULE_LOCATION[0]="/updates"
AUTOINSTALL="yes"
MAKE[0]="make -C /lib/modules/\${kernelver}/build M=\${dkms_tree}/${DRIVER_NAME}/${DRIVER_VERSION}/build modules"
CLEAN="make -C /lib/modules/\${kernelver}/build M=\${dkms_tree}/${DRIVER_NAME}/${DRIVER_VERSION}/build clean"
EOF

dkms add -m "$DRIVER_NAME" -v "$DRIVER_VERSION"
dkms build -m "$DRIVER_NAME" -v "$DRIVER_VERSION"
dkms install -m "$DRIVER_NAME" -v "$DRIVER_VERSION"

echo "[4/5] Setting up udev rules..."
cat > /etc/udev/rules.d/99-supercamera.rules << EOF
# SuperCamera USB device permissions
SUBSYSTEM=="usb", ATTR{idVendor}=="2ce3", ATTR{idProduct}=="3828", MODE="0666"
KERNEL=="supercamera", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "[5/5] Loading driver..."
modprobe supercamera

# Add to auto-load
if ! grep -q "^supercamera$" /etc/modules 2>/dev/null; then
    echo "supercamera" >> /etc/modules
fi

echo ""
echo "=== Installation Complete ==="
echo ""

# Check if device is present
if [ -c /dev/supercamera ]; then
    chmod 666 /dev/supercamera
    echo "SUCCESS: Camera detected at /dev/supercamera"
else
    echo "NOTE: Camera not detected. Plug in the camera and it will appear at /dev/supercamera"
fi

echo ""
echo "Usage:"
echo "  ros2 run supercamera_ros publisher"
echo "  ros2 run supercamera_ros publisher --ros-args -p topic:=/my/topic"
echo ""
