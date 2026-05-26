#!/usr/bin/env bash
# Setup script for ez80dbg on Linux / macOS
set -e

VENV=venv

echo "=== ez80dbg setup ==="

# Check Python
if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found. Install Python 3.9+ first."
    exit 1
fi

# Create venv
if [ ! -d "$VENV" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV"
fi

echo "Installing dependencies..."
"$VENV/bin/pip" install --upgrade pip -q
"$VENV/bin/pip" install -r requirements.txt

echo ""
echo "=== Platform notes ==="

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Linux: ensure libusb-1.0 is installed:"
    echo "  sudo apt install libusb-1.0-0          # Debian/Ubuntu"
    echo "  sudo dnf install libusb1               # Fedora"
    echo ""
    echo "For non-root USB access add a udev rule:"
    echo "  echo 'ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6011\", MODE=\"0660\", GROUP=\"plugdev\"' \\"
    echo "    | sudo tee /etc/udev/rules.d/99-ftdi.rules"
    echo "  sudo udevadm control --reload-rules"
    echo "  sudo usermod -aG plugdev \$USER  # then log out and back in"
    echo ""
    echo "If pyftdi can't open the device, unload the kernel driver:"
    echo "  sudo modprobe -r ftdi_sio"
    echo "  (or add 'blacklist ftdi_sio' to /etc/modprobe.d/blacklist.conf)"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "macOS: ensure libusb is installed:"
    echo "  brew install libusb"
    echo ""
    echo "Apple Silicon (M1/M2): libusb from Homebrew is at /opt/homebrew/lib."
    echo "pyftdi finds it automatically."
fi

echo ""
echo "=== Done ==="
echo "Run the tool:"
echo "  $VENV/bin/python3 ez80dbg.py --help"
echo "  $VENV/bin/python3 ez80dbg.py flash firmware.hex"
echo "  $VENV/bin/python3 ez80dbg.py --gui"
