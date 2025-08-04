#!/bin/bash

# Setup script for Ubuntu development environment

set -e

echo "Setting up development environment..."

# Update package lists
sudo apt-get update

# Install essential development tools
sudo apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    unzip \
    software-properties-common \
    udev \
    python3-pip \
    python3-venv \
    pkg-config \
    libusb-1.0-0-dev \
    libudev-dev

# Install PlatformIO
echo "Installing PlatformIO..."
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade platformio

# Add PlatformIO to PATH
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc

# Install PlatformIO udev rules for device access
echo "Setting up udev rules for device access..."
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# Create a symlink for common Arduino libraries location
mkdir -p ~/Arduino/libraries

# Install additional Python packages that might be useful
python3 -m pip install --upgrade \
    esptool \
    adafruit-ampy \
    pyserial

# Install Gemini CLI
echo "Installing Gemini CLI..."
curl -sS https://storage.googleapis.com/gemini-cli/install.sh | bash

# Set up Git configuration (if not already configured)
if [ -z "$(git config --global user.name)" ]; then
    echo "Git user name not configured. You may want to run:"
    echo "git config --global user.name 'Your Name'"
    echo "git config --global user.email 'your.email@example.com'"
fi

echo "✅ Development environment setup complete!"
echo ""
echo "🚀 You can now:"
echo "  - Use 'pio' commands for PlatformIO operations"
echo "  - Use 'node' and 'npm' for JavaScript/TypeScript development"
echo "  - Develop Arduino/ESP32 projects with IntelliSense"
echo ""
echo "📋 Next steps:"
echo "  1. Run 'pio run' to build the project"
echo "  2. Run 'pio device list' to see connected devices"
echo "  3. Use 'pio run --target upload' to flash firmware"
echo ""
echo "🔧 Hardware debugging:"
echo "  - USB devices are mounted with --privileged access"
echo "  - Serial ports should be accessible in /dev/"
