#!/bin/bash
################################################################################
# Install System Prerequisites
# Essential tools and libraries for Tank project
################################################################################

set -e

echo "Installing system prerequisites..."
echo ""

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install build essentials
echo ""
echo "Installing build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    software-properties-common

# Install Python and pip
echo ""
echo "Installing Python tools..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel

# Install USB tools
echo ""
echo "Installing USB and serial tools..."
sudo apt install -y \
    libusb-1.0-0-dev \
    usbutils \
    minicom \
    screen

# Install network tools
echo ""
echo "Installing network tools..."
sudo apt install -y \
    net-tools \
    tcpdump \
    network-manager \
    iputils-ping \
    traceroute

# Install Python packages
echo ""
echo "Installing Python packages..."
sudo pip3 install \
    pyserial

# Ensure .local/bin is in PATH
echo ""
echo "Configuring PATH..."
if ! grep -q '.local/bin' ~/.bashrc; then
    echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
    echo "Added ~/.local/bin to PATH in ~/.bashrc"
fi

echo ""
echo "✓ System prerequisites installed successfully"
echo ""
echo "Note: You may need to restart your terminal for PATH changes to take effect"

