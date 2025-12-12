#!/bin/bash
# Setup DHCP server on Jetson for WiFi clients
# This allows the Xiaomi router (in AP mode) to provide network access

set -e

echo "=========================================="
echo "Tank Rover - DHCP Server Setup"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo"
    exit 1
fi

echo "Installing dnsmasq (DHCP/DNS server)..."
apt-get update
apt-get install -y dnsmasq

echo ""
echo "Backing up original configuration..."
if [ ! -f /etc/dnsmasq.conf.backup ]; then
    cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup
    echo "✓ Backup created at /etc/dnsmasq.conf.backup"
else
    echo "✓ Backup already exists"
fi

echo ""
echo "Creating dnsmasq configuration..."
cat > /etc/dnsmasq.conf << 'EOF'
# Tank Rover DHCP Server Configuration
# Interface to listen on
interface=eno1
bind-interfaces

# DHCP range for WiFi clients (150-200)
# Reserves 100-149 for static assignments
dhcp-range=192.168.2.150,192.168.2.200,255.255.255.0,24h

# Static IP reservations
# LiDAR Front
dhcp-host=0c:29:ab:7c:00:01,192.168.2.62,lidar-front,infinite
# LiDAR Rear (same MAC, different IP - configured in LiDAR)
# dhcp-host=0c:29:ab:7c:00:01,192.168.2.63,lidar-rear,infinite

# Gateway (Jetson IP for internet sharing)
dhcp-option=3,192.168.2.100

# DNS servers (Google DNS)
dhcp-option=6,8.8.8.8,8.8.4.4

# Domain name
domain=tank.local
local=/tank.local/

# Expand simple names
expand-hosts

# Enable logging (helpful for debugging)
log-dhcp
log-queries

# Cache size
cache-size=1000

# Don't read /etc/resolv.conf or /etc/hosts
no-resolv
no-hosts

# Add custom hosts
addn-hosts=/etc/dnsmasq.hosts
EOF

echo "✓ Configuration created"

echo ""
echo "Creating custom hosts file..."
cat > /etc/dnsmasq.hosts << 'EOF'
# Tank Rover Network Hosts
192.168.2.100   jetson jetson.tank.local
192.168.2.1     router router.tank.local
192.168.2.62    lidar-front lidar-front.tank.local
192.168.2.63    lidar-rear lidar-rear.tank.local
EOF

echo "✓ Hosts file created"

echo ""
echo "Enabling IP forwarding for internet sharing..."
sysctl -w net.ipv4.ip_forward=1

# Make it permanent
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf; then
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
    echo "✓ IP forwarding enabled permanently"
else
    echo "✓ IP forwarding already configured"
fi

echo ""
echo "Configuring NAT (Network Address Translation)..."

# Install iptables-persistent for saving rules
DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent

# Clear existing NAT rules
iptables -t nat -F
iptables -F FORWARD

# Add NAT rules for internet sharing
# Forward traffic from lidar network (eno1) to WiFi (wlP1p1s0)
iptables -t nat -A POSTROUTING -o wlP1p1s0 -j MASQUERADE
iptables -A FORWARD -i eno1 -o wlP1p1s0 -j ACCEPT
iptables -A FORWARD -i wlP1p1s0 -o eno1 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Save rules
netfilter-persistent save

echo "✓ NAT configured and saved"

echo ""
echo "Starting dnsmasq service..."
systemctl enable dnsmasq
systemctl restart dnsmasq

# Wait a moment for service to start
sleep 2

# Check status
if systemctl is-active --quiet dnsmasq; then
    echo "✓ dnsmasq is running"
else
    echo "✗ dnsmasq failed to start"
    echo "Check logs with: sudo journalctl -u dnsmasq -n 50"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ DHCP Server Setup Complete!"
echo "=========================================="
echo ""
echo "Configuration Summary:"
echo "  Interface:        eno1"
echo "  Jetson IP:        192.168.2.100"
echo "  DHCP Range:       192.168.2.150 - 192.168.2.200"
echo "  Gateway:          192.168.2.100 (Jetson)"
echo "  DNS Servers:      8.8.8.8, 8.8.4.4"
echo "  Domain:           tank.local"
echo ""
echo "Static Reservations:"
echo "  LiDAR Front:      192.168.2.62"
echo "  LiDAR Rear:       192.168.2.63"
echo "  Router:           192.168.2.1"
echo ""
echo "Next Steps:"
echo "1. Configure Xiaomi router as Access Point"
echo "   - Set router IP: 192.168.2.1"
echo "   - Disable router's DHCP server"
echo "   - Enable WiFi with SSID: Tank_Rover_AP"
echo ""
echo "2. Connect WiFi clients to Tank_Rover_AP"
echo "   - Clients will receive IP: 192.168.2.150-200"
echo "   - Can access Jetson at: 192.168.2.100"
echo "   - Can access LiDARs at: .62 and .63"
echo ""
echo "Useful Commands:"
echo "  sudo systemctl status dnsmasq     # Check DHCP status"
echo "  sudo journalctl -u dnsmasq -f     # View live logs"
echo "  cat /var/lib/misc/dnsmasq.leases  # View DHCP leases"
echo "  sudo systemctl restart dnsmasq    # Restart service"
echo ""
echo "See docs/setup/XIAOMI_ROUTER_AP_SETUP.md for detailed guide"
echo ""

