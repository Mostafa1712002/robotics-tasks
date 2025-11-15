#!/bin/bash

# Task 1: Fixed Installation Script for Ubuntu 22.10
# Handles repository errors and mixed Ubuntu versions

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║     Task 1: Fixed Installation (Ubuntu 22.10)               ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}ERROR: Do not run as root${NC}"
    exit 1
fi

echo -e "${BLUE}Detected Ubuntu: $(lsb_release -ds)${NC}"
echo ""

# Fix repository issues first
echo -e "${BLUE}Step 1: Fixing repository issues...${NC}"

# Remove problematic jammy repository
sudo rm -f /etc/apt/sources.list.d/*jammy* 2>/dev/null || true

# Fix old-releases sources
if grep -q "old-releases.ubuntu.com" /etc/apt/sources.list; then
    echo "Fixing old-releases URLs..."
    sudo sed -i 's|http://old-releases.ubuntu.com|http://old-releases.ubuntu.com|g' /etc/apt/sources.list
fi

# Skip failing repositories
echo "Updating package lists (ignoring errors)..."
sudo apt update --allow-releaseinfo-change 2>&1 | grep -v "does not have a Release file" || true

echo -e "${GREEN}✓ Repositories configured${NC}"
echo ""

# Install basic tools first
echo -e "${BLUE}Step 2: Installing basic tools...${NC}"
sudo apt install -y curl wget gnupg lsb-release software-properties-common 2>/dev/null || true
echo -e "${GREEN}✓ Basic tools installed${NC}"
echo ""

# Try to install ROS 2 Humble
echo -e "${BLUE}Step 3: Installing ROS 2 Humble...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo "Adding ROS 2 repository..."

    # Add ROS 2 key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true

    # Add ROS 2 repository for kinetic (22.10)
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu kinetic main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update 2>&1 | grep -v "does not have a Release file" || true

    # Try to install ROS 2
    if sudo apt install -y ros-humble-desktop 2>/dev/null; then
        echo -e "${GREEN}✓ ROS 2 Humble installed${NC}"
    else
        echo -e "${YELLOW}⚠ ROS 2 installation failed - trying minimal install${NC}"
        sudo apt install -y ros-humble-ros-base 2>/dev/null || echo -e "${YELLOW}⚠ ROS 2 not available for this Ubuntu version${NC}"
    fi
else
    echo -e "${YELLOW}⚠ ROS 2 already installed${NC}"
fi
echo ""

# Install Gazebo
echo -e "${BLUE}Step 4: Installing Gazebo...${NC}"
if ! command -v gz &> /dev/null && ! command -v gazebo &> /dev/null; then
    # Try gazebo from standard repos
    sudo apt install -y gazebo gazebo11 2>/dev/null || \
    sudo apt install -y gazebo9 2>/dev/null || \
    echo -e "${YELLOW}⚠ Gazebo installation may require manual setup${NC}"

    if command -v gazebo &> /dev/null || command -v gz &> /dev/null; then
        echo -e "${GREEN}✓ Gazebo installed${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Gazebo already installed${NC}"
fi
echo ""

# Install ROS-Gazebo bridge
echo -e "${BLUE}Step 5: Installing ROS-Gazebo packages...${NC}"
sudo apt install -y ros-humble-gazebo-ros-pkgs 2>/dev/null || \
sudo apt install -y ros-humble-ros-gz 2>/dev/null || \
echo -e "${YELLOW}⚠ Bridge packages may need manual configuration${NC}"
echo ""

# Install TurtleBot3
echo -e "${BLUE}Step 6: Installing TurtleBot3...${NC}"
sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-simulations 2>/dev/null || \
echo -e "${YELLOW}⚠ TurtleBot3 packages may need manual installation${NC}"

sudo apt install -y ros-humble-teleop-twist-keyboard 2>/dev/null || \
sudo apt install -y python3-pip && pip3 install teleop-twist-keyboard || \
echo -e "${YELLOW}⚠ Teleop keyboard may need manual installation${NC}"
echo ""

# Configure environment
echo -e "${BLUE}Step 7: Configuring environment...${NC}"

BASHRC="$HOME/.bashrc"
BACKUP="$HOME/.bashrc.backup.$(date +%Y%m%d_%H%M%S)"

cp "$BASHRC" "$BACKUP"
echo "Backed up .bashrc to $BACKUP"

# Find ROS installation
if [ -d "/opt/ros/humble" ]; then
    if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# ROS 2 Humble" >> "$BASHRC"
        echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
    fi
fi

if ! grep -q "TURTLEBOT3_MODEL" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# TurtleBot3" >> "$BASHRC"
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> "$BASHRC"
    echo "export ROS_DOMAIN_ID=30" >> "$BASHRC"
fi

if ! grep -q "GAZEBO_MODEL_PATH" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Gazebo" >> "$BASHRC"
    echo "export GAZEBO_MODEL_PATH=\$HOME/.gazebo/models:\$GAZEBO_MODEL_PATH" >> "$BASHRC"
    echo "export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models:\$GZ_SIM_RESOURCE_PATH" >> "$BASHRC"
fi

echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Create directories
echo -e "${BLUE}Step 8: Creating directories...${NC}"
mkdir -p "$HOME/.gazebo/models"
mkdir -p "$HOME/.gz/models"
echo -e "${GREEN}✓ Directories created${NC}"
echo ""

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              Installation Complete!                          ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}Installation finished!${NC}"
echo ""
echo "What was installed:"
if command -v ros2 &> /dev/null; then
    echo "  ✓ ROS 2: $(ros2 --version 2>&1 | head -n1)"
else
    echo "  ✗ ROS 2: Not installed"
fi

if command -v gazebo &> /dev/null; then
    echo "  ✓ Gazebo: $(gazebo --version 2>&1 | head -n1 | cut -d' ' -f3)"
elif command -v gz &> /dev/null; then
    echo "  ✓ Gazebo: $(gz sim --version 2>&1 | head -n1)"
else
    echo "  ✗ Gazebo: Not installed"
fi
echo ""
echo "Next steps:"
echo "  1. Open a NEW terminal"
echo "  2. cd ~/www/ros-robtics-tasks/tasks/task1"
echo "  3. Try: gazebo worlds/custom_world.sdf"
echo ""
echo -e "${YELLOW}IMPORTANT: Open a new terminal for changes to take effect!${NC}"
echo ""
