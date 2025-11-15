#!/bin/bash

# Task 1: Installation Script for Ubuntu 22.10 with ROS 2 Humble
# Adapted for older Ubuntu versions

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║     Task 1: Installation for Ubuntu 22.10 (ROS 2 Humble)    ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}ERROR: Please do not run this script as root${NC}"
    exit 1
fi

echo -e "${BLUE}Detected Ubuntu version: $(lsb_release -ds)${NC}"
echo ""

# Update system
echo -e "${BLUE}Step 1: Updating system...${NC}"
sudo apt update
echo -e "${GREEN}✓ System updated${NC}"
echo ""

# Install ROS 2 Humble (compatible with Ubuntu 22.10)
echo -e "${BLUE}Step 2: Installing ROS 2 Humble...${NC}"
if ! command -v ros2 &> /dev/null; then
    # Add ROS 2 repository
    sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install ros-humble-desktop ros-dev-tools -y
    echo -e "${GREEN}✓ ROS 2 Humble installed${NC}"
else
    echo -e "${YELLOW}⚠ ROS 2 already installed${NC}"
fi
echo ""

# Install Gazebo (try Gazebo Garden for better compatibility)
echo -e "${BLUE}Step 3: Installing Gazebo...${NC}"
if ! command -v gz &> /dev/null && ! command -v gazebo &> /dev/null; then
    # Try to install Gazebo Garden
    sudo apt install lsb-release wget gnupg -y
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg || true

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null || true

    sudo apt update

    # Try gz-garden first, fallback to gazebo classic
    sudo apt install gz-garden -y 2>/dev/null || sudo apt install gazebo11 -y || sudo apt install gazebo -y
    echo -e "${GREEN}✓ Gazebo installed${NC}"
else
    echo -e "${YELLOW}⚠ Gazebo already installed${NC}"
fi
echo ""

# Install ROS-Gazebo bridge
echo -e "${BLUE}Step 4: Installing ROS-Gazebo bridge...${NC}"
sudo apt install ros-humble-ros-gz -y 2>/dev/null || \
sudo apt install ros-humble-gazebo-ros-pkgs -y 2>/dev/null || \
echo -e "${YELLOW}⚠ Bridge packages may need manual installation${NC}"
echo ""

# Install TurtleBot3
echo -e "${BLUE}Step 5: Installing TurtleBot3 packages...${NC}"
sudo apt install ros-humble-turtlebot3* -y 2>/dev/null || \
echo -e "${YELLOW}⚠ Installing basic TurtleBot3 support${NC}"

sudo apt install ros-humble-teleop-twist-keyboard -y 2>/dev/null || \
sudo apt install python3-pip -y && pip3 install teleop-twist-keyboard || \
echo -e "${YELLOW}⚠ Teleop may need manual installation${NC}"
echo ""

# Configure environment
echo -e "${BLUE}Step 6: Configuring environment...${NC}"

BASHRC="$HOME/.bashrc"
BACKUP="$HOME/.bashrc.backup.$(date +%Y%m%d_%H%M%S)"

cp "$BASHRC" "$BACKUP"
echo "Backed up .bashrc to $BACKUP"

# Find ROS 2 installation
ROS_DISTRO=""
if [ -d "/opt/ros/humble" ]; then
    ROS_DISTRO="humble"
elif [ -d "/opt/ros/jazzy" ]; then
    ROS_DISTRO="jazzy"
elif [ -d "/opt/ros/iron" ]; then
    ROS_DISTRO="iron"
fi

if [ -n "$ROS_DISTRO" ]; then
    if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# ROS 2 $ROS_DISTRO" >> "$BASHRC"
        echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> "$BASHRC"
    fi
fi

if ! grep -q "TURTLEBOT3_MODEL" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# TurtleBot3" >> "$BASHRC"
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> "$BASHRC"
    echo "export ROS_DOMAIN_ID=30" >> "$BASHRC"
fi

if ! grep -q "GZ_SIM_RESOURCE_PATH\|GAZEBO_MODEL_PATH" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Gazebo models" >> "$BASHRC"
    echo "export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models:\$GZ_SIM_RESOURCE_PATH" >> "$BASHRC"
    echo "export GAZEBO_MODEL_PATH=\$HOME/.gazebo/models:\$GAZEBO_MODEL_PATH" >> "$BASHRC"
fi

echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Create model directories
echo -e "${BLUE}Step 7: Creating model directories...${NC}"
mkdir -p "$HOME/.gz/models"
mkdir -p "$HOME/.gazebo/models"
echo -e "${GREEN}✓ Directories created${NC}"
echo ""

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              Installation Complete (Adapted)!                ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}Dependencies installed for Ubuntu 22.10!${NC}"
echo ""
echo -e "${YELLOW}NOTE: This system uses ROS 2 Humble instead of Jazzy${NC}"
echo -e "${YELLOW}Some features may differ from the original task design${NC}"
echo ""
echo "Next steps:"
echo "  1. Open a NEW terminal"
echo "  2. cd ~/www/ros-robtics-tasks/tasks/task1"
echo "  3. Try: gz sim worlds/custom_world.sdf"
echo "     OR: gazebo worlds/custom_world.sdf"
echo ""
echo -e "${YELLOW}IMPORTANT: Open a new terminal for changes to take effect!${NC}"
echo ""
