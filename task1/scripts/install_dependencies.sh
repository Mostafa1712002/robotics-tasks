#!/bin/bash

# Task 1: Automated Installation Script
# This script installs all required dependencies for Task 1

set -e  # Exit on error

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║           Task 1: Dependency Installation Script            ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}ERROR: Please do not run this script as root or with sudo${NC}"
    echo "The script will ask for sudo password when needed"
    exit 1
fi

# Check Ubuntu version
echo -e "${BLUE}Checking Ubuntu version...${NC}"
UBUNTU_VERSION=$(lsb_release -cs)
if [ "$UBUNTU_VERSION" != "noble" ]; then
    echo -e "${RED}ERROR: This script requires Ubuntu 24.04 (Noble)${NC}"
    echo "Current version: $UBUNTU_VERSION"
    exit 1
fi
echo -e "${GREEN}✓ Ubuntu 24.04 detected${NC}"
echo ""

# Update package lists
echo -e "${BLUE}Step 1: Updating package lists...${NC}"
sudo apt update
echo -e "${GREEN}✓ Package lists updated${NC}"
echo ""

# Install ROS 2 Jazzy
echo -e "${BLUE}Step 2: Installing ROS 2 Jazzy...${NC}"
echo "This may take several minutes..."

# Add ROS 2 repository
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    echo "Adding ROS 2 repository..."
    sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

# Install ROS 2
if ! command -v ros2 &> /dev/null; then
    sudo apt install ros-jazzy-desktop ros-dev-tools -y
    echo -e "${GREEN}✓ ROS 2 Jazzy installed${NC}"
else
    echo -e "${YELLOW}⚠ ROS 2 already installed${NC}"
fi
echo ""

# Install Gazebo Jetty
echo -e "${BLUE}Step 3: Installing Gazebo Jetty...${NC}"

# Add Gazebo repository
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    echo "Adding Gazebo repository..."
    sudo apt install lsb-release wget gnupg -y
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update
fi

# Install Gazebo
if ! command -v gz &> /dev/null; then
    sudo apt install gz-jetty -y
    echo -e "${GREEN}✓ Gazebo Jetty installed${NC}"
else
    echo -e "${YELLOW}⚠ Gazebo already installed${NC}"
fi
echo ""

# Install ROS-Gazebo bridge
echo -e "${BLUE}Step 4: Installing ROS-Gazebo bridge...${NC}"
sudo apt install ros-jazzy-ros-gz -y
echo -e "${GREEN}✓ ROS-Gazebo bridge installed${NC}"
echo ""

# Install TurtleBot3 packages
echo -e "${BLUE}Step 5: Installing TurtleBot3 packages...${NC}"
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-teleop-twist-keyboard -y
echo -e "${GREEN}✓ TurtleBot3 packages installed${NC}"
echo ""

# Configure environment
echo -e "${BLUE}Step 6: Configuring environment...${NC}"

BASHRC="$HOME/.bashrc"
BACKUP="$HOME/.bashrc.backup.$(date +%Y%m%d_%H%M%S)"

# Backup .bashrc
cp "$BASHRC" "$BACKUP"
echo "Backed up .bashrc to $BACKUP"

# Check if ROS 2 sourcing exists
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS 2 Jazzy (added by Task 1 installation)" >> "$BASHRC"
    echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
    echo "Added ROS 2 sourcing to .bashrc"
fi

# Check if TurtleBot3 model exists
if ! grep -q "TURTLEBOT3_MODEL" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# TurtleBot3 configuration (added by Task 1 installation)" >> "$BASHRC"
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> "$BASHRC"
    echo "export ROS_DOMAIN_ID=30" >> "$BASHRC"
    echo "Added TurtleBot3 configuration to .bashrc"
fi

# Check if Gazebo resource path exists
if ! grep -q "GZ_SIM_RESOURCE_PATH" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Gazebo resource path (added by Task 1 installation)" >> "$BASHRC"
    echo "export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models:\$GZ_SIM_RESOURCE_PATH" >> "$BASHRC"
    echo "Added Gazebo resource path to .bashrc"
fi

echo -e "${GREEN}✓ Environment configured${NC}"
echo ""

# Create models directory
echo -e "${BLUE}Step 7: Creating Gazebo models directory...${NC}"
mkdir -p "$HOME/.gz/models"
echo -e "${GREEN}✓ Models directory created${NC}"
echo ""

# Source environment for current session
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH

# Verify installation
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                  Installation Complete!                      ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}All dependencies installed successfully!${NC}"
echo ""
echo "Verification:"
echo "  ROS 2:     $(ros2 --version 2>&1 | head -n1)"
echo "  Gazebo:    $(gz sim --version 2>&1 | grep -oP 'version \K[0-9.]+' | head -n1)"
echo ""
echo "Next steps:"
echo "  1. Open a new terminal (to load environment variables)"
echo "  2. cd ~/www/ros-robtics-tasks/tasks/task1"
echo "  3. Run verification: ./scripts/verify_setup.sh"
echo "  4. Launch task: ./scripts/launch_task1.sh"
echo ""
echo -e "${YELLOW}IMPORTANT: You MUST open a new terminal for changes to take effect!${NC}"
echo ""
echo "Happy robotics! 🤖"
