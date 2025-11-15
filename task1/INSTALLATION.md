# Task 1: Complete Installation Guide

## System Requirements

- **OS:** Ubuntu 24.04 LTS (Noble Numbat)
- **RAM:** 4GB minimum, 8GB recommended
- **Disk:** 10GB free space

---

## Step-by-Step Installation

### Step 1: Install ROS 2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

**Verify ROS 2:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

### Step 2: Install Gazebo Jetty

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg -y

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Jetty
sudo apt-get update
sudo apt-get install gz-jetty -y
```

**Verify Gazebo:**
```bash
gz sim --version
```

Expected output: `Gazebo Sim, version 9.x.x`

---

### Step 3: Install ROS-Gazebo Bridge

```bash
# Install ros_gz packages
sudo apt install ros-jazzy-ros-gz -y

# This includes:
# - ros-jazzy-ros-gz-sim
# - ros-jazzy-ros-gz-bridge
# - ros-jazzy-ros-gz-image
```

**Verify installation:**
```bash
ros2 pkg list | grep ros_gz
```

---

### Step 4: Install TurtleBot3 Packages

```bash
# Install TurtleBot3 packages
sudo apt install ros-jazzy-turtlebot3* -y

# Install teleop keyboard
sudo apt install ros-jazzy-teleop-twist-keyboard -y
```

**Verify TurtleBot3:**
```bash
ros2 pkg list | grep turtlebot3
```

---

### Step 5: Configure Environment

Add to `~/.bashrc`:

```bash
nano ~/.bashrc
```

Add these lines at the end:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# TurtleBot3
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30

# Gazebo
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH

# Aliases (optional)
alias task1='cd ~/www/ros-robtics-tasks/tasks/task1'
```

**Apply changes:**
```bash
source ~/.bashrc
```

---

### Step 6: Create Models Directory

```bash
# Create Gazebo models directory
mkdir -p ~/.gz/models

# Gazebo will auto-download TurtleBot3 models from Fuel on first spawn
```

---

### Step 7: Verify Complete Setup

Run the verification script:

```bash
cd ~/www/ros-robtics-tasks/tasks/task1
./scripts/verify_setup.sh
```

**Expected:** All 15 tests should pass.

---

## Quick Test

After installation, test the setup:

**Terminal 1 - Launch World:**
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
gz sim -r worlds/custom_world.sdf
```

You should see the custom arena with colorful obstacles!

**Terminal 2 - Check ROS Topics (after full launch):**
```bash
ros2 topic list
```

---

## Troubleshooting Installation

### Issue: ROS 2 repository not found

**Solution:**
```bash
# Verify Ubuntu version
lsb_release -a
# Must be 24.04 (Noble)

# Re-add repository
sudo rm /etc/apt/sources.list.d/ros2.list
# Re-run Step 1
```

### Issue: Gazebo installation fails

**Solution:**
```bash
# Check Ubuntu version compatibility
lsb_release -cs
# Should output: noble

# Clean and retry
sudo apt clean
sudo apt update
sudo apt install gz-jetty
```

### Issue: Package not found errors

**Solution:**
```bash
# Update package lists
sudo apt update

# Fix broken dependencies
sudo apt --fix-broken install

# Retry installation
```

### Issue: Permission denied

**Solution:**
```bash
# Add user to dialout group (for robot communication)
sudo usermod -a -G dialout $USER

# Logout and login again
```

---

## Installation Verification Checklist

After installation, verify:

- [ ] `ros2 --version` shows ROS 2 Jazzy
- [ ] `gz sim --version` shows Gazebo Jetty 9.x
- [ ] `ros2 pkg list | grep ros_gz` shows bridge packages
- [ ] `ros2 pkg list | grep turtlebot3` shows TurtleBot3 packages
- [ ] `echo $TURTLEBOT3_MODEL` shows `waffle_pi`
- [ ] `echo $GZ_SIM_RESOURCE_PATH` includes `.gz/models`
- [ ] `./scripts/verify_setup.sh` passes all tests

---

## Disk Space Requirements

- **ROS 2 Jazzy:** ~3 GB
- **Gazebo Jetty:** ~500 MB
- **TurtleBot3 packages:** ~200 MB
- **Dependencies:** ~1 GB
- **Total:** ~5 GB

---

## Alternative: Docker Installation

If you prefer Docker:

```bash
# Pull ROS 2 + Gazebo image (coming soon)
docker pull ros:jazzy-gz

# Run container
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ~/www/ros-robtics-tasks:/workspace \
  ros:jazzy-gz
```

---

## Next Steps

After successful installation:

1. ✅ Run verification: `./scripts/verify_setup.sh`
2. ✅ Read quick start: `QUICK_START.md`
3. ✅ Launch task: `./scripts/launch_task1.sh`
4. ✅ Control robot: `./scripts/teleop_control.sh`

---

## Support

If you encounter issues:

1. Check this installation guide
2. Run `./scripts/verify_setup.sh` for diagnostics
3. Review error messages carefully
4. Check Ubuntu version compatibility (must be 24.04)

---

**Good luck with your installation! 🚀**
