# Task 1: Quick Start Guide

## 🚀 Fastest Way to Get Started

### Prerequisites (One-Time Setup)

```bash
# Install packages
sudo apt install gz-jetty ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard ros-jazzy-turtlebot3*

# Set environment
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 🎮 Run Task (2 Terminals)

### Terminal 1: Launch World
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
./scripts/launch_task1.sh
```

**Wait for Gazebo to open, then press the ▶ (play) button**

### Terminal 2: Control Robot
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
./scripts/teleop_control.sh
```

**Use keyboard to move:**
- `i` = Forward
- `,` = Backward
- `j` = Rotate left
- `l` = Rotate right
- `k` = Stop

---

## ✅ Verification

```bash
# Check topics (in new terminal)
ros2 topic list

# Should see:
# /cmd_vel
# /odom
# /scan
# /camera/image_raw
# /imu
```

---

## 📋 Task Objective

**Navigate the robot from starting position (top-right corner) to opposite corner without hitting obstacles!**

**Map:**
```
🟡 = Yellow cylinders
🟣 = Purple boxes
🔵 = Blue central box
🟩 = Green diagonal walls
🔴 = Red boundary walls

     N
     ↑
  🟡 🔵 🟡
    \ | /
W ← 🟩 + 🟩 → E
    / | \
  🟣   🟣
     ↓
     S

START: Top-right (🤖)
GOAL: Bottom-left
```

---

## 🐛 Quick Troubleshooting

### Robot doesn't spawn?
```bash
# Make sure Gazebo models are available
mkdir -p ~/.gz/models
# Gazebo will auto-download TurtleBot3 from Fuel
```

### Robot doesn't move?
```bash
# Test manual command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
```

### Teleop not working?
```bash
# Reinstall
sudo apt install ros-jazzy-teleop-twist-keyboard
```

---

## 📚 Full Documentation

For detailed instructions, see: [`README.md`](README.md)

---

**🎉 Have fun navigating! 🤖**
