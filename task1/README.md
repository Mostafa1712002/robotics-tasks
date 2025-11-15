# Task 1: Custom World with TurtleBot3 Teleoperation

## Overview

This task requires you to:
1. Design a custom Gazebo world with obstacles
2. Import/spawn TurtleBot3 robot in the world
3. Use teleop_twist_keyboard to control the robot

**Difficulty:** Beginner to Intermediate
**Estimated Time:** 30-45 minutes

---

## Learning Objectives

By completing this task, you will:
- ✅ Understand Gazebo world design and SDF format
- ✅ Work with TurtleBot3 robot platform
- ✅ Practice robot teleoperation using keyboard controls
- ✅ Learn about ROS 2 - Gazebo integration
- ✅ Understand sensor topics (lidar, camera, IMU, odometry)

---

## What's Provided

### Custom World (`worlds/custom_world.sdf`)
A 12×12 meter arena with:
- **Green floor** - Main navigation area
- **Red boundary walls** - Arena perimeter (1m height)
- **7 colorful obstacles:**
  - Central blue box (1.5×1.5m)
  - 2 yellow cylinders (top corners)
  - 2 purple boxes (bottom corners)
  - 2 green diagonal walls (angled obstacles)

### Configuration
- **Bridge config** (`config/bridge.yaml`) - Topic mappings for:
  - `/cmd_vel` - Robot velocity commands
  - `/odom` - Odometry data
  - `/scan` - LiDAR sensor data
  - `/camera/image_raw` - Camera feed
  - `/imu` - IMU sensor data
  - `/tf` - Transform frames

### Scripts
- **`launch_task1.sh`** - Launch simulation
- **`teleop_control.sh`** - Start keyboard control

---

## Prerequisites

### 1. Install Required Packages

```bash
# Install Gazebo Jetty
sudo apt install gz-jetty

# Install ROS-Gazebo bridge
sudo apt install ros-jazzy-ros-gz

# Install teleop keyboard
sudo apt install ros-jazzy-teleop-twist-keyboard

# Install TurtleBot3 packages
sudo apt install ros-jazzy-turtlebot3*
```

### 2. Install TurtleBot3 Models for Gazebo

Download TurtleBot3 models to Gazebo:

```bash
# Create models directory
mkdir -p ~/.gz/models

# Download TurtleBot3 models
cd ~/.gz/models

# Option 1: Clone from official repo
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cp -r turtlebot3_simulations/turtlebot3_gazebo/models/* .
rm -rf turtlebot3_simulations

# Option 2: Download from Gazebo Fuel (recommended)
# Models will be auto-downloaded when you spawn the robot
```

### 3. Set Environment Variables

Add to your `~/.bashrc`:

```bash
# TurtleBot3
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30

# Gazebo
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH

# Source ROS 2
source /opt/ros/jazzy/setup.bash
```

Apply changes:
```bash
source ~/.bashrc
```

---

## Quick Start (3 Steps)

### Method 1: Using Scripts (Easiest)

**Terminal 1 - Launch Simulation:**
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
./scripts/launch_task1.sh
```

**Terminal 2 - Control Robot:**
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
./scripts/teleop_control.sh
```

### Method 2: Manual Launch (Learning)

**Terminal 1 - Gazebo Only:**
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
gz sim -r worlds/custom_world.sdf
```

**Terminal 2 - Spawn TurtleBot3:**
```bash
# Set model type
export TURTLEBOT3_MODEL=waffle_pi

# Spawn at starting position (4.5, 4.5)
ros2 run ros_gz_sim create \
  -name turtlebot3_waffle_pi \
  -file /path/to/turtlebot3/model.sdf \
  -x 4.5 -y 4.5 -z 0.01 -Y -2.356
```

**Terminal 3 - Bridge Topics:**
```bash
cd ~/www/ros-robtics-tasks/tasks/task1
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=config/bridge.yaml
```

**Terminal 4 - Teleop Control:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Usage Instructions

### Keyboard Controls

Once teleop is running, use these keys:

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

u/o : Rotate left/right while moving forward
i   : Move forward
,   : Move backward
j/l : Rotate left/right in place
k   : Stop

Speed adjustment:
q/z : Increase/decrease linear speed by 10%
w/x : Increase/decrease angular speed by 10%

Press Ctrl+C to exit
```

### Navigation Challenge

Try to navigate the robot through the obstacles:

**Easy Route:**
- Start position: Top-right corner (4.5, 4.5)
- Goal: Reach opposite corner (-4.5, -4.5)
- Avoid all obstacles

**Medium Challenge:**
- Navigate a complete loop around the central box
- Don't hit any walls or obstacles

**Hard Challenge:**
- Navigate through the narrow gaps between obstacles
- Complete figure-8 pattern around the cylinders

---

## Verification

### Check Topics Are Publishing

```bash
# List all topics
ros2 topic list

# Expected output:
# /cmd_vel
# /odom
# /scan
# /camera/image_raw
# /camera/camera_info
# /imu
# /tf

# Monitor laser scan
ros2 topic echo /scan --once

# Monitor odometry
ros2 topic echo /odom --once

# Check camera image
ros2 topic hz /camera/image_raw
```

### Visualize in RViz (Optional)

```bash
# Launch RViz
rviz2

# Add displays:
# - RobotModel (set Fixed Frame to "odom")
# - LaserScan (topic: /scan)
# - Camera (topic: /camera/image_raw)
# - Odometry (topic: /odom)
# - TF
```

---

## World Design Details

### Arena Layout

```
     N (Red Wall)
     ↑
W ←  +  → E (Red Walls)
     ↓
     S (Red Wall)

Arena: 12×12 meters
Floor: Green (0.3, 0.5, 0.3)
Boundaries: Red walls (0.8, 0.3, 0.3)
```

### Obstacles Map

```
(-3,3)         (0,3)          (3,3)
  🟡           [Blue]          🟡
Yellow       Central Box      Yellow
Cylinder                     Cylinder

  \Green/                   /Green\
   \Wall/                   /Wall/

(-3,-3)                      (3,-3)
  🟣                          🟣
Purple                      Purple
 Box                         Box

🟡 = Yellow Cylinders (r=0.5m, h=0.8m)
🟣 = Purple Boxes (1×1×0.5m)
[Blue] = Central Box (1.5×1.5×0.6m)
Green = Diagonal Walls (2×0.2×0.6m)
```

### Starting Position

- **X:** 4.5 meters (near east wall)
- **Y:** 4.5 meters (near north wall)
- **Z:** 0.01 meters (on ground)
- **Yaw:** -135° (facing toward center)

---

## Customization Ideas

### Modify the World

**Add More Obstacles:**
```xml
<!-- Add to custom_world.sdf -->
<model name="my_obstacle">
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>W H D</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>W H D</size></box>
      </geometry>
      <material>
        <ambient>R G B 1</ambient>
        <diffuse>R G B 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Change Colors:**
Modify the `<ambient>` and `<diffuse>` values (RGB range 0-1)

**Resize Arena:**
Change the boundary wall positions and floor size

### Try Different TurtleBot3 Models

```bash
# Burger (smallest)
export TURTLEBOT3_MODEL=burger

# Waffle (medium)
export TURTLEBOT3_MODEL=waffle

# Waffle Pi (largest, with camera)
export TURTLEBOT3_MODEL=waffle_pi  # Default
```

---

## Troubleshooting

### Robot Doesn't Spawn

**Check model path:**
```bash
echo $GZ_SIM_RESOURCE_PATH
# Should include: /home/your_user/.gz/models

ls ~/.gz/models/
# Should show TurtleBot3 models
```

**Download models manually:**
```bash
# Gazebo will auto-download from Fuel on first spawn
# Or manually download and place in ~/.gz/models/
```

### Robot Doesn't Move

**Check cmd_vel topic:**
```bash
ros2 topic list | grep cmd_vel
# Should show: /cmd_vel

ros2 topic info /cmd_vel
# Should show publishers and subscribers
```

**Test manual command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --once
```

### Teleop Not Working

**Install teleop package:**
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

**Check if it's running:**
```bash
ros2 node list
# Should show: /teleop_twist_keyboard
```

### Bridge Issues

**Check bridge config:**
```bash
cat ~/www/ros-robtics-tasks/tasks/task1/config/bridge.yaml
```

**Verify Gazebo topics:**
```bash
gz topic -l
# Should show Gazebo-side topics
```

**Check ROS topics:**
```bash
ros2 topic list
# Should show ROS-side topics
```

---

## Task Deliverables

To complete this task, demonstrate:

1. ✅ **Custom world loads successfully** in Gazebo
2. ✅ **TurtleBot3 spawns** at the starting position
3. ✅ **Keyboard teleop controls work** - robot moves in response to commands
4. ✅ **Navigate around obstacles** - show the robot can avoid collisions
5. ✅ **Sensor data is published** - verify topics are active

### Bonus Challenges

- 🌟 **Record a rosbag** of the navigation session
- 🌟 **Visualize in RViz** with all sensor data
- 🌟 **Add custom obstacles** to make the world more complex
- 🌟 **Complete the navigation challenge** (corner to corner)
- 🌟 **Create a video** showing the robot navigation

---

## File Structure

```
tasks/task1/
├── README.md                      # This file
├── worlds/
│   └── custom_world.sdf           # Custom Gazebo world
├── config/
│   └── bridge.yaml                # ROS-Gazebo topic bridge
├── launch/
│   └── task1_simulation.launch.py # ROS 2 launch file
└── scripts/
    ├── launch_task1.sh            # Quick launch script
    └── teleop_control.sh          # Teleop startup script
```

---

## Related Resources

- **TurtleBot3 Official Docs:** https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Gazebo Tutorials:** https://gazebosim.org/docs
- **ROS 2 Jazzy Docs:** https://docs.ros.org/en/jazzy/
- **Teleop Twist Keyboard:** https://github.com/ros2/teleop_twist_keyboard

---

## Next Steps

After completing this task:

1. **Section 04:** Create custom keyboard teleop node
2. **Section 05:** Advanced TurtleBot3 with RViz and sensors
3. **Autonomous Navigation:** Implement wall-following algorithm

---

**Good luck with your robot navigation! 🤖**
