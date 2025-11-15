# ROS 2 Robotics Tasks

This directory contains hands-on robotics tasks designed to practice and demonstrate ROS 2 and Gazebo skills.

---

## 📋 Available Tasks

### Task 1: Custom World + TurtleBot3 Teleoperation
**Status:** ✅ Complete

**Objective:**
- Design custom Gazebo world with obstacles
- Import/spawn TurtleBot3 robot
- Use teleop_twist_keyboard for control

**Directory:** [`task1/`](task1/)

**Quick Start:**
```bash
cd task1
./scripts/launch_task1.sh      # Terminal 1
./scripts/teleop_control.sh    # Terminal 2
```

**Features:**
- 12×12m custom arena with 7 colorful obstacles
- TurtleBot3 waffle_pi with full sensor suite
- 7 bridged topics (cmd_vel, scan, camera, IMU, odom, tf)
- 3 difficulty navigation challenges
- Comprehensive documentation

**Documentation:**
- [Quick Start](task1/QUICK_START.md)
- [Complete Guide](task1/README.md)
- [Overview](task1/TASK_OVERVIEW.txt)

---

## 🎯 Task Requirements Format

Each task follows this structure:

```
tasks/taskN/
├── README.md                   # Complete documentation
├── QUICK_START.md             # Fast reference
├── TASK_OVERVIEW.txt          # Visual overview
├── worlds/                    # Gazebo world files
├── config/                    # Configuration files
├── launch/                    # ROS 2 launch files
└── scripts/                   # Helper scripts
```

---

## 🚀 Getting Started

### Prerequisites

All tasks require:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco
- Gazebo Jetty (gz-sim)

### Installation

```bash
# Install core packages
sudo apt install ros-jazzy-desktop gz-jetty ros-jazzy-ros-gz

# Install TurtleBot3 (for Task 1)
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-teleop-twist-keyboard

# Set environment
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 📚 Related Resources

- **Sections:** [`../sections/`](../sections/) - Lab exercises (01-05)
- **ROS 2 Docs:** https://docs.ros.org/en/jazzy/
- **Gazebo Docs:** https://gazebosim.org/docs
- **TurtleBot3:** https://emanual.robotis.com/docs/en/platform/turtlebot3/

---

## 🎓 Learning Path

**Recommended sequence:**

1. **Section 01-02:** ROS 2 basics and topics
2. **Section 03:** Warehouse robot simulation (custom robot)
3. **Task 1:** Custom world with TurtleBot3 (this directory)
4. **Section 04:** Keyboard teleop control
5. **Section 05:** TurtleBot3 with RViz and sensors

---

## 🤝 Contributing

To add a new task:

1. Create directory: `tasks/taskN/`
2. Follow the standard structure
3. Include complete documentation
4. Add verification script
5. Update this README

---

## 📝 Task Status

| Task | Status | Description |
|------|--------|-------------|
| Task 1 | ✅ Complete | Custom World + TurtleBot3 Teleoperation |
| Task 2 | 🔜 Planned | Autonomous Wall Following |
| Task 3 | 🔜 Planned | SLAM and Mapping |
| Task 4 | 🔜 Planned | Multi-Robot Coordination |

---

**Happy Coding! 🤖**
# robotics-tasks
