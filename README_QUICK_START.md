# TurtleBot3 Robotics Simulation - Quick Start Guide

## One-Click Launchers

I've created four easy-to-use batch files for you:

### 🚀 START_ROBOT.bat
**Double-click to start the simulation!**

This will automatically:
1. Check if Docker is running
2. Start VcXsrv (X Server) if not running
3. Start the Docker container
4. Launch Gazebo with the obstacle course
5. Spawn the TurtleBot3 robot
6. Start autonomous navigation
7. **Save detailed logs of everything!**

**Just double-click and wait 15-20 seconds for everything to start!**

All logs are saved in the `logs/` folder:
- **startup_YYYYMMDD_HHMMSS.log** - Main startup log with timestamps
- **gazebo.log** - Gazebo simulation output
- **spawn_robot.log** - Robot spawning process
- **autonomous_nav.log** - Live navigation decisions

---

### 📋 VIEW_LOGS.bat
**Double-click to view logs!**

Interactive menu to:
- View latest startup log
- View Gazebo log
- View robot spawn log
- View navigation log (live updates!)
- List all startup logs
- Open logs folder
- Clear all logs

---

### 🛑 STOP_ROBOT.bat
**Double-click to stop everything!**

This will:
1. Stop the robot movement
2. Stop the Docker container
3. Close VcXsrv X Server

---

### 🎮 MANUAL_CONTROL.bat
**Double-click to manually control the robot!**

Use keyboard controls:
- **W** - Move Forward
- **S** - Move Backward
- **A** - Turn Left
- **D** - Turn Right
- **X** - Stop
- **Q** - Quit

---

## First Time Setup

### Prerequisites (Install Once)
1. **Docker Desktop** - https://www.docker.com/products/docker-desktop/
2. **VcXsrv X Server** - https://sourceforge.net/projects/vcxsrv/

### First Run
1. Make sure Docker Desktop is running (whale icon in system tray)
2. Double-click **START_ROBOT.bat**
3. Wait 15-20 seconds
4. You should see the Gazebo window appear with the robot!

---

## What You'll See

### Gazebo Window
- Green floor arena (12x12 meters)
- Red boundary walls
- Various colored obstacles:
  - Blue central box
  - Yellow cylinders
  - Purple boxes
  - Teal diagonal walls
- TurtleBot3 Waffle Pi robot with camera and LiDAR

### Autonomous Navigation
The robot will automatically:
- Use LiDAR to detect obstacles
- Navigate around obstacles
- Explore the arena
- Avoid collisions

---

## Project Structure

```
robotics-tasks/
├── START_ROBOT.bat          ← Double-click to start!
├── STOP_ROBOT.bat           ← Double-click to stop!
├── MANUAL_CONTROL.bat       ← Double-click for manual control!
├── docker-compose.yml       ← Docker configuration
├── Dockerfile               ← Docker image definition
├── task1/
│   ├── autonomous_navigator.py  ← Autonomous navigation code
│   ├── worlds/
│   │   └── custom_world.sdf     ← Gazebo world file
│   └── scripts/
│       ├── launch_task1.sh
│       └── teleop_control.sh
└── control-robot.ps1        ← PowerShell control script
```

---

## Troubleshooting

### Problem: "Docker is not running"
**Solution:** Start Docker Desktop and wait for it to fully start (whale icon should be steady, not animated)

### Problem: "No Gazebo window appears"
**Solution:**
1. Check if VcXsrv is running (should see X icon in system tray)
2. Run `START_ROBOT.bat` again

### Problem: "Robot not moving"
**Solution:**
1. Check the console output for errors
2. Run `STOP_ROBOT.bat` then `START_ROBOT.bat` again

### Problem: "Container already exists" error
**Solution:** Run `STOP_ROBOT.bat` first, then `START_ROBOT.bat`

---

## Advanced Usage

### View Robot Camera Feed
```bash
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 run rqt_image_view rqt_image_view"
```

### View LiDAR Data
```bash
docker exec -it ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /scan"
```

### View Robot Position
```bash
docker exec -it ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /odom"
```

### List All ROS Topics
```bash
docker exec -it ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

---

## Technical Details

- **ROS Version:** ROS 2 Jazzy Jalisco
- **Gazebo Version:** Gazebo Harmonic (gz-sim)
- **Robot Model:** TurtleBot3 Waffle Pi
- **Sensors:** 360° LiDAR, RGB Camera, IMU
- **OS:** Ubuntu 24.04 (in Docker container)
- **Navigation:** Custom Python node using LiDAR-based obstacle avoidance

---

## Credits

Built with:
- ROS 2 Jazzy
- Gazebo Harmonic
- TurtleBot3 packages
- Docker Desktop for Windows
- VcXsrv X Server

---

**Enjoy exploring robotics with TurtleBot3!** 🤖
