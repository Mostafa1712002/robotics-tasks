# Running ROS 2 Robotics Tasks on Windows

This guide explains how to run the ROS 2 TurtleBot3 simulation on Windows using Docker.

## Prerequisites

### 1. Install Docker Desktop for Windows

**Download and Install:**
1. Go to https://www.docker.com/products/docker-desktop/
2. Download Docker Desktop for Windows
3. Run the installer
4. Follow the installation wizard
5. Restart your computer when prompted

**Verify Installation:**
```powershell
docker --version
docker-compose --version
```

### 2. Install WSL 2 (Windows Subsystem for Linux)

Docker Desktop requires WSL 2 backend.

**If not already installed:**
```powershell
# Run in PowerShell as Administrator
wsl --install
```

Restart your computer after installation.

### 3. Install X Server for GUI (Gazebo Visualization)

You need an X Server to display Gazebo GUI on Windows.

**Option A: VcXsrv (Recommended)**
1. Download from https://sourceforge.net/projects/vcxsrv/
2. Install VcXsrv
3. Launch XLaunch
4. Use these settings:
   - Display number: 0
   - Start no client
   - Check "Disable access control"
   - Save configuration

**Option B: WSLg (Windows 11 with WSL 2.0+)**
- WSL 2 on Windows 11 has built-in GUI support
- No additional X Server needed

### 4. Configure Display

**For VcXsrv (Windows 10 or Windows 11):**
```powershell
# Get your Windows IP address
ipconfig

# Set DISPLAY environment variable (replace with your IP)
$env:DISPLAY = "YOUR_IP_ADDRESS:0.0"
# Example: $env:DISPLAY = "192.168.1.100:0.0"
```

**For WSLg (Windows 11):**
```powershell
# WSLg handles this automatically
$env:DISPLAY = ":0"
```

## Quick Start

### Step 1: Build Docker Image

Open PowerShell or Command Prompt in the `robotics-tasks` directory:

```powershell
cd C:\colleague\robotics-tasks
docker-compose build
```

This will take 10-15 minutes the first time.

### Step 2: Start X Server

**If using VcXsrv:**
- Launch XLaunch with the settings mentioned above
- Keep it running in the background

### Step 3: Run Container

```powershell
docker-compose up -d
```

### Step 4: Enter Container

```powershell
docker exec -it ros2-turtlebot3-sim /bin/bash
```

You're now inside the Ubuntu container with ROS 2 Jazzy!

### Step 5: Run Task 1 Simulation

**Terminal 1 (inside container) - Launch Gazebo:**
```bash
cd /root/robotics-tasks/task1
./scripts/launch_task1.sh
```

Wait for Gazebo to load, then click the Play button ▶

**Terminal 2 (new PowerShell window) - Start Teleop:**
```powershell
docker exec -it ros2-turtlebot3-sim /bin/bash
```

Then inside the container:
```bash
cd /root/robotics-tasks/task1
./scripts/teleop_control.sh
```

## Usage

### Keyboard Controls

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i - Move forward
, - Move backward
j - Rotate left
l - Rotate right
k - Stop

q/z - Increase/decrease linear speed
w/x - Increase/decrease angular speed

Ctrl+C to exit
```

## Useful Docker Commands

### Start Container
```powershell
docker-compose up -d
```

### Stop Container
```powershell
docker-compose down
```

### Enter Container Shell
```powershell
docker exec -it ros2-turtlebot3-sim /bin/bash
```

### View Container Logs
```powershell
docker logs ros2-turtlebot3-sim
```

### Rebuild Container (after code changes)
```powershell
docker-compose build --no-cache
docker-compose up -d
```

### List Running Containers
```powershell
docker ps
```

## Troubleshooting

### Issue: Gazebo GUI Doesn't Appear

**Solution 1: Check X Server**
- Make sure VcXsrv is running
- Check "Disable access control" is enabled
- Verify firewall isn't blocking it

**Solution 2: Check DISPLAY Variable**
```powershell
# Inside container
echo $DISPLAY

# Should show your IP:0.0 or :0
```

**Solution 3: Test X11**
```bash
# Inside container
apt-get update && apt-get install -y x11-apps
xeyes
```

If xeyes appears, X11 is working.

### Issue: Container Won't Start

**Check Docker Status:**
```powershell
docker ps -a
docker logs ros2-turtlebot3-sim
```

**Restart Docker Desktop:**
- Right-click Docker Desktop icon
- Select "Restart"

### Issue: Robot Doesn't Move

**Check ROS Topics:**
```bash
# Inside container
ros2 topic list
ros2 topic echo /cmd_vel
```

**Send Test Command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
```

### Issue: Performance Issues

**Allocate More Resources to Docker:**
1. Open Docker Desktop
2. Go to Settings → Resources
3. Increase CPU and Memory allocation
4. Apply & Restart

## Alternative: Using WSL 2 Directly (Advanced)

If you have WSL 2, you can run the project natively in Ubuntu:

```bash
# In WSL 2 Ubuntu terminal
cd /mnt/c/colleague/robotics-tasks/task1
./scripts/install_dependencies.sh
./scripts/verify_setup.sh
./scripts/launch_task1.sh
```

This provides better performance but requires Ubuntu setup.

## Verification

Check if everything is working:

```bash
# Inside container
cd /root/robotics-tasks/task1
./scripts/verify_setup.sh
```

All tests should pass ✓

## Next Steps

After successfully running Task 1:
- Try the navigation challenges (see task1/README.md)
- Modify the world file (task1/worlds/custom_world.sdf)
- Explore RViz visualization
- Try other tasks

## Resources

- **Docker Desktop:** https://docs.docker.com/desktop/windows/
- **WSL 2 Setup:** https://docs.microsoft.com/en-us/windows/wsl/install
- **VcXsrv Guide:** https://sourceforge.net/projects/vcxsrv/
- **ROS 2 Docs:** https://docs.ros.org/en/jazzy/

## Help

If you encounter issues:
1. Check Docker Desktop is running
2. Verify X Server (VcXsrv) is running
3. Check container logs: `docker logs ros2-turtlebot3-sim`
4. Review DISPLAY environment variable
5. Restart Docker Desktop

Happy Robotics! 🤖
