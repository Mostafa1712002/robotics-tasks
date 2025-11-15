#!/bin/bash

# Teleop Control Script for Task 1
# Launches keyboard teleoperation for TurtleBot3

echo "========================================"
echo "Task 1: TurtleBot3 Teleop Control"
echo "========================================"
echo ""

# Check if simulation is running
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo "ERROR: Simulation not running or /cmd_vel topic not available"
    echo "Please launch the simulation first:"
    echo "  cd ~/www/ros-robtics-tasks/tasks/task1"
    echo "  ./scripts/launch_task1.sh"
    exit 1
fi

echo "Starting keyboard teleoperation..."
echo ""
echo "Controls:"
echo "  i - Forward"
echo "  , - Backward"
echo "  j - Turn left"
echo "  l - Turn right"
echo "  k - Stop"
echo "  q/z - Increase/decrease speed"
echo "  w/x - Increase/decrease turn speed"
echo ""
echo "Press Ctrl+C to exit"
echo ""

# Launch teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -r /cmd_vel:=/cmd_vel
