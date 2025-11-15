#!/bin/bash

# Task 1 Launch Script
# Launches the custom world with TurtleBot3

echo "========================================"
echo "Task 1: Custom World Simulation"
echo "========================================"
echo ""

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH

echo "Environment:"
echo "  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""

# Check if in correct directory
TASK1_DIR="$HOME/www/ros-robtics-tasks/tasks/task1"
if [ ! -d "$TASK1_DIR" ]; then
    echo "ERROR: Task1 directory not found at $TASK1_DIR"
    exit 1
fi

# Check if world file exists
WORLD_FILE="$TASK1_DIR/worlds/custom_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found: $WORLD_FILE"
    exit 1
fi

echo "Launching simulation..."
echo "  World: $WORLD_FILE"
echo ""
echo "NOTE: If TurtleBot3 doesn't spawn, you may need to:"
echo "  1. Install TurtleBot3 packages"
echo "  2. Download TurtleBot3 models to ~/.gz/models/"
echo ""
echo "After simulation starts:"
echo "  - Press play button in Gazebo"
echo "  - Open new terminal and run: cd tasks/task1 && ./scripts/teleop_control.sh"
echo ""

# Launch Gazebo directly (simple approach)
gz sim -r "$WORLD_FILE"
