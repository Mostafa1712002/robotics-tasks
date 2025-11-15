#!/bin/bash

# Verification script for Task 1 setup

set -e

echo "=========================================="
echo "Task 1: Setup Verification"
echo "=========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Test function
test_check() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: $1"
        ((TESTS_PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC}: $1"
        ((TESTS_FAILED++))
    fi
}

echo "=== Phase 1: System Requirements ==="
echo ""

# Test 1: Check Gazebo
echo -n "Test 1: Checking Gazebo Jetty... "
which gz >/dev/null 2>&1
test_check "Gazebo Jetty installed"

# Test 2: Check ROS 2
echo -n "Test 2: Checking ROS 2... "
which ros2 >/dev/null 2>&1
test_check "ROS 2 installed"

# Test 3: Check teleop package
echo -n "Test 3: Checking teleop_twist_keyboard... "
ros2 pkg list 2>/dev/null | grep -q "teleop_twist_keyboard"
test_check "teleop_twist_keyboard package"

# Test 4: Check ros_gz packages
echo -n "Test 4: Checking ros_gz packages... "
if [ -d "/opt/ros/jazzy/share/ros_gz_sim" ]; then
    test_check "ros_gz packages installed"
else
    echo -e "${RED}✗ FAIL${NC}: ros_gz packages not found"
    ((TESTS_FAILED++))
fi

# Test 5: Check TurtleBot3 packages
echo -n "Test 5: Checking TurtleBot3 packages... "
ros2 pkg list 2>/dev/null | grep -q "turtlebot3"
test_check "TurtleBot3 packages installed"

echo ""
echo "=== Phase 2: Environment Variables ==="
echo ""

# Test 6: Check TURTLEBOT3_MODEL
echo -n "Test 6: Checking TURTLEBOT3_MODEL... "
if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}: TURTLEBOT3_MODEL not set"
    echo "   Run: export TURTLEBOT3_MODEL=waffle_pi"
    ((TESTS_FAILED++))
else
    echo -e "${GREEN}✓ PASS${NC}: TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
    ((TESTS_PASSED++))
fi

# Test 7: Check ROS_DOMAIN_ID
echo -n "Test 7: Checking ROS_DOMAIN_ID... "
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}: ROS_DOMAIN_ID not set"
    echo "   Run: export ROS_DOMAIN_ID=30"
    ((TESTS_FAILED++))
else
    echo -e "${GREEN}✓ PASS${NC}: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    ((TESTS_PASSED++))
fi

# Test 8: Check GZ_SIM_RESOURCE_PATH
echo -n "Test 8: Checking GZ_SIM_RESOURCE_PATH... "
if echo "$GZ_SIM_RESOURCE_PATH" | grep -q ".gz/models"; then
    test_check "GZ_SIM_RESOURCE_PATH is set"
else
    echo -e "${YELLOW}⚠ WARNING${NC}: GZ_SIM_RESOURCE_PATH not configured"
    echo "   Run: export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models:\$GZ_SIM_RESOURCE_PATH"
    ((TESTS_FAILED++))
fi

echo ""
echo "=== Phase 3: Task Files ==="
echo ""

TASK1_DIR="$HOME/www/ros-robtics-tasks/tasks/task1"

# Test 9: Check world file
echo -n "Test 9: Checking world file... "
[ -f "$TASK1_DIR/worlds/custom_world.sdf" ]
test_check "custom_world.sdf exists"

# Test 10: Check bridge config
echo -n "Test 10: Checking bridge config... "
[ -f "$TASK1_DIR/config/bridge.yaml" ]
test_check "bridge.yaml exists"

# Test 11: Check launch file
echo -n "Test 11: Checking launch file... "
[ -f "$TASK1_DIR/launch/task1_simulation.launch.py" ]
test_check "task1_simulation.launch.py exists"

# Test 12: Check scripts
echo -n "Test 12: Checking launch script... "
[ -f "$TASK1_DIR/scripts/launch_task1.sh" ]
test_check "launch_task1.sh exists"

echo -n "Test 13: Checking teleop script... "
[ -f "$TASK1_DIR/scripts/teleop_control.sh" ]
test_check "teleop_control.sh exists"

# Test 14: Check script permissions
echo -n "Test 14: Checking script permissions... "
[ -x "$TASK1_DIR/scripts/launch_task1.sh" ] && [ -x "$TASK1_DIR/scripts/teleop_control.sh" ]
test_check "Scripts are executable"

echo ""
echo "=== Phase 4: Gazebo Models ==="
echo ""

# Test 15: Check models directory
echo -n "Test 15: Checking models directory... "
[ -d "$HOME/.gz/models" ]
test_check "~/.gz/models directory exists"

echo ""
echo "=========================================="
echo "Test Results Summary"
echo "=========================================="
echo -e "${GREEN}Passed: $TESTS_PASSED${NC}"
echo -e "${RED}Failed: $TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! You're ready to run Task 1.${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Terminal 1: cd $TASK1_DIR && ./scripts/launch_task1.sh"
    echo "2. Terminal 2: cd $TASK1_DIR && ./scripts/teleop_control.sh"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please fix the issues above.${NC}"
    echo ""
    echo "Common fixes:"
    echo "- Install missing packages: sudo apt install gz-jetty ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard"
    echo "- Set environment variables (add to ~/.bashrc):"
    echo "    export TURTLEBOT3_MODEL=waffle_pi"
    echo "    export ROS_DOMAIN_ID=30"
    echo "    export GZ_SIM_RESOURCE_PATH=\$HOME/.gz/models:\$GZ_SIM_RESOURCE_PATH"
    echo "- Source ROS: source /opt/ros/jazzy/setup.bash"
    exit 1
fi
