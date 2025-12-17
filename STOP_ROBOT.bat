@echo off
REM ============================================
REM Stop TurtleBot3 Simulation
REM ============================================

echo.
echo ========================================
echo   Stopping TurtleBot3 Simulation
echo ========================================
echo.

REM Stop the robot first
echo [1/3] Stopping robot movement...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'" 2>nul
echo.

REM Stop the container
echo [2/3] Stopping Docker container...
docker-compose down
echo.

REM Kill VcXsrv if running
echo [3/3] Stopping X Server...
taskkill /F /IM vcxsrv.exe >nul 2>&1
echo.

echo ========================================
echo   Simulation Stopped
echo ========================================
echo.
pause
