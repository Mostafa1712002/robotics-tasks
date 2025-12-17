@echo off
REM ============================================
REM Manual Control for TurtleBot3
REM ============================================

:menu
cls
echo.
echo ========================================
echo   TurtleBot3 Manual Control
echo ========================================
echo.
echo  W - Move Forward
echo  S - Move Backward
echo  A - Turn Left
echo  D - Turn Right
echo  X - Stop
echo  Q - Quit
echo.
echo ========================================
echo.

choice /C WSADXQ /N /M "Enter command: "

if errorlevel 6 goto end
if errorlevel 5 goto stop
if errorlevel 4 goto right
if errorlevel 3 goto left
if errorlevel 2 goto backward
if errorlevel 1 goto forward

:forward
echo Moving forward...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}'"
timeout /t 1 /nobreak >nul
goto menu

:backward
echo Moving backward...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: -0.3}, angular: {z: 0.0}}}'"
timeout /t 1 /nobreak >nul
goto menu

:left
echo Turning left...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}'"
timeout /t 1 /nobreak >nul
goto menu

:right
echo Turning right...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: -0.5}}}'"
timeout /t 1 /nobreak >nul
goto menu

:stop
echo Stopping robot...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'"
timeout /t 1 /nobreak >nul
goto menu

:end
echo.
echo Goodbye!
timeout /t 2 /nobreak >nul
exit
