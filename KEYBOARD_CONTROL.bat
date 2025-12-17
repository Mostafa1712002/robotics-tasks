@echo off
REM ============================================
REM Simple Keyboard Control - Press Keys
REM ============================================

echo.
echo ========================================
echo   TurtleBot3 Keyboard Control
echo ========================================
echo.
echo Press keys to control the robot:
echo.
echo   I - Forward
echo   K - Backward
echo   J - Turn Left
echo   L - Turn Right
echo   SPACE - Stop
echo   Q - Quit
echo.
echo ========================================
echo.

:loop
REM Wait for single keypress
choice /C IKJLSQ /N /M "Press key: "

if %errorlevel%==6 goto quit
if %errorlevel%==5 goto stop
if %errorlevel%==4 goto right
if %errorlevel%==3 goto left
if %errorlevel%==2 goto backward
if %errorlevel%==1 goto forward

:forward
echo [FORWARD] Moving forward...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}'" >nul 2>&1
goto loop

:backward
echo [BACKWARD] Moving backward...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: -0.3}, angular: {z: 0.0}}}'" >nul 2>&1
goto loop

:left
echo [LEFT] Turning left...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}'" >nul 2>&1
goto loop

:right
echo [RIGHT] Turning right...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: -0.5}}}'" >nul 2>&1
goto loop

:stop
echo [STOP] Stopping robot...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'" >nul 2>&1
goto loop

:quit
echo.
echo Stopping robot and exiting...
docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'" >nul 2>&1
echo Goodbye!
timeout /t 2 /nobreak >nul
exit
