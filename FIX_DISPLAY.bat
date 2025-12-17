@echo off
REM ============================================
REM Fix Gazebo Display Issues
REM ============================================

echo.
echo ========================================
echo   Fixing Gazebo Display
echo ========================================
echo.

echo [1/4] Stopping all Gazebo and navigation processes...
docker exec ros2-turtlebot3-sim bash -c "pkill -9 gz" >nul 2>&1
docker exec ros2-turtlebot3-sim bash -c "pkill -9 python3" >nul 2>&1
timeout /t 2 /nobreak >nul
echo Done.

echo [2/4] Restarting VcXsrv...
taskkill /F /IM vcxsrv.exe >nul 2>&1
timeout /t 2 /nobreak >nul
start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -ac -terminate -lesspointer -multiwindow -clipboard -wgl -dpi auto
timeout /t 3 /nobreak >nul
echo Done.

echo [3/4] Launching Gazebo with proper display...
start /B docker exec -e DISPLAY=host.docker.internal:0.0 ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/robotics-tasks/task1 && gz sim -r worlds/custom_world.sdf" >> logs\gazebo_fixed.log 2>&1
timeout /t 8 /nobreak >nul
echo Done.

echo [4/4] Spawning robot...
start /B docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:='-2.0' y_pose:='0.0'" >> logs\spawn_fixed.log 2>&1
timeout /t 8 /nobreak >nul
echo Done.

echo.
echo ========================================
echo   Display Fixed!
echo ========================================
echo.
echo Gazebo should now display properly.
echo.
echo To control the robot manually:
echo   Run: KEYBOARD_CONTROL.bat
echo.
pause
