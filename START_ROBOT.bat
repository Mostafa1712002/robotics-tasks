@echo off
REM ============================================
REM TurtleBot3 Autonomous Navigation Launcher
REM ============================================

REM Create logs directory if it doesn't exist
if not exist "logs" mkdir logs

REM Generate log filename with timestamp
for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value') do set datetime=%%I
set LOG_FILE=logs\startup_%datetime:~0,8%_%datetime:~8,6%.log

echo Starting TurtleBot3 Simulation... > "%LOG_FILE%"
echo Log file: %LOG_FILE% >> "%LOG_FILE%"
echo ======================================== >> "%LOG_FILE%"
echo. >> "%LOG_FILE%"

echo.
echo ========================================
echo   TurtleBot3 Robotics Simulation
echo ========================================
echo.
echo Log file: %LOG_FILE%
echo.

REM Check if Docker is running
echo [STEP 1/5] Checking Docker status...
echo [%TIME%] [STEP 1/5] Checking Docker status... >> "%LOG_FILE%"
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Docker is not running! >> "%LOG_FILE%"
    echo [ERROR] Docker is not running!
    echo Please start Docker Desktop and try again.
    echo. >> "%LOG_FILE%"
    echo Full error details: >> "%LOG_FILE%"
    docker info >> "%LOG_FILE%" 2>&1
    echo.
    pause
    exit /b 1
)
echo [SUCCESS] Docker is running >> "%LOG_FILE%"
docker version >> "%LOG_FILE%" 2>&1
echo. >> "%LOG_FILE%"
echo [1/5] Docker is running...
echo.

REM Check if VcXsrv is running
echo [STEP 2/5] Checking VcXsrv (X Server) status...
echo [%TIME%] [STEP 2/5] Checking VcXsrv status... >> "%LOG_FILE%"
tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I /N "vcxsrv.exe">NUL
if %errorlevel% neq 0 (
    echo [WARNING] VcXsrv is not running! >> "%LOG_FILE%"
    echo [WARNING] VcXsrv is not running!
    echo Starting VcXsrv...
    echo [%TIME%] Starting VcXsrv... >> "%LOG_FILE%"
    start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -ac -terminate -lesspointer -multiwindow -clipboard -wgl -dpi auto
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to start VcXsrv! >> "%LOG_FILE%"
        echo [ERROR] Failed to start VcXsrv!
        echo Please install VcXsrv from: https://sourceforge.net/projects/vcxsrv/
        pause
        exit /b 1
    )
    timeout /t 3 /nobreak >nul
    echo [SUCCESS] VcXsrv started >> "%LOG_FILE%"
) else (
    echo [SUCCESS] VcXsrv already running >> "%LOG_FILE%"
)
echo. >> "%LOG_FILE%"
echo [2/5] X Server is ready...
echo.

REM Start Docker container if not running
echo [STEP 3/5] Starting Docker container...
echo [%TIME%] [STEP 3/5] Checking container status... >> "%LOG_FILE%"
docker ps | find "ros2-turtlebot3-sim" >nul
if %errorlevel% neq 0 (
    echo Container not running, starting now... >> "%LOG_FILE%"
    echo [3/5] Starting Docker container...
    echo [%TIME%] Running: docker-compose up -d >> "%LOG_FILE%"
    docker-compose up -d >> "%LOG_FILE%" 2>&1
    REM Wait a moment then verify it started
    timeout /t 3 /nobreak >nul
    docker ps | find "ros2-turtlebot3-sim" >nul
    if %errorlevel% neq 0 (
        echo [ERROR] Failed to start container! >> "%LOG_FILE%"
        echo [ERROR] Failed to start container!
        echo Check the log file for details: %LOG_FILE%
        pause
        exit /b 1
    )
    echo [SUCCESS] Container started >> "%LOG_FILE%"
    timeout /t 2 /nobreak >nul
) else (
    echo [SUCCESS] Container already running >> "%LOG_FILE%"
    echo [3/5] Docker container already running...
)
echo. >> "%LOG_FILE%"
echo.

REM Launch Gazebo simulation
echo [STEP 4/5] Launching Gazebo simulation...
echo [%TIME%] [STEP 4/5] Launching Gazebo simulation... >> "%LOG_FILE%"
echo Command: docker exec -e DISPLAY=host.docker.internal:0.0 ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/robotics-tasks/task1 && gz sim -r worlds/custom_world.sdf" >> "%LOG_FILE%"
start /B docker exec -e DISPLAY=host.docker.internal:0.0 ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/robotics-tasks/task1 && gz sim -r worlds/custom_world.sdf" >> logs\gazebo.log 2>&1
echo [SUCCESS] Gazebo launch command sent >> "%LOG_FILE%"
timeout /t 5 /nobreak >nul
echo [4/5] Gazebo simulation launching...
echo.

REM Spawn TurtleBot3 robot
echo [STEP 5/5] Spawning TurtleBot3 robot...
echo [%TIME%] [STEP 5/5] Spawning TurtleBot3 robot... >> "%LOG_FILE%"
echo Command: ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:='-2.0' y_pose:='0.0' >> "%LOG_FILE%"
start /B docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:='-2.0' y_pose:='0.0'" >> logs\spawn_robot.log 2>&1
echo [SUCCESS] Robot spawn command sent >> "%LOG_FILE%"
timeout /t 8 /nobreak >nul
echo [5/5] TurtleBot3 robot spawning...
echo.

REM Launch autonomous navigation
echo [FINAL STEP] Starting autonomous navigation...
echo [%TIME%] [FINAL] Starting autonomous navigation... >> "%LOG_FILE%"
echo Command: python3 autonomous_navigator.py >> "%LOG_FILE%"
start /B docker exec ros2-turtlebot3-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /root/robotics-tasks/task1 && python3 autonomous_navigator.py" >> logs\autonomous_nav.log 2>&1
echo [SUCCESS] Autonomous navigation command sent >> "%LOG_FILE%"
timeout /t 3 /nobreak >nul
echo.

echo [%TIME%] All startup commands completed >> "%LOG_FILE%"
echo. >> "%LOG_FILE%"
echo ======================================== >> "%LOG_FILE%"
echo   Startup Summary >> "%LOG_FILE%"
echo ======================================== >> "%LOG_FILE%"
echo - Docker: Running >> "%LOG_FILE%"
echo - VcXsrv: Running >> "%LOG_FILE%"
echo - Container: ros2-turtlebot3-sim >> "%LOG_FILE%"
echo - Gazebo: Launched (see logs\gazebo.log) >> "%LOG_FILE%"
echo - Robot: Spawned (see logs\spawn_robot.log) >> "%LOG_FILE%"
echo - Navigation: Active (see logs\autonomous_nav.log) >> "%LOG_FILE%"
echo. >> "%LOG_FILE%"

echo ========================================
echo   Simulation Started Successfully!
echo ========================================
echo.
echo The robot is now navigating autonomously!
echo.
echo You should see:
echo  - Gazebo window showing the obstacle course
echo  - TurtleBot3 robot avoiding obstacles
echo.
echo Log files saved in 'logs' folder:
echo  - Main log: %LOG_FILE%
echo  - Gazebo: logs\gazebo.log
echo  - Robot spawn: logs\spawn_robot.log
echo  - Navigation: logs\autonomous_nav.log
echo.
echo To stop: Run STOP_ROBOT.bat
echo.

echo Press any key to view the navigation log...
pause

REM Show the navigation log in real-time
echo.
echo ========================================
echo   Live Navigation Log
echo ========================================
echo.
type logs\autonomous_nav.log
echo.
echo.
echo To continue monitoring, run: type logs\autonomous_nav.log
echo.
pause
