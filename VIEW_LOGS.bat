@echo off
REM ============================================
REM View TurtleBot3 Simulation Logs
REM ============================================

:menu
cls
echo.
echo ========================================
echo   TurtleBot3 Simulation Logs
echo ========================================
echo.
echo  1 - View Latest Startup Log
echo  2 - View Gazebo Log
echo  3 - View Robot Spawn Log
echo  4 - View Navigation Log (Live)
echo  5 - View All Startup Logs
echo  6 - Open Logs Folder
echo  7 - Clear All Logs
echo  Q - Quit
echo.
echo ========================================
echo.

choice /C 1234567Q /N /M "Select option: "

if errorlevel 8 goto end
if errorlevel 7 goto clear_logs
if errorlevel 6 goto open_folder
if errorlevel 5 goto all_startup
if errorlevel 4 goto nav_log
if errorlevel 3 goto spawn_log
if errorlevel 2 goto gazebo_log
if errorlevel 1 goto startup_log

:startup_log
cls
echo.
echo ========================================
echo   Latest Startup Log
echo ========================================
echo.
if not exist "logs\startup_*.log" (
    echo No startup logs found!
    echo Run START_ROBOT.bat first.
    pause
    goto menu
)
for /f "delims=" %%i in ('dir /b /o-d logs\startup_*.log') do (
    echo Showing: %%i
    echo.
    type "logs\%%i"
    goto :done_startup
)
:done_startup
echo.
pause
goto menu

:gazebo_log
cls
echo.
echo ========================================
echo   Gazebo Simulation Log
echo ========================================
echo.
if not exist "logs\gazebo.log" (
    echo No Gazebo log found!
    echo Run START_ROBOT.bat first.
    pause
    goto menu
)
type logs\gazebo.log
echo.
pause
goto menu

:spawn_log
cls
echo.
echo ========================================
echo   Robot Spawn Log
echo ========================================
echo.
if not exist "logs\spawn_robot.log" (
    echo No robot spawn log found!
    echo Run START_ROBOT.bat first.
    pause
    goto menu
)
type logs\spawn_robot.log
echo.
pause
goto menu

:nav_log
cls
echo.
echo ========================================
echo   Autonomous Navigation Log (Live)
echo ========================================
echo.
echo Press Ctrl+C to stop viewing...
echo.
if not exist "logs\autonomous_nav.log" (
    echo No navigation log found!
    echo Run START_ROBOT.bat first.
    pause
    goto menu
)
REM Show last 50 lines and keep updating
powershell -Command "Get-Content logs\autonomous_nav.log -Wait -Tail 50"
goto menu

:all_startup
cls
echo.
echo ========================================
echo   All Startup Logs
echo ========================================
echo.
if not exist "logs\startup_*.log" (
    echo No startup logs found!
    pause
    goto menu
)
dir /b /o-d logs\startup_*.log
echo.
pause
goto menu

:open_folder
start explorer logs
goto menu

:clear_logs
echo.
echo WARNING: This will delete all log files!
choice /C YN /M "Are you sure"
if errorlevel 2 goto menu
if errorlevel 1 (
    if exist "logs\*.log" del /q logs\*.log
    echo All logs cleared!
    timeout /t 2 /nobreak >nul
)
goto menu

:end
echo.
echo Goodbye!
timeout /t 1 /nobreak >nul
exit
