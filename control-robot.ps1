# TurtleBot3 Control Script
# This script sends velocity commands to the robot

param(
    [string]$Action = "help"
)

function Send-Velocity {
    param($Linear, $Angular)

    $cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"\"base_link\"\"}, twist: {linear: {x: $Linear, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $Angular}}}'"

    docker exec ros2-turtlebot3-sim bash -c $cmd
}

switch ($Action.ToLower()) {
    "forward" {
        Write-Host "Moving forward..."
        Send-Velocity -Linear 0.2 -Angular 0.0
    }
    "backward" {
        Write-Host "Moving backward..."
        Send-Velocity -Linear -0.2 -Angular 0.0
    }
    "left" {
        Write-Host "Turning left..."
        Send-Velocity -Linear 0.0 -Angular 0.5
    }
    "right" {
        Write-Host "Turning right..."
        Send-Velocity -Linear 0.0 -Angular -0.5
    }
    "stop" {
        Write-Host "Stopping robot..."
        Send-Velocity -Linear 0.0 -Angular 0.0
    }
    "help" {
        Write-Host ""
        Write-Host "TurtleBot3 Robot Control Commands"
        Write-Host "==================================="
        Write-Host ""
        Write-Host "Usage: .\control-robot.ps1 -Action <command>"
        Write-Host ""
        Write-Host "Available commands:"
        Write-Host "  forward  - Move robot forward"
        Write-Host "  backward - Move robot backward"
        Write-Host "  left     - Turn robot left"
        Write-Host "  right    - Turn robot right"
        Write-Host "  stop     - Stop the robot"
        Write-Host ""
        Write-Host "Examples:"
        Write-Host "  .\control-robot.ps1 -Action forward"
        Write-Host "  .\control-robot.ps1 -Action left"
        Write-Host "  .\control-robot.ps1 -Action stop"
        Write-Host ""
    }
    default {
        Write-Host "Unknown action: $Action"
        Write-Host "Run '.\control-robot.ps1 -Action help' for usage information"
    }
}
