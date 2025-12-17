# PowerShell Script to Run ROS 2 Robotics Tasks
# This script automates the Docker setup and launch process

Write-Host "======================================" -ForegroundColor Cyan
Write-Host "  ROS 2 Robotics Tasks - Docker Setup" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Check if Docker is installed
Write-Host "Checking Docker installation..." -ForegroundColor Yellow
try {
    $dockerVersion = docker --version
    Write-Host "✓ Docker found: $dockerVersion" -ForegroundColor Green
} catch {
    Write-Host "✗ Docker not found!" -ForegroundColor Red
    Write-Host "Please install Docker Desktop from: https://www.docker.com/products/docker-desktop/" -ForegroundColor Yellow
    exit 1
}

# Check if Docker is running
Write-Host "Checking if Docker is running..." -ForegroundColor Yellow
try {
    docker ps | Out-Null
    Write-Host "✓ Docker is running" -ForegroundColor Green
} catch {
    Write-Host "✗ Docker is not running!" -ForegroundColor Red
    Write-Host "Please start Docker Desktop" -ForegroundColor Yellow
    exit 1
}

# Set DISPLAY environment variable
Write-Host ""
Write-Host "Setting up X11 display..." -ForegroundColor Yellow

# Get Windows IP for DISPLAY
$ip = (Get-NetIPAddress -AddressFamily IPv4 -InterfaceAlias "Wi-Fi*","Ethernet*" | Select-Object -First 1).IPAddress

if ($ip) {
    $env:DISPLAY = "${ip}:0.0"
    Write-Host "✓ DISPLAY set to: $env:DISPLAY" -ForegroundColor Green
    Write-Host "  Make sure VcXsrv is running!" -ForegroundColor Yellow
} else {
    $env:DISPLAY = ":0"
    Write-Host "✓ DISPLAY set to: $env:DISPLAY (WSLg)" -ForegroundColor Green
}

# Build Docker image
Write-Host ""
$build = Read-Host "Do you want to build/rebuild the Docker image? (y/n)"
if ($build -eq "y" -or $build -eq "Y") {
    Write-Host "Building Docker image (this may take 10-15 minutes)..." -ForegroundColor Yellow
    docker-compose build
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Docker image built successfully" -ForegroundColor Green
    } else {
        Write-Host "✗ Docker build failed" -ForegroundColor Red
        exit 1
    }
}

# Start container
Write-Host ""
Write-Host "Starting Docker container..." -ForegroundColor Yellow
docker-compose up -d

if ($LASTEXITCODE -eq 0) {
    Write-Host "✓ Container started" -ForegroundColor Green
} else {
    Write-Host "✗ Failed to start container" -ForegroundColor Red
    exit 1
}

# Show next steps
Write-Host ""
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "  Container is running!" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Make sure VcXsrv/X Server is running" -ForegroundColor White
Write-Host "   (Download from: https://sourceforge.net/projects/vcxsrv/)" -ForegroundColor Gray
Write-Host ""
Write-Host "2. Enter container:" -ForegroundColor White
Write-Host "   docker exec -it ros2-turtlebot3-sim /bin/bash" -ForegroundColor Cyan
Write-Host ""
Write-Host "3. Inside container, run simulation:" -ForegroundColor White
Write-Host "   cd /root/robotics-tasks/task1" -ForegroundColor Cyan
Write-Host "   ./scripts/launch_task1.sh" -ForegroundColor Cyan
Write-Host ""
Write-Host "4. Open another terminal and run teleop:" -ForegroundColor White
Write-Host "   docker exec -it ros2-turtlebot3-sim /bin/bash" -ForegroundColor Cyan
Write-Host "   cd /root/robotics-tasks/task1" -ForegroundColor Cyan
Write-Host "   ./scripts/teleop_control.sh" -ForegroundColor Cyan
Write-Host ""
Write-Host "To stop the container:" -ForegroundColor Yellow
Write-Host "   docker-compose down" -ForegroundColor Cyan
Write-Host ""
Write-Host "Happy Robotics! 🤖" -ForegroundColor Green
