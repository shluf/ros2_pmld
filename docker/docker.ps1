# Docker management script for Windows PowerShell
# Usage: .\docker\docker.ps1 [command] [options]

param(
    [Parameter(Position=0)]
    [ValidateSet('build', 'build-dev', 'run', 'run-dev', 'stop', 'shell', 'logs', 'clean', 'rebuild', 'test', 'launch', 'help')]
    [string]$Command = 'help'
)

# Configuration
$ImageName = "ros2_pmld"
$ContainerName = "ros2_drone_control"
$Dockerfile = "Dockerfile"

# Helper functions
function Write-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Blue
}

function Write-Success {
    param([string]$Message)
    Write-Host "[SUCCESS] $Message" -ForegroundColor Green
}

function Write-Warning {
    param([string]$Message)
    Write-Host "[WARNING] $Message" -ForegroundColor Yellow
}

function Write-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

# Check if Docker is installed
function Test-Docker {
    try {
        $null = docker --version
    } catch {
        Write-Error "Docker is not installed. Please install Docker Desktop for Windows."
        exit 1
    }
}

# Build Docker image
function Build-Image {
    Write-Info "Building Docker image: ${ImageName}:humble"
    docker build --target runtime -t "${ImageName}:humble" -f $Dockerfile .
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Image built successfully!"
    } else {
        Write-Error "Build failed!"
        exit 1
    }
}

# Build development image
function Build-DevImage {
    Write-Info "Building development Docker image: ${ImageName}:dev"
    docker build --target dev -t "${ImageName}:dev" -f $Dockerfile .
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Development image built successfully!"
    } else {
        Write-Error "Build failed!"
        exit 1
    }
}

# Run container
function Start-Container {
    Write-Info "Starting container: $ContainerName"
    
    # Set DISPLAY for WSL2 (if using WSLg)
    $env:DISPLAY = ":0"
    
    docker-compose up -d ros2_drone_control
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Container started! Use 'docker exec -it $ContainerName bash' to enter."
        Write-Info "For GUI applications, ensure WSLg or VcXsrv is running."
    } else {
        Write-Error "Failed to start container!"
        exit 1
    }
}

# Run development container
function Start-DevContainer {
    Write-Info "Starting development container"
    
    $env:DISPLAY = ":0"
    
    docker-compose --profile dev up -d ros2_drone_dev
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Development container started!"
    } else {
        Write-Error "Failed to start container!"
        exit 1
    }
}

# Stop container
function Stop-Container {
    Write-Info "Stopping containers"
    docker-compose down
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Containers stopped!"
    }
}

# Enter container shell
function Enter-Shell {
    Write-Info "Entering container shell"
    docker exec -it $ContainerName bash
}

# View container logs
function Show-Logs {
    docker-compose logs -f ros2_drone_control
}

# Clean up
function Remove-All {
    Write-Warning "This will remove all containers, images, and volumes!"
    $confirmation = Read-Host "Are you sure? (y/N)"
    
    if ($confirmation -eq 'y' -or $confirmation -eq 'Y') {
        Write-Info "Cleaning up..."
        docker-compose down -v
        docker rmi "${ImageName}:humble" "${ImageName}:dev" 2>$null
        Write-Success "Cleanup complete!"
    } else {
        Write-Info "Cleanup cancelled"
    }
}

# Rebuild workspace
function Rebuild-Workspace {
    Write-Info "Rebuilding ROS 2 workspace inside container"
    
    $rebuildScript = @"
cd /root/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
"@
    
    docker exec -it $ContainerName bash -c $rebuildScript
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Workspace rebuilt!"
    } else {
        Write-Error "Rebuild failed!"
    }
}

# Run tests
function Invoke-Tests {
    Write-Info "Running tests inside container"
    
    $testScript = @"
cd /root/ros2_ws
source install/setup.bash
colcon test
colcon test-result --verbose
"@
    
    docker exec -it $ContainerName bash -c $testScript
}

# Launch system
function Start-Launch {
    Write-Info "Launching full drone control system"
    
    $launchScript = @"
source /root/ros2_ws/install/setup.bash
ros2 launch tello_control full_system.launch.py
"@
    
    docker exec -it $ContainerName bash -c $launchScript
}

# Show help
function Show-Help {
    Write-Host @"
🚁 ROS 2 Multi-Mode Drone Control System - Docker Management Script (Windows)

Usage: .\docker\docker.ps1 [command]

Commands:
    build           Build runtime Docker image
    build-dev       Build development Docker image
    run             Start runtime container
    run-dev         Start development container
    stop            Stop all containers
    shell           Open bash shell in container
    logs            View container logs
    clean           Remove containers, images, and volumes
    rebuild         Rebuild ROS 2 workspace inside container
    test            Run tests inside container
    launch          Launch full drone control system
    help            Show this help message

Examples:
    .\docker\docker.ps1 build        # Build the image
    .\docker\docker.ps1 run          # Start container
    .\docker\docker.ps1 shell        # Enter container
    .\docker\docker.ps1 launch       # Launch drone system

Prerequisites:
    - Docker Desktop for Windows
    - WSL2 with Ubuntu 22.04 (optional, for GUI)
    - VcXsrv or WSLg for X11 forwarding (for GUI applications)

For more information, see DOCKER.md
"@
}

# Main script logic
Test-Docker

switch ($Command) {
    'build' { Build-Image }
    'build-dev' { Build-DevImage }
    'run' { Start-Container }
    'run-dev' { Start-DevContainer }
    'stop' { Stop-Container }
    'shell' { Enter-Shell }
    'logs' { Show-Logs }
    'clean' { Remove-All }
    'rebuild' { Rebuild-Workspace }
    'test' { Invoke-Tests }
    'launch' { Start-Launch }
    'help' { Show-Help }
    default { 
        Write-Error "Unknown command: $Command"
        Show-Help
        exit 1
    }
}
