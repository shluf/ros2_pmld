#!/bin/bash
# Docker management script for ROS 2 Multi-Mode Drone Control System
# Usage: ./docker/docker.sh [command] [options]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="ros2_pmld"
CONTAINER_NAME="ros2_drone_control"
DOCKERFILE="Dockerfile"

# Helper functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is installed
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        print_warning "docker-compose not found, using 'docker compose' instead"
    fi
}

# Build Docker image
build() {
    print_info "Building Docker image: ${IMAGE_NAME}:humble"
    docker build \
        --target runtime \
        -t ${IMAGE_NAME}:humble \
        -f ${DOCKERFILE} \
        .
    print_success "Image built successfully!"
}

# Build development image
build_dev() {
    print_info "Building development Docker image: ${IMAGE_NAME}:dev"
    docker build \
        --target dev \
        -t ${IMAGE_NAME}:dev \
        -f ${DOCKERFILE} \
        .
    print_success "Development image built successfully!"
}

# Run container
run() {
    print_info "Starting container: ${CONTAINER_NAME}"
    
    # Allow X11 connections
    xhost +local:docker || print_warning "Could not configure X11 (xhost not found)"
    
    docker-compose up -d ros2_drone_control
    print_success "Container started! Use 'docker exec -it ${CONTAINER_NAME} bash' to enter."
}

# Run development container
run_dev() {
    print_info "Starting development container"
    
    xhost +local:docker || print_warning "Could not configure X11 (xhost not found)"
    
    docker-compose --profile dev up -d ros2_drone_dev
    print_success "Development container started!"
}

# Stop container
stop() {
    print_info "Stopping containers"
    docker-compose down
    print_success "Containers stopped!"
}

# Enter container shell
shell() {
    print_info "Entering container shell"
    docker exec -it ${CONTAINER_NAME} bash
}

# View container logs
logs() {
    docker-compose logs -f ros2_drone_control
}

# Clean up (remove containers, images, volumes)
clean() {
    print_warning "This will remove all containers, images, and volumes!"
    read -p "Are you sure? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Cleaning up..."
        docker-compose down -v
        docker rmi ${IMAGE_NAME}:humble ${IMAGE_NAME}:dev 2>/dev/null || true
        print_success "Cleanup complete!"
    else
        print_info "Cleanup cancelled"
    fi
}

# Rebuild workspace inside container
rebuild_workspace() {
    print_info "Rebuilding ROS 2 workspace inside container"
    docker exec -it ${CONTAINER_NAME} bash -c "
        cd /root/ros2_ws && \
        rm -rf build install log && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install
    "
    print_success "Workspace rebuilt!"
}

# Run tests inside container
test() {
    print_info "Running tests inside container"
    docker exec -it ${CONTAINER_NAME} bash -c "
        cd /root/ros2_ws && \
        source install/setup.bash && \
        colcon test && \
        colcon test-result --verbose
    "
}

# Launch full system
launch() {
    print_info "Launching full drone control system"
    docker exec -it ${CONTAINER_NAME} bash -c "
        source /root/ros2_ws/install/setup.bash && \
        ros2 launch tello_control full_system.launch.py
    "
}

# Show help
show_help() {
    cat << EOF
🚁 ROS 2 Multi-Mode Drone Control System - Docker Management Script

Usage: $0 [command]

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
    $0 build        # Build the image
    $0 run          # Start container
    $0 shell        # Enter container
    $0 launch       # Launch drone system

For more information, see DOCKER.md
EOF
}

# Main script logic
check_docker

case "$1" in
    build)
        build
        ;;
    build-dev)
        build_dev
        ;;
    run)
        run
        ;;
    run-dev)
        run_dev
        ;;
    stop)
        stop
        ;;
    shell)
        shell
        ;;
    logs)
        logs
        ;;
    clean)
        clean
        ;;
    rebuild)
        rebuild_workspace
        ;;
    test)
        test
        ;;
    launch)
        launch
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
