# 🐳 Docker Integration Guide - ROS 2 Multi-Mode Drone Control System

This guide provides comprehensive instructions for running the ROS 2 Multi-Mode Drone Control System using Docker containers.

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Docker Architecture](#docker-architecture)
- [Building Images](#building-images)
- [Running Containers](#running-containers)
- [Development Workflow](#development-workflow)
- [Networking and Tello Connection](#networking-and-tello-connection)
- [GUI Applications](#gui-applications)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

---

## 🔧 Prerequisites

### Linux (Ubuntu 22.04 or similar)

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Install Docker Compose
sudo apt-get update
sudo apt-get install docker-compose-plugin

# Add user to docker group (to run without sudo)
sudo usermod -aG docker $USER
newgrp docker

# Install X11 utilities (for GUI)
sudo apt-get install x11-xserver-utils
```

### Windows

1. **Install Docker Desktop for Windows**
   - Download from: https://www.docker.com/products/docker-desktop
   - Enable WSL2 backend during installation

2. **Install WSL2 with Ubuntu 22.04**
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

3. **For GUI applications (optional)**
   - **Option 1**: Use WSLg (built into Windows 11)
   - **Option 2**: Install VcXsrv
     - Download: https://sourceforge.net/projects/vcxsrv/
     - Configure: Disable access control, enable "Native OpenGL"

### macOS

```bash
# Install Docker Desktop for Mac
# Download from: https://www.docker.com/products/docker-desktop

# Install XQuartz for X11 (for GUI)
brew install --cask xquartz
```

---

## 🚀 Quick Start

### Linux / macOS

```bash
# 1. Navigate to workspace
cd /path/to/ros2_pmld

# 2. Make scripts executable
chmod +x docker/*.sh

# 3. Build Docker image
./docker/docker.sh build

# 4. Run container
./docker/docker.sh run

# 5. Enter container shell
./docker/docker.sh shell

# Inside container: Launch system
ros2 launch tello_control full_system.launch.py
```

### Windows (PowerShell)

```powershell
# 1. Navigate to workspace
cd C:\path\to\ros2_pmld

# 2. Build Docker image
.\docker\docker.ps1 build

# 3. Run container
.\docker\docker.ps1 run

# 4. Enter container shell
.\docker\docker.ps1 shell

# Inside container: Launch system
ros2 launch tello_control full_system.launch.py
```

---

## 🏗️ Docker Architecture

### Multi-Stage Build

The Dockerfile uses a multi-stage build strategy for optimization:

```
┌─────────────────────────────────────────────┐
│  Stage 1: base                              │
│  - Ubuntu 22.04 + ROS 2 Humble Desktop      │
│  - System dependencies                      │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│  Stage 2: dependencies                      │
│  - Python packages (ultralytics, mediapipe) │
│  - Pre-download YOLO models                 │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│  Stage 3: builder                           │
│  - Copy source code                         │
│  - colcon build all packages                │
└──────────────────┬──────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
┌───────▼────────┐   ┌────────▼─────────┐
│  Stage 4:      │   │  Stage 5:        │
│  runtime       │   │  dev             │
│  (production)  │   │  (development)   │
└────────────────┘   └──────────────────┘
```

### Images

- **ros2_pmld:humble** - Production runtime image (~3GB)
  - Optimized for deployment
  - Only includes runtime dependencies
  
- **ros2_pmld:dev** - Development image (~4GB)
  - Includes development tools (gdb, pytest, rqt, rviz2)
  - Suitable for active development

### Containers

Three container profiles are available via docker-compose:

1. **ros2_drone_control** - Main runtime container
2. **ros2_drone_dev** - Development container (profile: dev)
3. **ros2_visualization** - Visualization tools (profile: viz)

---

## 🔨 Building Images

### Build Runtime Image

**Linux/macOS:**
```bash
./docker/docker.sh build
```

**Windows:**
```powershell
.\docker\docker.ps1 build
```

**Manual build:**
```bash
docker build --target runtime -t ros2_pmld:humble -f Dockerfile .
```

### Build Development Image

**Linux/macOS:**
```bash
./docker/docker.sh build-dev
```

**Windows:**
```powershell
.\docker\docker.ps1 build-dev
```

### Build Options

```bash
# Build with no cache (fresh build)
docker build --no-cache -t ros2_pmld:humble .

# Build specific stage
docker build --target dev -t ros2_pmld:dev .

# Build with custom ROS distro
docker build --build-arg ROS_DISTRO=humble -t ros2_pmld:humble .
```

---

## 🎮 Running Containers

### Start Container

**Using helper scripts:**
```bash
# Linux/macOS
./docker/docker.sh run

# Windows
.\docker\docker.ps1 run
```

**Using docker-compose:**
```bash
# Start runtime container
docker-compose up -d ros2_drone_control

# Start development container
docker-compose --profile dev up -d ros2_drone_dev

# Start with visualization
docker-compose --profile viz up -d ros2_visualization
```

### Enter Container Shell

```bash
# Using helper script
./docker/docker.sh shell

# Using docker directly
docker exec -it ros2_drone_control bash
```

### Stop Containers

```bash
# Using helper script
./docker/docker.sh stop

# Using docker-compose
docker-compose down
```

---

## 💻 Development Workflow

### 1. Source Code Editing

Edit source files on your host machine - they're mounted as volumes:

```bash
# Host machine
cd ros2_pmld/src/tello_control/tello_control
nano tracking_controller_node.py
```

### 2. Rebuild Inside Container

```bash
# Inside container
cd /root/ros2_ws
colcon build --symlink-install --packages-select tello_control
source install/setup.bash
```

**Or use helper script:**
```bash
# From host
./docker/docker.sh rebuild
```

### 3. Test Changes

```bash
# Inside container
ros2 run tello_control tracking_controller_node

# Or run tests
colcon test --packages-select tello_control
colcon test-result --verbose
```

### 4. Development Container Features

The development image includes:
- **Debugging tools**: gdb, valgrind
- **Testing tools**: pytest, pytest-cov
- **Code quality**: black, flake8, pylint
- **ROS tools**: rqt, rviz2
- **Aliases**:
  - `build` → `colcon build --symlink-install`
  - `clean` → `rm -rf build install log`
  - `test` → `colcon test && colcon test-result --verbose`
  - `format` → `black src/`
  - `lint` → `flake8 src/`

---

## 🌐 Networking and Tello Connection

### Network Mode: Host

The container uses `network_mode: host` for:
- **DDS Discovery**: ROS 2 nodes can discover each other
- **Tello UDP**: Direct communication with Tello drone (192.168.10.1)

### Connect to Tello Drone

**1. Connect WiFi to Tello**
- SSID: `TELLO-XXXXXX`
- Password: (no password)
- Tello IP: `192.168.10.1`

**2. Verify connection inside container**
```bash
# Inside container
ping 192.168.10.1

# Check if tello_driver can connect
ros2 run tello_driver tello_driver_main
```

**3. Launch with Tello**
```bash
# Inside container
ros2 launch tello_control full_system.launch.py
```

### Firewall Configuration

**Linux (UFW):**
```bash
# Allow DDS discovery
sudo ufw allow 7400:7500/udp

# Allow Tello communication
sudo ufw allow from 192.168.10.0/24
```

**Windows Firewall:**
- Open Windows Defender Firewall
- Allow Docker Desktop
- Allow WSL2 network traffic

---

## 🖥️ GUI Applications

### Linux

```bash
# Allow X11 connections
xhost +local:docker

# Start container (already configured)
./docker/docker.sh run

# Inside container, run GUI app
rviz2
# or
rqt
```

### Windows (WSLg - Windows 11)

```powershell
# WSLg is built-in, just start container
.\docker\docker.ps1 run

# Inside container
rviz2
```

### Windows (VcXsrv - Windows 10)

```powershell
# 1. Start VcXsrv with:
#    - Display number: 0
#    - Disable access control: YES
#    - Native OpenGL: YES

# 2. Get WSL IP
wsl hostname -I

# 3. Set DISPLAY in container
docker exec -it ros2_drone_control bash
export DISPLAY=<WSL_IP>:0.0

# 4. Test
xclock
```

### macOS (XQuartz)

```bash
# 1. Start XQuartz
open -a XQuartz

# 2. In XQuartz preferences, enable "Allow connections from network clients"

# 3. Allow connections
xhost +localhost

# 4. Get IP
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')

# 5. Start container with DISPLAY
docker run -e DISPLAY=$IP:0 -v /tmp/.X11-unix:/tmp/.X11-unix ros2_pmld:humble
```

---

## 🔍 Troubleshooting

### Build Issues

**Problem: "Could not find a version that satisfies the requirement"**
```bash
# Solution: Update pip
docker build --build-arg PIP_VERSION=23.0 .
```

**Problem: "Package not found"**
```bash
# Solution: Update rosdep
docker exec -it ros2_drone_control bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Runtime Issues

**Problem: "Cannot connect to Tello"**
```bash
# Check WiFi connection
ping 192.168.10.1

# Check if Tello SDK is responding
echo -n "command" | nc -u 192.168.10.1 8889
```

**Problem: "ROS nodes can't communicate"**
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Check DDS implementation
echo $RMW_IMPLEMENTATION

# List active nodes
ros2 node list

# Check topic communication
ros2 topic list
ros2 topic echo /cmd_vel
```

**Problem: "GUI not working"**
```bash
# Linux: Check X11
echo $DISPLAY
xhost +local:docker

# Test with simple GUI
xclock

# Check OpenGL
glxinfo | grep "direct rendering"
```

### Container Management

**Problem: "Port already in use"**
```bash
# Find and stop conflicting container
docker ps -a
docker stop <container_id>
```

**Problem: "Volume permission denied"**
```bash
# Fix permissions (Linux)
sudo chown -R $USER:$USER ros2_pmld/

# Or run container with your UID
docker run --user $(id -u):$(id -g) ros2_pmld:humble
```

---

## 🚀 Advanced Usage

### Multi-Container Setup

```bash
# Terminal 1: Main control system
docker-compose up -d ros2_drone_control
docker exec -it ros2_drone_control bash
ros2 launch tello_control full_system.launch.py

# Terminal 2: Visualization
docker-compose --profile viz up -d ros2_visualization

# Terminal 3: Development/debugging
docker-compose --profile dev up -d ros2_drone_dev
docker exec -it ros2_drone_dev bash
```

### Custom Configuration

**Override config files:**
```bash
# Mount custom config
docker run -v $(pwd)/my_config.yaml:/root/ros2_ws/config/tracking.yaml ros2_pmld:humble
```

**Environment variables:**
```bash
# Set custom ROS_DOMAIN_ID
docker run -e ROS_DOMAIN_ID=5 ros2_pmld:humble

# Set Tello IP
docker run -e TELLO_IP=192.168.10.1 ros2_pmld:humble
```

### GPU Support (NVIDIA)

For YOLO inference acceleration:

```bash
# Install nvidia-docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Run with GPU
docker run --gpus all ros2_pmld:humble
```

**Update docker-compose.yml:**
```yaml
services:
  ros2_drone_control:
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

### CI/CD Integration

**GitHub Actions example:**
```yaml
name: Docker Build and Test

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      
      - name: Build Docker image
        run: docker build -t ros2_pmld:humble .
      
      - name: Run tests
        run: |
          docker run ros2_pmld:humble bash -c "
            cd /root/ros2_ws &&
            colcon test &&
            colcon test-result --verbose
          "
```

---

## 📚 Additional Resources

- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **Docker Documentation**: https://docs.docker.com/
- **Tello SDK**: https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf
- **YOLO Ultralytics**: https://docs.ultralytics.com/

---

## 🆘 Getting Help

If you encounter issues:

1. Check logs: `./docker/docker.sh logs`
2. Review this troubleshooting guide
3. Check QUICKSTART.md and ARCHITECTURE.md
4. Open an issue with:
   - Docker version: `docker --version`
   - OS and version
   - Complete error message
   - Steps to reproduce

---

**Happy Drone Flying! 🚁**
