# ROS2 PMLD (Tello Drone Multi-Mode Control)

A comprehensive ROS2-based Tello drone control system featuring multi-mode operation (Manual, Gesture, Autonomous Tracking), advanced perception, and Docker integration.

## 🎯 Overview

This project implements an advanced multi-mode control system for Tello/Tello EDU drones using ROS 2 Humble, featuring:

### **Multi-Mode Control System (NEW)**
- **Manual Mode** - Keyboard/GUI control with full 6-DOF movement
- **Gesture Mode** - Hand gesture control using MediaPipe
- **Tracking Mode** - Autonomous object tracking with YOLO + PID control
- **Graceful Mode Switching** - Automatic hover transitions between modes

### **Advanced Perception (NEW)**
- **YOLO Object Detection** - Real-time object detection with YOLOv8
- **ArUco Marker Detection** - Reference marker for distance estimation
- **Distance Measurement** - Pixel-to-metric conversion for spatial awareness

### **Original Features**
- **Interactive Menu System** - Easy-to-use script for launching all modes
- **Gesture Control** - Control drone with hand gestures using MediaPipe
- **GUI Interface** - PyQt5-based graphical control panel
- **Gazebo Simulation** - Safe testing environment
- **Real Drone Support** - Tested with Tello/Tello EDU

### **🐳 Docker Integration (NEW)**
- **Ubuntu 22.04 + ROS 2 Humble** - Fully containerized environment
- **Multi-stage builds** - Optimized images for production and development
- **Easy deployment** - One-command build and run
- **Cross-platform** - Works on Linux, Windows (WSL2), and macOS

## 📋 Prerequisites

### Option 1: Docker (Recommended - Easiest Setup)

- **Docker Desktop** (Windows/macOS) or **Docker Engine** (Linux)
- **Docker Compose** (included with Docker Desktop)
- For GUI: **WSLg** (Windows 11) or **VcXsrv** (Windows 10) or **XQuartz** (macOS)

**See [DOCKER.md](DOCKER.md) for detailed Docker setup instructions.**

### Option 2: Native Installation

- **ROS 2 Humble** (Ubuntu 22.04)
- **Python 3.10+**
- **Gazebo Classic** (gazebo11)
- **ROS 2 packages**: `gazebo_ros_pkgs`, `robot_state_publisher`, `cv_bridge`, `image_transport`
- **Python packages**: `ultralytics`, `opencv-contrib-python`, `mediapipe`, `PyQt5`
- **Colcon build tools**

## 🚀 Installation

### 🐳 Docker Installation (Recommended)

#### Linux/macOS
```bash
cd ~/ros2_pmld

# Build Docker image
./docker/docker.sh build

# Run container
./docker/docker.sh run

# Enter container shell
./docker/docker.sh shell

# Inside container: Launch multi-mode system
ros2 launch tello_control full_system.launch.py
```

#### Windows (PowerShell)
```powershell
cd C:\path\to\ros2_pmld

# Build Docker image
.\docker\docker.ps1 build

# Run container
.\docker\docker.ps1 run

# Enter container shell
.\docker\docker.ps1 shell

# Inside container: Launch multi-mode system
ros2 launch tello_control full_system.launch.py
```

**📖 For complete Docker documentation, see [DOCKER.md](DOCKER.md)**

---

### 🔧 Native Installation (Alternative)

1. **Clone the repository:**
```bash
cd ~/
git clone https://github.com/shluf/ros2_pmld.git
cd ros2_pmld
```

2. **Install dependencies:**
```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Install Python packages
pip install ultralytics opencv-contrib-python mediapipe PyQt5
```

3. **Build the workspace:**
```bash
# Use automated build script
./build_system.ps1  # Windows
# OR
python build_system.py  # Cross-platform

# Or build manually
colcon build --symlink-install
```

4. **Source the environment:**
```bash
source install/setup.bash
```

## 🎮 Quick Start

### Multi-Mode Control System

```bash
# Launch full multi-mode system
ros2 launch tello_control full_system.launch.py initial_mode:=manual

# Switch between modes via topic
ros2 topic pub /mode_switch std_msgs/String "data: 'tracking'" --once
ros2 topic pub /mode_switch std_msgs/String "data: 'gesture'" --once
ros2 topic pub /mode_switch std_msgs/String "data: 'manual'" --once

# Monitor current mode
ros2 topic echo /control_mode

# Launch only perception system
ros2 launch tello_perception perception.launch.py

# Launch only control system
ros2 launch tello_control control_system.launch.py
```

** For complete usage guide, see [QUICKSTART.md](docs/QUICKSTART.md)**

---

### Original Menu System

#### Method 1: Interactive Menu
```bash
./scripts/x.sh
```

**Menu Options:**
```
================================================
          ROS2 Tello Control Menu               
================================================

  [SETUP]
  1) Build workspace

  [SIMULATION]
  2) Gazebo simulation only
  3) Gesture Control (Gazebo + Debug)
  4) Gesture Control (Gazebo + Production)

  [GUI CONTROL]
  5) Tello Control GUI

  [REAL DRONE]
  6) Connect to REAL drone
  7) Gesture Control (Real Drone)

  0) Exit

```

#### Method 2: Quick Commands
```bash

# Build workspace
./scripts/x.sh build

# Test gestures in Gazebo (debug mode - fast iteration)
./scripts/x.sh gd

# Test gestures in Gazebo (production mode - safe settings)
./scripts/x.sh gp

# Launch GUI
./scripts/x.sh gui

# Connect to real drone
./scripts/x.sh real

# Gesture control with real drone
./scripts/x.sh gr

# Show all commands
./scripts/x.sh help
```

### Method 3: Direct ROS2 Launch
```bash
# Gesture control - Gazebo Debug
ros2 launch gesture_control debug_gazebo_launch.py

# Gesture control - Custom parameters
ros2 launch gesture_control gesture_control_launch.py \
    namespace:=drone1 \
    use_drone_camera:=true \
    debug_mode:=false \
    enable_safety:=true \
    simulation:=true

# GUI with simulation
ros2 launch tello_control_gui tello_gui_launch.py \
    with_gesture:=true \
    simulation:=true

# GUI with real drone
ros2 launch tello_control_gui tello_gui_launch.py \
    with_driver:=true \
    with_gesture:=true \
    simulation:=false
```

## Package Structure

### New Multi-Mode Control Packages

- **tello_interfaces/** - Custom ROS 2 message definitions
  - `Detection.msg`, `DetectionArray.msg` - YOLO detection results
  - `ObjectDistance.msg`, `ObjectDistanceArray.msg` - Distance measurements
  - `ControlMode.msg` - Current control mode status

- **tello_perception/** - Perception layer (NEW)
  - `yolo_detector_node` - YOLOv8 object detection
  - `aruco_detector_node` - ArUco marker detection
  - `distance_estimator_node` - Pixel-to-metric distance estimation

- **tello_control/** - Multi-mode control layer (UPDATED)
  - `mode_manager_node` - Mode switching with graceful transitions
  - `control_arbitrator_node` - Command multiplexing
  - **control_modes/** - Specific controllers:
    - `tracking_controller_node` - PID-based autonomous tracking
    - `gesture_control_node` - Gesture-based control logic
    - `keyboard_controller` - Keyboard teleoperation
    - `joy_controller_node` - Joystick control (Logitech Extreme 3D)

### Original Packages

- **gesture_control/** - Hand gesture recognition (UPDATED)
  - `gesture_detector_node` - Detects gestures and publishes status (Perception only)
  
- **tello_control_gui/** - PyQt5 GUI interface
- **tello_ros/** - Tello driver interface
- **tello_activation/** - Tello service activation

---

## Documentation

- **[DOCKER.md](docs/DOCKER.md)** - Complete Docker integration guide
- **[QUICKSTART.md](docs/QUICKSTART.md)** - Quick start and usage guide
- **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System architecture and design
- **[IMPLEMENTATION_SUMMARY.md](docs/IMPLEMENTATION_SUMMARY.md)** - Implementation details

---

## Configuration

### Multi-Mode System Configuration

**Perception config** (`config/perception.yaml`):
```yaml
yolo_detector:
  model_name: yolov8n  # or yolov8s
  confidence_threshold: 0.5
  device: cpu  # or cuda

aruco_detector:
  marker_size: 0.10  # meters
  camera_matrix: [...]
```

**Control config** (`config/tracking.yaml`):
```yaml
tracking_controller:
  pid_gains:
    x: {kp: 0.5, ki: 0.0, kd: 0.1}
    y: {kp: 0.5, ki: 0.0, kd: 0.1}
    z: {kp: 0.5, ki: 0.0, kd: 0.1}
  target_distance: 1.5  # meters
```

---

## Gesture Controls

Control the drone with hand gestures!

| Gesture | Action | Description |
|---------|--------|-------------|
| ✋ **Open Hand** (5 fingers) | Move Forward | Drone moves forward |
| 👆 **Pointer** (1 finger) | Rotate | Drone rotates based on hand position |
| ✊ **Closed Fist** | Move Backward | Drone moves backward |
| 👌 **OK Sign** | Land | Drone lands safely |

## Keyboard Controls

Control the drone with keyboard!

### Movement (6-DOF)
| Key | Action |
|-----|--------|
| W/S | Forward/Backward |
| A/D | Left/Right |
| Q/E | Yaw left/right |
| I/K | Up/Down |
| SPACE | Stop/Hover |

### Commands
| Key | Action |
|-----|--------|
| T | Takeoff |
| L | Land |
| H | Emergency Stop |

### Exit
| Key | Action |
|-----|--------|
| ESC | Exit controller |

For detailed keyboard controls, see: [Keyboard Controller README](src/tello_keyboard/README.md)

## Project Structure

```
ros2_pmld/
├── src/
│   ├── ros2_shared/          # Shared ROS2 components
│   ├── gesture_control/      # Gesture recognition & control
│   │   ├── gesture_control/
│   │   │   ├── gesture_controller.py      # Main controller node
│   │   │   ├── gesture_recognition.py     # MediaPipe + TFLite
│   │   │   ├── keypoint_classifier.py     # Static gesture classifier
│   │   │   └── point_history_classifier.py # Dynamic gesture classifier
│   │   ├── model/            # TFLite models
│   │   ├── config/           # Gesture mapping config
│   │   └── launch/           # Launch files
│   ├── tello_control_gui/    # PyQt5 GUI interface
│   ├── tello_keyboard/       # Keyboard controller
│   └── tello_ros/            # Original Tello ROS2 driver
│       ├── tello_driver/     # C++ driver node
│       ├── tello_msgs/       # Message definitions
│       ├── tello_description/# Robot URDF files
│       └── tello_gazebo/     # Gazebo simulation
├── scripts/
│   ├── init.sh              # Environment setup
│   ├── setup.sh             # Dependencies installation
│   ├── x.sh                 # Main interactive menu
│   ├── connect_tello.sh     # Real drone connection
│   └── kill_gazebo.sh       # Cleanup script
├── build/                   # Build artifacts
├── install/                 # Installed packages
├── log/                     # Build logs
└── README.md                # This file
```

## Topics & Services

### Published Topics
- `/drone1/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `/gesture_recognition/detected_gesture` (std_msgs/String) - Detected gesture status

### Subscribed Topics
- `/drone1/flight_data` (tello_msgs/FlightData) - Telemetry data
- `/drone1/image_raw` (sensor_msgs/Image) - Camera feed for gesture detection

### Services
- `/drone1/tello_action` (tello_msgs/TelloAction) - Command execution (takeoff, land, etc.)



## References

- [Tello ROS Original](https://github.com/clydemcqueen/tello_ros)
- [Tello SDK Documentation](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic](http://gazebosim.org/)

## License

This project is licensed under the Apache-2.0 License - see the LICENSE file for details.