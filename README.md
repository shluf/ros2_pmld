# ROS2 PMLD (Tello Drone Control)

A ROS2-based Tello drone control project featuring gesture control, keyboard control, Gazebo simulation, and GUI interface.

## Overview

This project implements a comprehensive control system for Tello/Tello EDU drones using ROS2, featuring:

- **Interactive Menu System** - Easy-to-use script for launching all modes
- **Gesture Control** - Control drone with hand gestures using MediaPipe + TFLite
- **Keyboard Control** - Manual control with full 6-DOF movement
- **GUI Interface** - PyQt5-based graphical control panel
- **Gazebo Simulation** - Safe testing environment
- **Real Drone Support** - Tested with Tello/Tello EDU
- **Debug Modes** - Multiple testing configurations
- **Safety Features** - Gesture hold time, confirmations, emergency stop

## Prerequisites

- ROS2 (Foxy/Humble)
- Python 3.8+
- Gazebo Classic (gazebo11)
- ROS2 packages: `gazebo_ros_pkgs`, `robot_state_publisher`, `joy`
- Python packages: `opencv-python`, `mediapipe`, `tensorflow-lite`, `PyQt5`
- Colcon build tools

## Installation

1. **Clone the repository:**
```bash
cd ~/
git clone https://github.com/shluf/ros2_pmld.git
cd ros2_pmld
```

2. **Install dependencies:**
```bash
source scripts/setup.sh
```

3. **Build the workspace:**
```bash
colcon build --symlink-install
# Or use the menu
./scripts/x.sh
# Then choose option 1
```

4. **Source the environment:**
```bash
source scripts/init.sh
```

## Quick Start

### Method 1: Interactive Menu
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

### Method 2: Quick Commands
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

## Gesture Controls

Control the drone with hand gestures!

| Gesture | Action | Description |
|---------|--------|-------------|
| ✋ **Open Hand** (5 fingers) | Move Forward | Drone moves forward |
| 👆 **Pointer** (1 finger) | Rotate | Drone rotates based on hand position |
| ✊ **Closed Fist** | Move Backward | Drone moves backward |
| 👌 **OK Sign** | Land | Drone lands safely |


### Gesture Modes

| Mode | Safety | Hold Time | Debug | Best For |
|------|--------|-----------|-------|----------|
| **Gazebo Debug** | ✗ | 0.5s | ✓ | Fast testing & development |
| **Gazebo Production** | ✓ | 1.0s | ✗ | Pre-flight testing |
| **Real Drone** | ✓ | 1.0s | ✗ | Actual flying |

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