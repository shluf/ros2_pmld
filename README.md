# ROS2 PMLD (Tello Drone Control)

A ROS2-based Tello drone control project featuring keyboard control, Gazebo simulation, and autonomous flight capabilities.

## Overview

This project implements a comprehensive control system for Tello/Tello EDU drones using ROS2, featuring:
- Keyboard-based manual control with full 6-DOF movement
- Gazebo simulation environment for safe testing
- Service integration for drone commands (takeoff, land, emergency stop)
- Flip maneuvers with safety confirmation
- Real-time speed adjustment
- Modular ROS2 package architecture

## Prerequisites

- ROS2 (Foxy/Humble)
- Python 3.0+
- Gazebo Classic (gazebo11)
- ROS2 packages:
  - `gazebo_ros_pkgs`
  - `robot_state_publisher`
  - `joy` (for joystick support)
- Colcon build tools

## Installation

1. Clone the repository:
```bash
cd ~/
git clone https://github.com/shluf/ros2_pmld.git
cd ros2_pmld
```

2. Install dependencies:
```bash
source scripts/setup.sh
```

3. Build the workspace:
```bash
colcon build --symlink-install
# Or use the helper script
source scripts/x.sh build
```

4. Source the environment:
```bash
source scripts/init.sh
```

## Quick Start

### Interactive Menu
```bash
source scripts/x.sh
```

### Command Line Options

**Build Workspace:**
```bash
source scripts/x.sh build
```

**Run Gazebo Simulation:**
```bash
source scripts/x.sh sim
# Or with specific port
source scripts/x.sh rsim 11345
```

**Run Keyboard Controller:**
```bash
source scripts/x.sh keyboard
# Or directly
ros2 run tello_keyboard keyboard_controller --ros-args -p namespace:=drone1
```

## Keyboard Controls

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
│   ├── tello_ros/            # Original Tello ROS2 driver
│   │   ├── tello_driver/     # C++ driver node
│   │   ├── tello_msgs/       # Message definitions
│   │   ├── tello_description/# Robot URDF files
│   │   └── tello_gazebo/     # Gazebo simulation
│   └── tello_keyboard/       # Keyboard controller (NEW)
│       ├── tello_keyboard/
│       │   └── keyboard_controller.py
│       └── launch/
│           ├── keyboard_launch.py
│           └── sim_keyboard_launch.py
├── scripts/
│   ├── init.sh              # Environment setup
│   ├── setup.sh             # Dependencies installation
│   ├── x.sh                 # Main execution menu
│   └── kill_gazebo.sh       # Cleanup script
├── build/                   # Build artifacts
├── install/                 # Installed packages
├── log/                     # Build logs
└── README.md                # This file
```

## Topics & Services

### Published Topics
- `/drone1/cmd_vel` (geometry_msgs/Twist) - Velocity commands

### Subscribed Topics
- `/drone1/flight_data` (tello_msgs/FlightData) - Telemetry data

### Services
- `/drone1/tello_action` (tello_msgs/TelloAction) - Command execution


## Development

### Adding New Features
1. Create new package in `src/`
2. Update dependencies in `package.xml`
3. Build: `colcon build --packages-select <package_name>`
4. Source: `source install/setup.bash`


## References

- [Tello ROS Original](https://github.com/clydemcqueen/tello_ros)
- [Tello SDK Documentation](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic](http://gazebosim.org/)

## License

This project is licensed under the Apache-2.0 License - see the LICENSE file for details.