# ROS2 PMLD (Line Following Robot)

A ROS2-based line following robot project that demonstrates autonomous navigation using computer vision and control algorithms.

## Overview

This project implements a line following robot using ROS2, featuring:
- Computer vision-based line detection
- Real-time control algorithms
- Simulation environment support
- Modular node architecture

## Prerequisites

- ROS2 (Humble)
- Python 3.8+
- OpenCV
- Gazebo (for simulation)
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
source scripts/x.sh build
```

4. Setup Environment
```bash
source scripts/init.sh
```

## Quick Start

### Run Simulation
```bash
source scripts/x.sh sim
```

### Launch Line Follower Node
```bash
source scripts/x.sh node
```

### Start Services
```bash
source scripts/x.sh service
```

## Project Structure

```
ros2_pmld/
├── src/                  # ROS2 packages
├── scripts/              # Utility scripts
│   ├── init.sh           # Environment setup
│   ├── setup.sh          # Dependencies instalation
│   └── x.sh              # Main execution script
└── README.md             # This file
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.