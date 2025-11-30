# Tello Multi-Mode Control System - Quick Start Guide

## 🐳 Docker Quick Start (Recommended)

**For detailed Docker instructions, see [DOCKER.md](DOCKER.md)**

### Linux/macOS
```bash
# Build and run with Docker
./docker/docker.sh build
./docker/docker.sh run
./docker/docker.sh shell

# Inside container
ros2 launch tello_control full_system.launch.py
```

### Windows (PowerShell)
```powershell
# Build and run with Docker
.\docker\docker.ps1 build
.\docker\docker.ps1 run
.\docker\docker.ps1 shell

# Inside container
ros2 launch tello_control full_system.launch.py
```

---

## 🔧 Native Installation (Alternative)

### 1. Install Dependencies

```bash
# Python packages
pip install ultralytics opencv-contrib-python mediapipe

# ROS 2 dependencies (if not already installed)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

### 2. Build Packages

```bash
cd ~/ros2_pmld

# Build new packages in order
colcon build --packages-select tello_interfaces
colcon build --packages-select tello_perception
colcon build --packages-select tello_control
colcon build --packages-select gesture_control

# Or use build script
./build_system.ps1  # Windows
# OR
python build_system.py  # Cross-platform

# Source workspace
source install/setup.bash
```

## 🎮 Usage

### Quick Start: Full System

```bash
# Terminal 1: Launch full system
source install/setup.bash
ros2 launch tello_control full_system.launch.py initial_mode:=manual

# Terminal 2: Launch Tello driver (if not auto-launched)
ros2 run tello_ros tello_driver_node
```

### Individual Components

#### Perception Only
```bash
ros2 launch tello_perception perception.launch.py model_name:=yolov8n marker_size:=0.10
```

#### Control Only
```bash
ros2 launch tello_control control.launch.py initial_mode:=manual hover_duration:=1.0
```

## 🔄 Mode Switching

### Switch to Tracking Mode
```bash
ros2 topic pub /mode_switch std_msgs/String "data: 'tracking'" --once
```

### Switch to Gesture Mode
```bash
ros2 topic pub /mode_switch std_msgs/String "data: 'gesture'" --once
```

### Switch to Manual Mode
```bash
ros2 topic pub /mode_switch std_msgs/String "data: 'manual'" --once
```

### Change Camera Source (for Gesture Mode)
```bash
# Switch to webcam
ros2 topic pub /camera_switch std_msgs/String "data: 'webcam'" --once

# Switch to drone camera
ros2 topic pub /camera_switch std_msgs/String "data: 'drone'" --once
```

## 📊 Monitoring

### Check Current Mode
```bash
ros2 topic echo /control_mode
```

### View Detections
```bash
# YOLO detections
ros2 topic echo /detections

# ArUco markers
ros2 topic echo /aruco_poses

# Distance measurements
ros2 topic echo /object_distances
```

### View Control Commands
```bash
# Final cmd_vel to drone
ros2 topic echo /cmd_vel

# Individual sources
ros2 topic echo /manual/cmd_vel
ros2 topic echo /gesture/cmd_vel
ros2 topic echo /tracking/cmd_vel
```

### View Annotated Images
```bash
# YOLO detections
ros2 run rqt_image_view rqt_image_view /detections/annotated

# ArUco markers
ros2 run rqt_image_view rqt_image_view /aruco/annotated
```

## 🧪 Testing Individual Nodes

### Test YOLO Detector
```bash
ros2 run tello_perception yolo_detector_node --ros-args \
  -p model_name:=yolov8n \
  -p confidence_threshold:=0.5 \
  -p target_classes:="['person', 'bottle']"
```

### Test ArUco Detector
```bash
ros2 run tello_perception aruco_detector_node --ros-args \
  -p marker_size:=0.10 \
  -p aruco_dict:=DICT_4X4_50
```

### Test Mode Manager
```bash
ros2 run tello_control mode_manager_node --ros-args \
  -p initial_mode:=manual \
  -p hover_duration:=1.0
```

### Test Tracking Controller
```bash
ros2 run tello_control tracking_controller_node --ros-args \
  -p target_class:=person \
  -p deadzone_pixels:=50
```

### Test Control Arbitrator
```bash
ros2 run tello_control control_arbitrator_node --ros-args \
  -p command_timeout:=0.5 \
  -p publish_rate:=20.0
```

## 🎯 Typical Workflow

### 1. Setup and Takeoff
```bash
# Start full system
ros2 launch tello_control full_system.launch.py

# Takeoff via service (or manual keyboard control)
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
```

### 2. Test Gesture Mode
```bash
# Switch to gesture mode with drone camera
ros2 topic pub /mode_switch std_msgs/String "data: 'gesture'" --once

# Or with webcam for testing
ros2 topic pub /camera_switch std_msgs/String "data: 'webcam'" --once
ros2 topic pub /mode_switch std_msgs/String "data: 'gesture'" --once
```

### 3. Test Tracking Mode
```bash
# Place ArUco marker in view (for distance measurement)
# Ensure target object (e.g., person) is in frame

# Switch to tracking mode (will hover for 1s, then start tracking)
ros2 topic pub /mode_switch std_msgs/String "data: 'tracking'" --once

# Monitor tracking
ros2 topic echo /tracking/cmd_vel
```

### 4. Land
```bash
# Switch to manual mode first (recommended)
ros2 topic pub /mode_switch std_msgs/String "data: 'manual'" --once

# Land via service
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
```

## 🔧 Configuration

### Perception Config
Edit `src/tello_perception/config/perception_config.yaml`:
- YOLO model selection
- Detection confidence threshold
- ArUco marker size and dictionary
- Camera calibration

### Control Config
Edit `src/tello_control/config/control_config.yaml`:
- PID gains for X, Y, Z, Yaw axes
- Velocity limits
- Tracking parameters
- Mode manager settings

### Gesture Config
Edit `src/gesture_control/config/gesture_mapping.yaml`:
- Hand gesture to command mapping
- Velocity settings
- Safety parameters

## 📝 Notes

### ArUco Marker Setup
1. Print ArUco marker from DICT_4X4_50 (ID 0-49)
2. Measure exact marker size (default: 10cm)
3. Update `marker_size` parameter if different
4. Place marker in view for distance measurements

### YOLO Model Selection
- **yolov8n**: Fastest, lowest accuracy (recommended for Tello)
- **yolov8s**: Balanced speed/accuracy
- **yolov8m/l/x**: Higher accuracy, slower (may lag on Tello feed)

### Performance Tips
- Use `yolov8n` for real-time performance
- Reduce `publish_rate` in control_arbitrator if experiencing lag
- Adjust `deadzone_pixels` in tracking controller for stability
- Increase `hover_duration` for safer mode transitions

## 🐛 Troubleshooting

### No detections appearing
```bash
# Check if YOLO model is loaded
ros2 topic echo /detections

# View annotated image to debug
ros2 run rqt_image_view rqt_image_view /detections/annotated
```

### Mode not switching
```bash
# Check current mode
ros2 topic echo /control_mode

# Verify mode_manager is running
ros2 node list | grep mode_manager
```

### Gesture control not working
```bash
# Verify gesture mode is active
ros2 topic echo /control_mode

# Check gesture commands
ros2 topic echo /gesture/cmd_vel

# Verify camera source
ros2 param get /gesture_controller camera_source
```

### Tracking unstable
```bash
# Increase deadzone
ros2 param set /tracking_controller deadzone_pixels 100

# Reduce PID gains
ros2 param set /tracking_controller pid_x.kp 0.3
```

## 📚 See Also

- Full architecture: `ARCHITECTURE.md`
- Package READMEs:
  - `src/tello_perception/README.md`
  - `src/tello_control/README.md`
  - `src/gesture_control/README.md`
