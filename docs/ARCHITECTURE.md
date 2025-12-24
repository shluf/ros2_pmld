# ARSITEKTUR SISTEM TELLO MULTI-MODE CONTROL

## Overview
Sistem kontrol drone Tello dengan 3 mode operasi:
- **Manual Mode**: Kontrol manual via keyboard/joystick
- **Gesture Mode**: Kontrol via hand gestures (MediaPipe)
- **Tracking Mode**: Autonomous object tracking (YOLO + PID)

## Package Structure

```
ros2_pmld/src/
├── tello_interfaces/          # Custom ROS 2 messages
│   └── msg/
│       ├── Detection.msg
│       ├── DetectionArray.msg
│       ├── ObjectDistance.msg
│       ├── ObjectDistanceArray.msg
│       └── ControlMode.msg
│
├── tello_perception/          # Perception nodes
│   └── tello_perception/
│       ├── yolo_detector_node.py
│       ├── aruco_detector_node.py
│       └── distance_estimator_node.py
│
├── tello_control/             # Control & mode management
│   └── tello_control/
│       ├── mode_manager_node.py
│       ├── tracking_controller_node.py
│       └── control_arbitrator_node.py
│
├── gesture_control/           # Gesture recognition (existing)
│   └── gesture_control/
│       ├── gesture_controller.py  # TO BE REFACTORED
│       └── gesture_recognition.py
│
└── tello_ros/                 # Tello driver (existing)
    └── tello_driver/
        └── tello_driver_node.py
```

## Node Responsibilities

### 1. tello_perception Package

#### yolo_detector_node
- **Subscribes**: `/image_raw` (Image)
- **Publishes**: 
  - `/detections` (DetectionArray) - Detected objects with bbox, class, confidence
  - `/detections/annotated` (Image) - Annotated image with bounding boxes
- **Purpose**: Real-time object detection using YOLOv8

#### aruco_detector_node
- **Subscribes**: `/image_raw` (Image), `/camera_info` (CameraInfo)
- **Publishes**: 
  - `/aruco_poses` (PoseArray) - 3D poses of detected ArUco markers
  - `/aruco/annotated` (Image) - Annotated image with markers
- **Purpose**: Detect ArUco markers for distance reference

#### distance_estimator_node
- **Subscribes**: `/detections` (DetectionArray), `/aruco_poses` (PoseArray)
- **Publishes**: `/object_distances` (ObjectDistanceArray)
- **Purpose**: Calculate distances between detected objects and ArUco markers
- **Method**: Pixel-to-metric conversion using marker as scale reference

### 2. tello_control Package

#### mode_manager_node
- **Subscribes**: 
  - `/mode_switch` (String) - Mode change requests ("manual"/"gesture"/"tracking")
  - `/camera_switch` (String) - Camera source switch ("drone"/"webcam")
- **Publishes**: 
  - `/control_mode` (ControlMode) - Current mode, camera source, transition state
  - `/cmd_vel` (Twist) - Hover command during transitions
- **Purpose**: Manage mode transitions with graceful hovering
- **State Machine**:
  ```
  IDLE → mode_switch → HOVERING (1s) → SWITCHING → IDLE
  ```

#### tracking_controller_node
- **Subscribes**: 
  - `/detections` (DetectionArray)
  - `/control_mode` (ControlMode)
  - `/object_distances` (ObjectDistanceArray)
- **Publishes**: `/tracking/cmd_vel` (Twist)
- **Purpose**: PID-based autonomous tracking
- **Features**:
  - Track specific object class or closest to center
  - Multi-axis PID control (X, Y, Z, Yaw)
  - Distance-aware tracking (stop if too far)
  - Only active when mode == "tracking"

#### control_arbitrator_node
- **Subscribes**: 
  - `/manual/cmd_vel` (Twist)
  - `/gesture/cmd_vel` (Twist)
  - `/tracking/cmd_vel` (Twist)
  - `/control_mode` (ControlMode)
- **Publishes**: `/cmd_vel` (Twist) - Final command to tello_driver
- **Purpose**: Multiplex cmd_vel based on current mode
- **Logic**:
  ```python
  if mode == "manual":
      forward /manual/cmd_vel
  elif mode == "gesture":
      forward /gesture/cmd_vel
  elif mode == "tracking":
      forward /tracking/cmd_vel
  ```

### 3. gesture_control Package (REFACTORED)

#### gesture_controller (modified)
- **Subscribes**: 
  - `/image_raw` or webcam (based on `/control_mode.camera_source`)
  - `/control_mode` (ControlMode)
- **Publishes**: `/gesture/cmd_vel` (Twist) [CHANGED from /cmd_vel]
- **Purpose**: Gesture-based drone control
- **Changes**:
  - Only process gestures when `mode == "gesture"`
  - Support dynamic camera switching
  - Publish to `/gesture/cmd_vel` instead of `/cmd_vel`

### 4. tello_ros Package (EXISTING)

#### tello_driver_node
- **Subscribes**: `/cmd_vel` (Twist)
- **Publishes**: 
  - `/image_raw` (Image)
  - `/flight_data` (FlightData)
- **Services**: `/tello_action` (TelloAction)
- **Purpose**: Hardware interface to Tello drone

## Topic Architecture

```
┌─────────────────────┐
│   Tello Drone       │
└──────────┬──────────┘
           │ (UDP)
           ↓
┌─────────────────────┐
│  tello_driver_node  │
└──────────┬──────────┘
           │
           ├─→ /image_raw ─────┬─→ yolo_detector_node → /detections  ────┐
           │                   │                                         │
           │                   ├─→ aruco_detector_node → /aruco_poses ───┤
           │                   │                                         │
           │                   └─→ gesture_controller (/gesture mode)    │
           │                                                             ↓
           ├─→ /flight_data                              distance_estimator_node
           │                                                     │
           ↑                                                     ↓
           │                                            /object_distances
      /cmd_vel ← control_arbitrator_node                         │
           ↑                   ↑                                 │
           │                   │                                 ↓
     ┌─────┴────────┬──────────┴────────┐              tracking_controller_node
     │              │                   │                        │
/manual/cmd_vel  /gesture/cmd_vel  /tracking/cmd_vel ←───────────┘
     │              │                   │
     │              │                   │
  manual       gesture_controller    (tracking)
  control           ↑
     │              │
     │         /control_mode
     │              │
     └──────────────┴───────────────────── mode_manager_node
                                                   ↑
                                            /mode_switch (String)
                                            /camera_switch (String)
```

## Mode Switching Flow

### Example: Manual → Tracking

1. User publishes: `ros2 topic pub /mode_switch std_msgs/String "data: 'tracking'"`

2. **mode_manager_node**:
   - Receives request
   - Sets `transition_state = HOVERING`
   - Publishes `/cmd_vel` with (0, 0, 0) - **Drone hovers**
   - Publishes `/control_mode` with `transitioning=True`

3. **After 1.0 second** (hover_duration):
   - Sets `current_mode = TRACKING`
   - Sets `transition_state = IDLE`
   - Publishes `/control_mode` with `mode="tracking"`, `transitioning=False`

4. **control_arbitrator_node**:
   - Switches cmd_vel source from `/manual/cmd_vel` to `/tracking/cmd_vel`

5. **tracking_controller_node**:
   - Activates (was dormant in manual mode)
   - Reads `/detections`
   - Calculates PID control
   - Publishes `/tracking/cmd_vel`

6. **Drone starts tracking**

## Distance Measurement Method

### Pixel-to-Metric Conversion using ArUco Marker

1. **ArUco marker** placed in scene (known size: 10cm)
2. **aruco_detector_node** detects marker, estimates 3D pose
3. **Calculate scale**:
   ```
   marker_z = distance from camera (meters)
   pixels_per_meter = focal_length * marker_size / marker_z
   ```
4. **distance_estimator_node**:
   - Gets object pixel position (from YOLO)
   - Gets marker pixel position (from ArUco)
   - Calculates pixel distance
   - Converts to meters: `distance_m = distance_pixels / pixels_per_meter`

### Limitations & Improvements Needed
- **Current**: ArUco detector doesn't publish pixel coordinates (only 3D pose)
- **TODO**: Modify `aruco_detector_node` to publish marker corners/centers in pixels
- **Alternative**: Use depth from ArUco pose to estimate object depth directly

## Configuration Files

### perception_config.yaml
- YOLO model selection, confidence threshold
- ArUco dictionary, marker size
- Camera calibration parameters

### control_config.yaml
- PID gains (4 axes: X, Y, Z, Yaw)
- Velocity limits
- Tracking parameters (deadzone, max distance)
- Mode manager settings (hover duration)

## Launch Files (TODO)

### perception.launch.py
```python
# Launch YOLO + ArUco + Distance Estimator
```

### control.launch.py
```python
# Launch Mode Manager + Tracking Controller + Control Arbitrator
```

### full_system.launch.py
```python
# Launch all nodes including tello_driver, gesture_control
```

## Build & Run Instructions

### 1. Build packages
```bash
cd ~/ros2_pmld
colcon build --packages-select tello_interfaces tello_perception tello_control
source install/setup.bash
```

### 2. Run perception nodes
```bash
ros2 run tello_perception yolo_detector_node
ros2 run tello_perception aruco_detector_node
ros2 run tello_perception distance_estimator_node
```

### 3. Run control nodes
```bash
ros2 run tello_control mode_manager_node
ros2 run tello_control tracking_controller_node  # (TO BE IMPLEMENTED)
ros2 run tello_control control_arbitrator_node    # (TO BE IMPLEMENTED)
```

### 4. Switch modes
```bash
# Switch to gesture mode
ros2 topic pub /mode_switch std_msgs/String "data: 'gesture'" --once

# Switch camera source
ros2 topic pub /camera_switch std_msgs/String "data: 'webcam'" --once

# Switch to tracking mode
ros2 topic pub /mode_switch std_msgs/String "data: 'tracking'" --once
```

## Next Steps

1. ✅ Create tello_interfaces package
2. ✅ Create tello_perception package with YOLO, ArUco, Distance nodes
3. ✅ Create tello_control package with Mode Manager
4. ⏳ Implement tracking_controller_node (PID-based tracking)
5. ⏳ Implement control_arbitrator_node (cmd_vel multiplexer)
6. ⏳ Refactor gesture_controller for mode-aware operation
7. ⏳ Enhance ArUco detector to publish pixel coordinates
8. ⏳ Create launch files
9. ⏳ Test complete system integration

## Dependencies

```xml
<!-- package.xml additions -->
<depend>ultralytics</depend>  <!-- YOLO -->
<depend>opencv-python</depend>
<depend>opencv-contrib-python</depend>  <!-- ArUco -->
<depend>mediapipe</depend>  <!-- Gesture recognition -->
```

```bash
# Python packages
pip install ultralytics opencv-contrib-python mediapipe
```
