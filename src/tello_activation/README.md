# Tello Activation Package

Central launcher package untuk mengaktifkan Tello drone dalam berbagai mode operasi.

## 📦 Overview

Package ini menyediakan unified launch files untuk:
- ✅ Real drone dengan keyboard controller
- ✅ Simulation drone dengan keyboard controller  
- ✅ Real drone only (driver saja)
- ✅ Simulation only (tanpa controller)

## 🚀 Launch Files

### 1. `real_drone.launch.py`
Launch real Tello drone dengan keyboard controller.

**Usage:**
```bash
ros2 launch tello_activation real_drone.launch.py

# With custom parameters
ros2 launch tello_activation real_drone.launch.py \
  namespace:=my_drone \
  speed:=0.7 \
  drone_ip:=192.168.10.1
```

**Parameters:**
- `namespace` (default: '') - Namespace untuk drone
- `speed` (default: 0.5) - Linear speed
- `yaw_speed` (default: 0.5) - Angular speed
- `drone_ip` (default: 192.168.10.1) - IP address drone

**Nodes:**
- `tello_driver_main` - Driver untuk komunikasi dengan drone
- `keyboard_controller` - Controller keyboard (terminal terpisah)

---

### 2. `sim_drone.launch.py`
Launch Gazebo simulation dengan keyboard controller.

**Usage:**
```bash
ros2 launch tello_activation sim_drone.launch.py

# With custom parameters
ros2 launch tello_activation sim_drone.launch.py \
  namespace:=drone1 \
  speed:=0.6
```

**Parameters:**
- `namespace` (default: drone1) - Namespace untuk drone
- `speed` (default: 0.5) - Linear speed
- `yaw_speed` (default: 0.5) - Angular speed

**Nodes:**
- Gazebo simulator
- `inject_entity.py` - Spawn drone model
- `robot_state_publisher` - Publish robot state
- `keyboard_controller` - Controller keyboard

---

### 3. `real_only.launch.py`
Launch real drone driver saja (tanpa controller).

**Usage:**
```bash
ros2 launch tello_activation real_only.launch.py

# With custom IP
ros2 launch tello_activation real_only.launch.py drone_ip:=192.168.10.2
```

**Parameters:**
- `namespace` (default: '') - Namespace untuk drone
- `drone_ip` (default: 192.168.10.1) - IP address drone

**Use Case:**
- Debugging koneksi
- Monitoring telemetry
- Custom controller development

---

### 4. `sim_only.launch.py`
Launch Gazebo simulation saja (tanpa keyboard controller).

**Usage:**
```bash
ros2 launch tello_activation sim_only.launch.py

# With namespace
ros2 launch tello_activation sim_only.launch.py namespace:=test_drone
```

**Parameters:**
- `namespace` (default: drone1) - Namespace untuk drone

**Use Case:**
- Testing custom controllers
- Algorithm development
- Multi-drone scenarios

---

## 🎮 Quick Reference

| Launch File | Real/Sim | Controller | Use Case |
|-------------|----------|------------|----------|
| `real_drone.launch.py` | Real | Keyboard | ✅ Manual flight real drone |
| `sim_drone.launch.py` | Sim | Keyboard | ✅ Manual flight simulation |
| `real_only.launch.py` | Real | None | Debugging/Custom |
| `sim_only.launch.py` | Sim | None | Development/Testing |

## 📋 Prerequisites

Pastikan semua dependencies sudah di-build:
```bash
cd ~/ros2_pmld
colcon build --packages-select \
  tello_driver \
  tello_msgs \
  tello_keyboard \
  tello_gazebo \
  tello_description \
  tello_activation
source install/setup.bash
```

## 💡 Usage Examples

### Example 1: Quick Test Real Drone
```bash
# 1. Connect ke WiFi TELLO-XXXXXX
# 2. Launch
ros2 launch tello_activation real_drone.launch.py

# 3. Di terminal keyboard yang muncul:
#    - Press T for takeoff
#    - Use WASD for movement
#    - Press L for land
```

### Example 2: Simulation Testing
```bash
# Launch simulation
ros2 launch tello_activation sim_drone.launch.py

# Di terminal keyboard:
# - Wait for Gazebo to load
# - Press T for takeoff
# - Test movements
```

### Example 3: Custom Controller Development
```bash
# Terminal 1: Launch simulation only
ros2 launch tello_activation sim_only.launch.py

# Terminal 2: Your custom controller
ros2 run my_package my_controller

# Terminal 3: Monitor
ros2 topic echo /drone1/flight_data
```

### Example 4: Multi-Drone Simulation
```bash
# Drone 1
ros2 launch tello_activation sim_only.launch.py namespace:=drone1

# Drone 2 (edit world/URDF first for different position)
ros2 launch tello_activation sim_only.launch.py namespace:=drone2
```

## 🔧 Integration with Scripts

Package ini terintegrasi dengan `scripts/x.sh`:

```bash
# Via menu
bash scripts/x.sh
# Opsi 2: Gazebo simulation (uses sim_drone.launch.py)
# Opsi 5: Real drone (uses real_drone.launch.py)

# Via command line
bash scripts/x.sh sim        # Simulation
bash scripts/x.sh real       # Real drone
```

## 📊 Topics & Services

### Published Topics (via driver/simulation)
- `/<namespace>/cmd_vel` - Velocity commands
- `/<namespace>/flight_data` - Telemetry data
- `/<namespace>/image_raw` - Camera feed
- `/<namespace>/camera_info` - Camera info

### Services
- `/<namespace>/tello_action` - Execute commands (takeoff, land, etc)

## 🐛 Troubleshooting

### Launch fails: Package not found
```bash
# Build package
cd ~/ros2_pmld
colcon build --packages-select tello_activation
source install/setup.bash
```

### Gazebo doesn't start
```bash
# Kill existing Gazebo
bash scripts/kill_gazebo.sh
# Try again
ros2 launch tello_activation sim_drone.launch.py
```

### Real drone not connecting
```bash
# Check WiFi connection
ping 192.168.10.1

# Try driver only first
ros2 launch tello_activation real_only.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /flight_data
```

## 📖 See Also

- [Keyboard Controller Guide](../tello_keyboard/README.md)
- [Real Drone Connection](../tello_keyboard/REAL_DRONE.md)
- [Tello ROS Documentation](../tello_ros/README.md)

## 📝 Notes

- Launch files menggunakan `gnome-terminal` untuk keyboard controller
- Jika menggunakan terminal lain, edit `prefix` parameter di launch file
- Untuk headless operation, gunakan `*_only.launch.py` files
- Multi-drone support dengan parameter `namespace`

---

**Package**: tello_activation  
**Version**: 0.0.0  
**License**: Apache-2.0  
**Maintainer**: shluf
