# Tello Keyboard Controller

Package ROS2 untuk mengontrol drone Tello menggunakan keyboard.

## Instalasi

```bash
cd ~/ros2_pmld
colcon build --packages-select tello_keyboard
source install/setup.bash
```

## Penggunaan

### 1. Jalankan keyboard controller saja
```bash
ros2 run tello_keyboard keyboard_controller --ros-args -p namespace:=drone1
```

### 2. Launch dengan parameter
```bash
ros2 launch tello_keyboard keyboard_launch.py namespace:=drone1 speed:=0.5
```

### 3. Launch Gazebo + Keyboard controller
```bash
ros2 launch tello_keyboard sim_keyboard_launch.py
```

## Kontrol Keyboard

### Movement
| Key | Aksi |
|-----|------|
| W | Maju |
| S | Mundur |
| A | Kiri |
| D | Kanan |
| Q | Yaw kiri |
| E | Yaw kanan |
| I | Naik |
| K | Turun |
| Space | Stop/Hover |

### Commands
| Key | Aksi |
|-----|------|
| T | Takeoff |
| L | Land |
| H | Emergency Stop |

### Flips (tekan 2x untuk konfirmasi)
| Key | Aksi |
|-----|------|
| F | Flip forward |
| B | Flip backward |
| R | Flip right |
| V | Flip left |

### Speed Control
| Key | Aksi |
|-----|------|
| + / = | Increase speed |
| - / _ | Decrease speed |

### Exit
| Key | Aksi |
|-----|------|
| ESC | Keluar |

## Parameter

- `namespace`: Namespace drone (default: `drone1`)
- `speed`: Kecepatan linear 0.0-1.0 (default: `0.5`)
- `yaw_speed`: Kecepatan angular/yaw 0.0-1.0 (default: `0.5`)

## Topic

Publish ke: `/<namespace>/cmd_vel` (geometry_msgs/Twist)
