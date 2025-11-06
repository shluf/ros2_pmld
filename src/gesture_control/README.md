# Tello Gesture Control

ROS2 package untuk mengontrol drone Tello menggunakan hand gesture recognition dengan MediaPipe.

## Features

- ✋ **Hand Gesture Recognition** - Deteksi gesture tangan menggunakan MediaPipe
- 🚁 **Drone Control** - Kontrol penuh drone Tello via ROS2
- 🎮 **Multiple Gesture Support** - Support berbagai gesture untuk berbagai perintah
- 🛡️ **Safety Features** - Gesture hold time, konfirmasi, dan emergency stop
- 🎯 **Position-based Movement** - Gerakan drone berdasarkan posisi tangan
- 🧪 **Simulation Support** - Test dengan Gazebo sebelum terbang real
- 📊 **Real-time Visualization** - Lihat gesture detection secara real-time

## Gesture Mapping (Default)

### Hand Signs (Static Gestures)
| Gesture | Action | Description |
|---------|--------|-------------|
| Open Hand (0) | Takeoff | Drone takeoff |
| Close Hand (1) | Land | Drone landing |
| Pointing (2) | Emergency | Emergency stop (prioritas tertinggi) |
| Peace Sign (3)* | Flip | Flip maneuver |
| Thumb Up (4)* | Hover | Stop dan hover |

*Perlu ditambahkan training data

### Finger Gestures (Dynamic Movements)
| Gesture | Action | Description |
|---------|--------|-------------|
| Stationary (0) | Hover | Hover di tempat |
| Clockwise (1) | Rotate CW | Rotasi searah jarum jam |
| Counterclockwise (2) | Rotate CCW | Rotasi berlawanan jarum jam |
| Moving (4) | Move | Gerakan berdasarkan posisi tangan |

## Quick Start

```bash
# Clone repo
cd ~/
git clone <this-repo>

# Run install script
cd gesture_control
chmod +x install_gesture_control.sh
./install_gesture_control.sh

# Test gesture recognition
./test_gesture.sh

# Run dengan simulation
./run_gesture_sim.sh
```

## Installation

Lihat [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) untuk panduan lengkap.

## Usage Examples

### Test Gesture Recognition
```bash
ros2 run gesture_control test_gesture
```

### Simulation
```bash
# Terminal 1
ros2 launch tello_gazebo simple_launch.py

# Terminal 2
ros2 run gesture_control gesture_controller
```

### Real Drone
```bash
# Connect to Tello WiFi first
ros2 run gesture_control gesture_controller
```

## Configuration

Edit `config/gesture_mapping.yaml`:
- Gesture mapping
- Velocity settings
- Safety parameters
- Camera settings

## Menambahkan Gesture Baru

1. Collect training data menggunakan `app.py`
2. Retrain model dengan Jupyter notebook
3. Update `gesture_mapping.yaml`
4. Rebuild package

Lihat [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) untuk detail lengkap.

## Safety

⚠️ **PENTING:**
- Test di simulation dulu
- Area terbuka minimal 3x3m
- Emergency gesture (pointing) selalu siap
- Battery monitoring aktif

## Troubleshooting

**Gesture tidak terdeteksi?**
- Check pencahayaan
- Jarak 30-100cm dari kamera
- Background kontras

**Camera tidak terbuka?**
```bash
ls /dev/video*
# Update camera_id parameter
```

**Drone tidak merespon?**
```bash
ros2 topic list  # Check topics
ros2 topic echo /drone1/cmd_vel
```

## Documentation

- [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - Panduan integrasi lengkap
- [gesture_mapping.yaml](config/gesture_mapping.yaml) - Konfigurasi
- [ROS2 Documentation](https://docs.ros.org/en/foxy/)

## License

Apache-2.0 License

## Credits

- [kinivi/hand-gesture-recognition-mediapipe](https://github.com/kinivi/hand-gesture-recognition-mediapipe)
- [shluf/ros2_pmld](https://github.com/shluf/ros2_pmld)
- Google MediaPipe
