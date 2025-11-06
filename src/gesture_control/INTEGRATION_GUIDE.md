# Panduan Integrasi Hand Gesture Recognition dengan ROS2 Tello Drone


## Arsitektur Sistem

```
Webcam → MediaPipe → Gesture Recognition → ROS2 Node → Drone Tello
                        (Hand Poses)         (cmd_vel)
```

## Gesture Mapping (Rekomendasi)

### Hand Signs (Static Gestures):
- **Open Hand (0)** → Takeoff
- **Close Hand (1)** → Stop
- **Pointing (2)** → Emergency Stop
- **Peace Sign (3)** → Flip [BARU]
- **Thumb Up (4)** → Hover/Stop [BARU]

### Finger Gestures (Dynamic Movements):
- **Stationary (0)** → Hover
- **Move Up** → Altitude increase
- **Move Down** → Altitude decrease
- **Move Right** → Move right
- **Move Left** → Move left
- **Move Forward** → Move forward
- **Move Backward** → Move backward
- **Clockwise (1)** → Rotate CW
- **Counterclockwise (2)** → Rotate CCW

## Langkah-langkah Implementasi

### 1. Setup Environment

```bash
# Clone kedua repository
cd ~/
git clone https://github.com/kinivi/hand-gesture-recognition-mediapipe.git
git clone https://github.com/shluf/ros2_pmld.git

# Setup ROS2 workspace
cd ros2_pmld
source scripts/setup.sh
```

### 2. Buat Package Baru untuk Gesture Control

```bash
cd ~/ros2_pmld/src
ros2 pkg create --build-type ament_python gesture_control \
  --dependencies rclpy geometry_msgs tello_msgs
```

### 3. Struktur Package

```
gesture_control/
├── gesture_control/
│   ├── __init__.py
│   ├── gesture_controller.py      # Main controller
│   ├── gesture_recognition.py     # MediaPipe gesture detection
│   ├── keypoint_classifier.py     # Dari repo gesture
│   ├── point_history_classifier.py
│   └── utils/
│       └── cvfpscalc.py
├── model/
│   ├── keypoint_classifier/
│   │   ├── keypoint_classifier.tflite
│   │   └── keypoint_classifier_label.csv
│   └── point_history_classifier/
│       ├── point_history_classifier.tflite
│       └── point_history_classifier_label.csv
├── launch/
│   └── gesture_control_launch.py
├── config/
│   └── gesture_mapping.yaml
├── package.xml
├── setup.py
└── README.md
```

### 4. Instalasi Dependencies

```bash
# Install MediaPipe dan dependencies lainnya
pip install mediapipe opencv-python tensorflow numpy --break-system-packages
```

### 5. Konfigurasi Gesture Mapping

Buat file `config/gesture_mapping.yaml` untuk mapping gesture ke perintah drone:

```yaml
hand_signs:
  0: "takeoff"      # Open hand
  1: "land"         # Close hand
  2: "emergency"    # Pointing
  3: "flip"         # Peace sign (tambahkan nanti)
  4: "hover"        # Thumb up (tambahkan nanti)

finger_gestures:
  0: "hover"        # Stationary
  1: "rotate_cw"    # Clockwise
  2: "rotate_ccw"   # Counterclockwise
  4: "move"         # Moving (akan dideteksi arahnya)

velocity_settings:
  linear_speed: 0.5    # m/s
  angular_speed: 0.5   # rad/s
  vertical_speed: 0.3  # m/s

safety:
  enable_confirmation: true
  gesture_hold_time: 1.0  # seconds, untuk menghindari false positive
```

### 6. Menambahkan Gesture Baru

Untuk menambahkan gesture baru (contoh: Peace Sign, Thumb Up):

```bash
# Jalankan app.py dari repo gesture recognition
cd ~/hand-gesture-recognition-mediapipe
python app.py

# Untuk menambah hand sign:
# 1. Tekan 'k' untuk masuk mode logging
# 2. Tunjukkan gesture yang ingin ditambahkan
# 3. Tekan '3' untuk peace sign atau '4' untuk thumb up
# 4. Ulangi 100-200x untuk data yang cukup
# 5. Tekan 'k' lagi untuk keluar dari mode logging

# Retrain model:
# Buka keypoint_classification.ipynb
# Ubah NUM_CLASSES = 5 (atau sesuai jumlah gesture)
# Edit keypoint_classifier_label.csv:
#   Open
#   Close  
#   Pointing
#   Peace
#   ThumbUp
# Run notebook dari atas ke bawah
```

### 7. Testing

#### Test dengan Gazebo Simulation:

```bash
# Terminal 1: Jalankan Gazebo simulation
cd ~/ros2_pmld
source scripts/init.sh
source scripts/x.sh sim

# Terminal 2: Jalankan gesture controller
source scripts/init.sh
ros2 run gesture_control gesture_controller

# Atau dengan launch file
ros2 launch gesture_control gesture_control_launch.py
```

#### Test dengan Drone Real:

```bash
# Pastikan terhubung ke WiFi drone Tello
# Terminal 1: Jalankan driver
cd ~/ros2_pmld
source scripts/init.sh
ros2 launch tello_driver tello_driver_launch.py

# Terminal 2: Jalankan gesture controller
ros2 run gesture_control gesture_controller
```

### 8. Monitoring & Debugging

```bash
# Monitor topics
ros2 topic echo /drone1/cmd_vel
ros2 topic echo /drone1/tello_action

# Monitor gesture detection
ros2 topic echo /gesture_recognition/detected_gesture

# Check transformasi
ros2 run tf2_tools view_frames
```

## Safety Features

1. **Gesture Hold Time**: Gesture harus dihold minimal 1 detik untuk menghindari false positive
2. **Confirmation Mode**: Gesture kritis (takeoff, land, emergency) memerlukan konfirmasi
3. **Dead Man's Switch**: Jika tidak ada gesture terdeteksi >5 detik, drone akan hover
4. **Emergency Stop**: Gesture pointing akan selalu diprioritaskan sebagai emergency stop

## Troubleshooting

### Gesture tidak terdeteksi:
- Pastikan pencahayaan cukup
- Jaga jarak tangan 30-100cm dari kamera
- Background kontras dengan warna kulit
- Kalibrasi confidence threshold

### Drone tidak merespon:
- Check koneksi ROS2: `ros2 topic list`
- Verify gesture mapping di config file
- Check log: `ros2 run gesture_control gesture_controller --ros-args --log-level debug`

### Latency tinggi:
- Reduce camera resolution di gesture recognition
- Optimize MediaPipe settings (static_image_mode=False)
- Use GPU if available

## Pengembangan Lebih Lanjut

1. **Multi-hand support**: Kontrol lebih kompleks dengan dua tangan
2. **Voice commands**: Kombinasi gesture + suara
3. **Computer vision**: Tambahkan object tracking
4. **Waypoint navigation**: Pre-programmed routes dengan gesture triggers
5. **Gesture recording**: Record sequence of gestures untuk autonomous flight

## Resources

- [MediaPipe Documentation](https://google.github.io/mediapipe/)
- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [Tello SDK](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)

## License

Ikuti lisensi dari kedua repository:
- hand-gesture-recognition-mediapipe: Apache v2
- ros2_pmld: Apache-2.0
