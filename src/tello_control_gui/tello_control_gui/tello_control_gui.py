#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QGroupBox, 
                             QGridLayout, QSlider, QProgressBar, QTabWidget,
                             QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QCoreApplication
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPen

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction
from std_msgs.msg import String, Bool

import numpy as np
import math

cv2 = None
CvBridge = None


class ROSThread(QThread):
    """Thread untuk menjalankan ROS2 spin"""
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True
        
    def run(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def stop(self):
        self.running = False


class SignalEmitter(QThread):
    """QObject wrapper for emitting Qt signals from ROS2 callbacks"""
    image_signal = pyqtSignal(np.ndarray)
    flight_data_signal = pyqtSignal(dict)
    gesture_status_signal = pyqtSignal(str)


class TelloControlNode(Node):
    """ROS2 Node untuk komunikasi dengan drone"""
    
    def __init__(self, signal_emitter):
        super().__init__('tello_control_gui')
        
        # Store signal emitter
        self.signals = signal_emitter
        
        # Parameters
        self.declare_parameter('namespace', 'drone1')
        self.namespace = self.get_parameter('namespace').value
        
        # QoS Profiles
        # Sensor data uses BEST_EFFORT for low latency
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Commands use RELIABLE for guaranteed delivery
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            sensor_qos
        )
        
        self.flight_data_sub = self.create_subscription(
            FlightData,
            '/flight_data',
            self.flight_data_callback,
            sensor_qos
        )
        
        self.gesture_status_sub = self.create_subscription(
            String,
            '/gesture_recognition/detected_gesture',
            self.gesture_callback,
            sensor_qos
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            command_qos
        )
        
        self.gesture_enable_pub = self.create_publisher(
            Bool,
            '/gesture_control/enable',
            command_qos
        )
        
        # Service Clients
        self.tello_action_client = self.create_client(
            TelloAction,
            '/tello_action'
        )
        
        self.get_logger().info('TelloAction service client created')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State
        self.current_flight_data = {}
        self.gesture_enabled = False
        
        self.get_logger().info('Tello Control GUI Node initialized')
    
    def image_callback(self, msg):
        """Callback untuk video stream"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def flight_data_callback(self, msg):
        """Callback untuk flight data"""
        flight_data = {
            'battery': msg.bat,
            'altitude': msg.h,
            'temperature_low': msg.templ,
            'temperature_high': msg.temph,
            'tof': msg.tof,
            'pitch': msg.pitch,
            'roll': msg.roll,
            'yaw': msg.yaw,
            'velocity_x': msg.vgx,
            'velocity_y': msg.vgy,
            'velocity_z': msg.vgz,
            'barometer': msg.baro,
            'flight_time': msg.time,
            'acceleration_x': msg.agx,
            'acceleration_y': msg.agy,
            'acceleration_z': msg.agz,
        }
        self.current_flight_data = flight_data
        self.signals.flight_data_signal.emit(flight_data)
    
    def gesture_callback(self, msg):
        """Callback untuk gesture status"""
        self.signals.gesture_status_signal.emit(msg.data)
    
    def send_velocity(self, vx, vy, vz, vw):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(vw)
        self.cmd_vel_pub.publish(msg)
    
    def send_action(self, action):
        """Send action command (takeoff, land, etc) using service call"""
        # Check if service is available
        if not self.tello_action_client.service_is_ready():
            self.get_logger().warn(f'TelloAction service not available, cannot send: {action}')
            return
        
        request = TelloAction.Request()
        request.cmd = action
        
        self.get_logger().info(f'Sending action: {action}')
        
        # Call service asynchronously
        future = self.tello_action_client.call_async(request)
        future.add_done_callback(lambda f: self._action_response_callback(f, action))
    
    def _action_response_callback(self, future, action):
        """Handle service response"""
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info(f'Action "{action}" succeeded')
            elif response.rc == TelloAction.Response.ERROR_NOT_CONNECTED:
                self.get_logger().error(f'Action "{action}" failed: Drone not connected')
            elif response.rc == TelloAction.Response.ERROR_BUSY:
                self.get_logger().warn(f'Action "{action}" failed: Drone busy')
            else:
                self.get_logger().error(f'Action "{action}" failed with code: {response.rc}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for action "{action}": {str(e)}')
    
    def enable_gesture_control(self, enable):
        """Enable/disable gesture control"""
        msg = Bool()
        msg.data = enable
        self.gesture_enable_pub.publish(msg)
        self.gesture_enabled = enable
        self.get_logger().info(f'Gesture control: {"enabled" if enable else "disabled"}')


class VideoWidget(QWidget):
    """Widget untuk menampilkan video stream"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Video display label
        self.video_label = QLabel()
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; border: 2px solid #2196F3;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setText("Waiting for video stream...")
        
        layout.addWidget(self.video_label)
        
        self.setLayout(layout)
        
        # Stats
        self.frame_count = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)
        self.current_fps = 0
        
        # Overlay telemetry data (dict)
        self.overlay_telemetry = {}
        # Keep last pixmap to allow repainting overlay when telemetry updates
        self._last_pixmap = None
        # Draw initial overlay on an empty background so the text is visible
        try:
            self.update_overlay_telemetry({})
        except Exception:
            pass
    
    def update_image(self, cv_image):
        """Update displayed image"""
        try:
            self.frame_count += 1
            
            # Convert to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Convert to QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Scale to fit label
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.video_label.size(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )

            # Store last pixmap and draw overlay telemetry on it
            self._last_pixmap = scaled_pixmap
            overlayed = self._draw_overlay_on(scaled_pixmap)
            self.video_label.setPixmap(overlayed)
            
        except Exception as e:
            print(f"Error updating image: {e}")
    
    def update_fps(self):
        """Update FPS counter"""
        self.current_fps = self.frame_count
        self.frame_count = 0
        # If overlay exists, refresh it to show updated FPS
        if self._last_pixmap is not None:
            overlayed = self._draw_overlay_on(self._last_pixmap)
            self.video_label.setPixmap(overlayed)
        else:
            # No video received yet: draw on a blank pixmap of the label size
            blank = self._make_blank_pixmap()
            overlayed = self._draw_overlay_on(blank)
            self.video_label.setPixmap(overlayed)

    def update_overlay_telemetry(self, flight_data: dict):
        """Update the telemetry dict used for on-screen overlay and repaint if possible."""
        try:
            self.overlay_telemetry = flight_data or {}
            # Refresh overlay if we have a last pixmap, otherwise draw on
            # a blank pixmap so overlay is visible even with no video.
            if self._last_pixmap is not None:
                overlayed = self._draw_overlay_on(self._last_pixmap)
                self.video_label.setPixmap(overlayed)
            else:
                blank = self._make_blank_pixmap()
                overlayed = self._draw_overlay_on(blank)
                self.video_label.setPixmap(overlayed)
        except Exception as e:
            print(f"Error updating overlay telemetry: {e}")

    def _draw_overlay_on(self, pixmap: QPixmap) -> QPixmap:
        """Return a copy of pixmap with top-left telemetry overlay drawn."""
        try:
            p = QPixmap(pixmap)
            painter = QPainter(p)
            painter.setRenderHint(QPainter.TextAntialiasing)

            # Styling / layout
            margin = 10
            padding = 8
            line_height = 20
            # Slightly larger, more readable font
            font = QFont("Arial", 12)
            painter.setFont(font)

            # Prepare lines to show (one per row). Always show labels with
            # placeholder values when no data is available so the overlay is
            # obvious during UI development.
            fd = self.overlay_telemetry or {}
            lines = []
            # Battery first (show -- when unknown)
            battery = fd.get('battery')
            batt_text = f"{battery}%" if battery is not None else "--"
            # We'll render a small bar above the text; keep the text line too.
            lines.append(f"Battery: {batt_text}")
            # FPS
            lines.append(f"FPS: {self.current_fps}")
            # Altitude
            altitude = fd.get('altitude')
            alt_text = f"{altitude} cm" if altitude is not None else "--"
            lines.append(f"Altitude: {alt_text}")
            # TOF / distance
            tof = fd.get('tof')
            tof_text = f"{tof} cm" if tof is not None else "--"
            lines.append(f"Distance: {tof_text}")
            # Flight time
            ft = fd.get('flight_time')
            ft_text = f"{ft} s" if ft is not None else "--"
            lines.append(f"Flight Time: {ft_text}")
            # Temperature (low-high)
            t_low = fd.get('temperature_low')
            t_high = fd.get('temperature_high')
            if t_low is not None or t_high is not None:
                if t_low is None: t_low = "--"
                if t_high is None: t_high = "--"
                lines.append(f"Temp: {t_low}-{t_high} °C")
            # Attitude (pitch/roll/yaw)
            pitch = fd.get('pitch')
            roll = fd.get('roll')
            yaw = fd.get('yaw')
            pr_text = f"{pitch}°" if pitch is not None else "--"
            rr_text = f"{roll}°" if roll is not None else "--"
            yy_text = f"{yaw}°" if yaw is not None else "--"
            # Separate rows for pitch, roll, yaw
            lines.append(f"Pitch: {pr_text}")
            lines.append(f"Roll: {rr_text}")
            lines.append(f"Yaw: {yy_text}")

            # Velocity per axis and overall speed magnitude
            vx = fd.get('velocity_x')
            vy = fd.get('velocity_y')
            vz = fd.get('velocity_z')
            vx_text = f"{vx} cm/s" if vx is not None else "--"
            vy_text = f"{vy} cm/s" if vy is not None else "--"
            vz_text = f"{vz} cm/s" if vz is not None else "--"
            lines.append(f"VX: {vx_text}")
            lines.append(f"VY: {vy_text}")
            lines.append(f"VZ: {vz_text}")
            # Speed magnitude (use zeros if missing)
            try:
                sx = float(vx) if vx is not None else 0.0
                sy = float(vy) if vy is not None else 0.0
                sz = float(vz) if vz is not None else 0.0
                speed = math.sqrt(sx*sx + sy*sy + sz*sz)
                lines.append(f"Speed: {int(speed)} cm/s")
            except Exception:
                lines.append("Speed: --")

            # Compute background rect size (responsive to pixmap width)
            box_width = min(320, int(pixmap.width() * 0.45))
            box_height = padding * 2 + line_height * max(1, len(lines))

            # Draw translucent background (darker for readability)
            bg_color = QColor(0, 0, 0, 200)
            painter.fillRect(margin, margin, box_width, box_height, bg_color)

            # Draw battery bar above the text lines (if battery known/unknown still draw empty bar)
            bar_x = margin + padding
            bar_y = margin + padding
            bar_w = min(120, box_width - padding*2)
            bar_h = 12
            # Bar background
            painter.setPen(QColor(120, 120, 120))
            painter.setBrush(QColor(60, 60, 60))
            painter.drawRect(bar_x, bar_y, bar_w, bar_h)
            # Fill proportionally if battery value exists
            try:
                if battery is not None:
                    pct = max(0, min(100, int(battery)))
                    fill_w = int((pct / 100.0) * (bar_w - 2))
                    # Choose color based on battery
                    if pct > 50:
                        fill_col = QColor(76, 175, 80)  # green
                    elif pct > 20:
                        fill_col = QColor(255, 152, 0)  # orange
                    else:
                        fill_col = QColor(244, 67, 54)  # red
                    painter.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, fill_col)
                else:
                    # unknown battery: draw a faint empty bar (already drawn)
                    pass
            except Exception:
                pass

            # Draw each line of text with a subtle shadow for contrast, starting below the bar
            x = margin + padding
            y = bar_y + bar_h + 8 + (line_height - 6)
            for ln in lines:
                # Shadow (black, slightly offset)
                painter.setPen(QColor(0, 0, 0))
                painter.drawText(x + 1, y + 1, ln)
                # Main text (white)
                painter.setPen(QColor(255, 255, 255))
                painter.drawText(x, y, ln)
                y += line_height

            painter.end()
            return p
        except Exception as e:
            print(f"Error drawing overlay: {e}")
            return pixmap

    def _make_blank_pixmap(self) -> QPixmap:
        """Create a black pixmap matching the video_label size to draw overlay on when
        no video frames have been received yet."""
        try:
            size = self.video_label.size()
            if size.width() <= 0 or size.height() <= 0:
                # Fallback to a reasonable default
                w, h = 640, 480
            else:
                w, h = size.width(), size.height()
            p = QPixmap(w, h)
            p.fill(QColor(0, 0, 0))
            return p
        except Exception as e:
            print(f"Error creating blank pixmap: {e}")
            return QPixmap(640, 480)


class TelemetryWidget(QWidget):
    """Widget untuk menampilkan telemetry data"""
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Drone Telemetry")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Battery section
        battery_group = QGroupBox("Battery")
        battery_layout = QVBoxLayout()
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setMinimum(0)
        self.battery_bar.setMaximum(100)
        self.battery_bar.setValue(0)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                height: 30px;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """)
        
        self.battery_label = QLabel("Battery: --")
        self.battery_label.setAlignment(Qt.AlignCenter)
        
        battery_layout.addWidget(self.battery_bar)
        battery_layout.addWidget(self.battery_label)
        battery_group.setLayout(battery_layout)
        layout.addWidget(battery_group)
        
        # Flight data section
        flight_group = QGroupBox("Flight Data")
        flight_layout = QGridLayout()
        
        # Create labels
        self.altitude_label = self.create_data_label("Altitude:", "0 cm")
        self.temperature_label = self.create_data_label("Temperature:", "-- °C")
        self.tof_label = self.create_data_label("Distance:", "-- cm")
        self.flight_time_label = self.create_data_label("Flight Time:", "0 s")
        
        flight_layout.addWidget(self.altitude_label[0], 0, 0)
        flight_layout.addWidget(self.altitude_label[1], 0, 1)
        flight_layout.addWidget(self.temperature_label[0], 1, 0)
        flight_layout.addWidget(self.temperature_label[1], 1, 1)
        flight_layout.addWidget(self.tof_label[0], 2, 0)
        flight_layout.addWidget(self.tof_label[1], 2, 1)
        flight_layout.addWidget(self.flight_time_label[0], 3, 0)
        flight_layout.addWidget(self.flight_time_label[1], 3, 1)
        
        flight_group.setLayout(flight_layout)
        layout.addWidget(flight_group)
        
        # Attitude section
        attitude_group = QGroupBox("Attitude")
        attitude_layout = QGridLayout()
        
        self.pitch_label = self.create_data_label("Pitch:", "0°")
        self.roll_label = self.create_data_label("Roll:", "0°")
        self.yaw_label = self.create_data_label("Yaw:", "0°")
        
        attitude_layout.addWidget(self.pitch_label[0], 0, 0)
        attitude_layout.addWidget(self.pitch_label[1], 0, 1)
        attitude_layout.addWidget(self.roll_label[0], 1, 0)
        attitude_layout.addWidget(self.roll_label[1], 1, 1)
        attitude_layout.addWidget(self.yaw_label[0], 2, 0)
        attitude_layout.addWidget(self.yaw_label[1], 2, 1)
        
        attitude_group.setLayout(attitude_layout)
        layout.addWidget(attitude_group)
        
        # Velocity section
        velocity_group = QGroupBox("Velocity")
        velocity_layout = QGridLayout()
        
        self.vx_label = self.create_data_label("X:", "0 cm/s")
        self.vy_label = self.create_data_label("Y:", "0 cm/s")
        self.vz_label = self.create_data_label("Z:", "0 cm/s")
        
        velocity_layout.addWidget(self.vx_label[0], 0, 0)
        velocity_layout.addWidget(self.vx_label[1], 0, 1)
        velocity_layout.addWidget(self.vy_label[0], 1, 0)
        velocity_layout.addWidget(self.vy_label[1], 1, 1)
        velocity_layout.addWidget(self.vz_label[0], 2, 0)
        velocity_layout.addWidget(self.vz_label[1], 2, 1)
        
        velocity_group.setLayout(velocity_layout)
        layout.addWidget(velocity_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def create_data_label(self, name, value):
        """Helper to create label pair"""
        name_label = QLabel(name)
        name_label.setFont(QFont("Arial", 10, QFont.Bold))
        value_label = QLabel(value)
        value_label.setFont(QFont("Arial", 10))
        return (name_label, value_label)
    
    def update_telemetry(self, flight_data):
        """Update telemetry display"""
        try:
            # Battery
            battery = flight_data.get('battery', 0)
            self.battery_bar.setValue(battery)
            self.battery_label.setText(f"Battery: {battery}%")
            
            # Update color based on battery level
            if battery > 50:
                color = "#4CAF50"  # Green
            elif battery > 20:
                color = "#FF9800"  # Orange
            else:
                color = "#F44336"  # Red
            
            self.battery_bar.setStyleSheet(f"""
                QProgressBar {{
                    border: 2px solid grey;
                    border-radius: 5px;
                    text-align: center;
                    height: 30px;
                }}
                QProgressBar::chunk {{
                    background-color: {color};
                }}
            """)
            
            # Flight data
            self.altitude_label[1].setText(f"{flight_data.get('altitude', 0)} cm")
            temp_low = flight_data.get('temperature_low', 0)
            temp_high = flight_data.get('temperature_high', 0)
            self.temperature_label[1].setText(f"{temp_low}-{temp_high} °C")
            self.tof_label[1].setText(f"{flight_data.get('tof', 0)} cm")
            self.flight_time_label[1].setText(f"{flight_data.get('flight_time', 0)} s")
            
            # Attitude
            self.pitch_label[1].setText(f"{flight_data.get('pitch', 0)}°")
            self.roll_label[1].setText(f"{flight_data.get('roll', 0)}°")
            self.yaw_label[1].setText(f"{flight_data.get('yaw', 0)}°")
            
            # Velocity
            self.vx_label[1].setText(f"{flight_data.get('velocity_x', 0)} cm/s")
            self.vy_label[1].setText(f"{flight_data.get('velocity_y', 0)} cm/s")
            self.vz_label[1].setText(f"{flight_data.get('velocity_z', 0)} cm/s")
            
        except Exception as e:
            print(f"Error updating telemetry: {e}")


class ManualControlWidget(QWidget):
    """Widget untuk kontrol manual drone"""
    
    velocity_changed = pyqtSignal(float, float, float, float)
    action_requested = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        
        # Control state
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vw = 0.0
        self.speed_factor = 0.5
        
        # Timer for continuous control
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.send_velocity)
        self.control_timer.start(100)  # 10 Hz
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Manual Control")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Speed control
        speed_group = QGroupBox("Speed Control")
        speed_layout = QVBoxLayout()
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(10)
        self.speed_slider.setValue(5)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        self.speed_label = QLabel("Speed: 50%")
        self.speed_label.setAlignment(Qt.AlignCenter)
        
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # Action buttons
        action_group = QGroupBox("Quick Actions")
        action_layout = QGridLayout()
        
        self.takeoff_btn = self.create_action_button("Takeoff", "#4CAF50", "takeoff")
        self.land_btn = self.create_action_button("Land", "#2196F3", "land")
        self.emergency_btn = self.create_action_button("EMERGENCY", "#F44336", "emergency")
        self.hover_btn = self.create_action_button("Hover", "#FF9800", "hover")
        
        action_layout.addWidget(self.takeoff_btn, 0, 0)
        action_layout.addWidget(self.land_btn, 0, 1)
        action_layout.addWidget(self.hover_btn, 1, 0)
        action_layout.addWidget(self.emergency_btn, 1, 1)
        
        action_group.setLayout(action_layout)
        layout.addWidget(action_group)
        
        # Movement controls
        move_group = QGroupBox("Movement")
        move_layout = QGridLayout()
        
        # Forward/Backward/Left/Right
        self.forward_btn = self.create_move_button("↑\nForward", self.move_forward)
        self.backward_btn = self.create_move_button("↓\nBackward", self.move_backward)
        self.left_btn = self.create_move_button("←\nLeft", self.move_left)
        self.right_btn = self.create_move_button("→\nRight", self.move_right)
        
        # Up/Down
        self.up_btn = self.create_move_button("⬆\nUp", self.move_up)
        self.down_btn = self.create_move_button("⬇\nDown", self.move_down)
        
        # Rotate
        self.rotate_left_btn = self.create_move_button("⟲\nRotate L", self.rotate_left)
        self.rotate_right_btn = self.create_move_button("⟳\nRotate R", self.rotate_right)
        
        # Arrange in grid
        move_layout.addWidget(QLabel(""), 0, 0)
        move_layout.addWidget(self.forward_btn, 0, 1)
        move_layout.addWidget(self.up_btn, 0, 2)
        
        move_layout.addWidget(self.left_btn, 1, 0)
        move_layout.addWidget(self.hover_btn, 1, 1)
        move_layout.addWidget(self.right_btn, 1, 2)
        
        move_layout.addWidget(QLabel(""), 2, 0)
        move_layout.addWidget(self.backward_btn, 2, 1)
        move_layout.addWidget(self.down_btn, 2, 2)
        
        move_layout.addWidget(self.rotate_left_btn, 3, 0)
        move_layout.addWidget(QLabel(""), 3, 1)
        move_layout.addWidget(self.rotate_right_btn, 3, 2)
        
        move_group.setLayout(move_layout)
        layout.addWidget(move_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def create_action_button(self, text, color, action):
        """Create action button"""
        btn = QPushButton(text)
        btn.setMinimumHeight(50)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {color};
                opacity: 0.8;
            }}
            QPushButton:pressed {{
                background-color: #555;
            }}
        """)
        btn.clicked.connect(lambda: self.action_requested.emit(action))
        return btn
    
    def create_move_button(self, text, callback):
        """Create movement button"""
        btn = QPushButton(text)
        btn.setMinimumSize(80, 60)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #607D8B;
                color: white;
                border: 2px solid #455A64;
                border-radius: 5px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #546E7A;
            }
            QPushButton:pressed {
                background-color: #37474F;
            }
        """)
        btn.pressed.connect(callback)
        btn.released.connect(self.stop_movement)
        return btn
    
    def update_speed(self, value):
        """Update speed factor"""
        self.speed_factor = value / 10.0
        self.speed_label.setText(f"Speed: {int(self.speed_factor * 100)}%")
    
    def move_forward(self):
        self.vx = self.speed_factor
    
    def move_backward(self):
        self.vx = -self.speed_factor
    
    def move_left(self):
        self.vy = self.speed_factor
    
    def move_right(self):
        self.vy = -self.speed_factor
    
    def move_up(self):
        self.vz = self.speed_factor
    
    def move_down(self):
        self.vz = -self.speed_factor
    
    def rotate_left(self):
        self.vw = self.speed_factor
    
    def rotate_right(self):
        self.vw = -self.speed_factor
    
    def stop_movement(self):
        """Stop all movement"""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vw = 0.0
    
    def send_velocity(self):
        """Send velocity command"""
        if self.vx != 0 or self.vy != 0 or self.vz != 0 or self.vw != 0:
            self.velocity_changed.emit(self.vx, self.vy, self.vz, self.vw)


class GestureControlWidget(QWidget):
    """Widget untuk gesture control status"""
    
    mode_changed = pyqtSignal(bool)  # True = gesture, False = manual
    
    def __init__(self):
        super().__init__()
        self.gesture_mode = False
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Control Mode")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Mode toggle
        mode_group = QGroupBox("Current Mode")
        mode_layout = QVBoxLayout()
        
        self.mode_btn = QPushButton("Switch to Gesture Mode")
        self.mode_btn.setMinimumHeight(60)
        self.mode_btn.setCheckable(True)
        self.mode_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:checked {
                background-color: #4CAF50;
            }
        """)
        self.mode_btn.clicked.connect(self.toggle_mode)
        
        self.mode_status = QLabel("📱 Manual Control Active")
        self.mode_status.setFont(QFont("Arial", 12))
        self.mode_status.setAlignment(Qt.AlignCenter)
        self.mode_status.setStyleSheet("color: #2196F3; padding: 10px;")
        
        mode_layout.addWidget(self.mode_btn)
        mode_layout.addWidget(self.mode_status)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # Gesture status
        gesture_group = QGroupBox("Gesture Status")
        gesture_layout = QVBoxLayout()
        
        self.gesture_label = QLabel("No gesture detected")
        self.gesture_label.setFont(QFont("Arial", 11))
        self.gesture_label.setAlignment(Qt.AlignCenter)
        self.gesture_label.setStyleSheet("""
            background-color: #f0f0f0;
            padding: 15px;
            border-radius: 5px;
            color: #666;
        """)
        
        gesture_layout.addWidget(self.gesture_label)
        gesture_group.setLayout(gesture_layout)
        layout.addWidget(gesture_group)
        
        # Instructions
        instructions_group = QGroupBox("Instructions")
        instructions_layout = QVBoxLayout()
        
        instructions = QLabel(
            "<b>Manual Mode:</b><br>"
            "• Use buttons to control drone<br>"
            "• Adjust speed with slider<br><br>"
            "<b>Gesture Mode:</b><br>"
            "• Open hand: Takeoff<br>"
            "• Close hand: Land<br>"
            "• Pointing: Emergency<br>"
            "• Rotate hand: Rotate drone"
        )
        instructions.setWordWrap(True)
        instructions.setStyleSheet("padding: 10px;")
        
        instructions_layout.addWidget(instructions)
        instructions_group.setLayout(instructions_layout)
        layout.addWidget(instructions_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def toggle_mode(self):
        """Toggle between manual and gesture mode"""
        self.gesture_mode = self.mode_btn.isChecked()
        
        if self.gesture_mode:
            self.mode_btn.setText("Switch to Manual Mode")
            self.mode_status.setText("Gesture Control Active")
            self.mode_status.setStyleSheet("color: #4CAF50; padding: 10px; font-weight: bold;")
        else:
            self.mode_btn.setText("Switch to Gesture Mode")
            self.mode_status.setText("Manual Control Active")
            self.mode_status.setStyleSheet("color: #2196F3; padding: 10px;")
        
        self.mode_changed.emit(self.gesture_mode)
    
    def update_gesture_status(self, gesture):
        """Update gesture status display"""
        if gesture:
            self.gesture_label.setText(f"Detected: {gesture}")
            self.gesture_label.setStyleSheet("""
                background-color: #4CAF50;
                padding: 15px;
                border-radius: 5px;
                color: white;
                font-weight: bold;
            """)
        else:
            self.gesture_label.setText("No gesture detected")
            self.gesture_label.setStyleSheet("""
                background-color: #f0f0f0;
                padding: 15px;
                border-radius: 5px;
                color: #666;
            """)


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS2
        rclpy.init()
        
        # Create signal emitter
        self.signal_emitter = SignalEmitter()
        
        # Create ROS node with signal emitter
        self.ros_node = TelloControlNode(self.signal_emitter)
        
        # Start ROS thread
        self.ros_thread = ROSThread(self.ros_node)
        self.ros_thread.start()
        
        # Connect signals from signal emitter
        self.signal_emitter.image_signal.connect(self.update_video)
        self.signal_emitter.flight_data_signal.connect(self.update_telemetry)
        self.signal_emitter.gesture_status_signal.connect(self.update_gesture)
        
        self.setup_ui()
        self.setWindowTitle("Tello Drone Control Center")
        self.resize(1400, 800)
    
    def setup_ui(self):
        """Setup main UI"""
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Left panel - Video and telemetry
        left_panel = QVBoxLayout()
        
        self.video_widget = VideoWidget()
        left_panel.addWidget(self.video_widget, stretch=3)
        
        # Keep an internal telemetry widget for legacy updates but do not add it
        # to the layout — telemetry will be shown as an overlay on the video.
        self.telemetry_widget = TelemetryWidget()
        
        # Right panel - Controls
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        # Tab widget for different control modes
        self.tab_widget = QTabWidget()
        
        self.manual_control = ManualControlWidget()
        self.manual_control.velocity_changed.connect(self.send_velocity)
        self.manual_control.action_requested.connect(self.send_action)
        
        self.gesture_control = GestureControlWidget()
        self.gesture_control.mode_changed.connect(self.change_control_mode)
        
        self.tab_widget.addTab(self.manual_control, "Manual Control")
        self.tab_widget.addTab(self.gesture_control, "Gesture Mode")
        
        right_layout.addWidget(self.tab_widget)
        right_panel.setLayout(right_layout)
        right_panel.setMaximumWidth(450)
        
        # Add to main layout
        main_layout.addLayout(left_panel, stretch=2)
        main_layout.addWidget(right_panel, stretch=1)
        
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
    
    def update_video(self, cv_image):
        """Update video display"""
        self.video_widget.update_image(cv_image)
    
    def update_telemetry(self, flight_data):
        """Update telemetry display"""
        # Update the hidden telemetry widget (keeps internal state) and the
        # video overlay in the top-left of the video display.
        try:
            self.telemetry_widget.update_telemetry(flight_data)
        except Exception:
            pass
        self.video_widget.update_overlay_telemetry(flight_data)
    
    def update_gesture(self, gesture):
        """Update gesture status"""
        self.gesture_control.update_gesture_status(gesture)
    
    def send_velocity(self, vx, vy, vz, vw):
        """Send velocity command to drone"""
        if not self.gesture_control.gesture_mode:
            self.ros_node.send_velocity(vx, vy, vz, vw)
    
    def send_action(self, action):
        """Send action command to drone"""
        if action == "hover":
            self.ros_node.send_velocity(0, 0, 0, 0)
        else:
            self.ros_node.send_action(action)
    
    def change_control_mode(self, gesture_mode):
        """Change between manual and gesture control"""
        self.ros_node.enable_gesture_control(gesture_mode)
        
        # Enable/disable manual control
        self.manual_control.setEnabled(not gesture_mode)
        
        if gesture_mode:
            print("Switched to Gesture Control Mode")
        else:
            print("Switched to Manual Control Mode")
    
    def closeEvent(self, event):
        """Handle window close"""
        print("Shutting down...")
        
        # Stop ROS thread
        self.ros_thread.stop()
        self.ros_thread.wait()
        
        # Shutdown ROS
        self.ros_node.destroy_node()
        rclpy.shutdown()
        
        event.accept()


def main():
    global cv2, CvBridge
    
    app = QApplication(sys.argv)
    
    import cv2 as cv2_module
    from cv_bridge import CvBridge as CvBridge_class
    cv2 = cv2_module
    CvBridge = CvBridge_class
    
    # Set application style
    app.setStyle('bb10dark')
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()