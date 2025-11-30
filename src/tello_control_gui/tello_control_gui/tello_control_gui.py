#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QGroupBox, 
                             QGridLayout, QSlider, QProgressBar, QTabWidget,
                             QFrame, QComboBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QCoreApplication, QRect
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPen

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, PoseArray
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction
from std_msgs.msg import String, Bool
from tello_interfaces.msg import DetectionArray, ObjectDistanceArray, ControlMode
from tello_interfaces.srv import SetControlMode, SetGestureCamera

import numpy as np
import math
import json

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
    webcam_image_signal = pyqtSignal(np.ndarray)
    flight_data_signal = pyqtSignal(dict)
    gesture_status_signal = pyqtSignal(str)
    gesture_annotated_signal = pyqtSignal(np.ndarray)  # Gesture skeleton annotated image
    
    # Signals for annotations
    detections_signal = pyqtSignal(list)
    aruco_signal = pyqtSignal(list)
    distance_signal = pyqtSignal(list)
    tracking_signal = pyqtSignal(dict)
    mode_signal = pyqtSignal(str)
    cmd_vel_signal = pyqtSignal(object)  # For Twist message


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
            '/gesture_recognition/status',
            self.gesture_callback,
            sensor_qos
        )
        
        # New Subscribers for Annotations
        self.detection_sub = self.create_subscription(
            DetectionArray, 
            '/detections', 
            self.detection_callback, 
            sensor_qos
        )
        
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            '/aruco_poses', 
            self.aruco_callback, 
            sensor_qos
        )
        
        self.distance_sub = self.create_subscription(
            ObjectDistanceArray, 
            '/object_distances', 
            self.distance_callback, 
            sensor_qos
        )
        
        self.tracking_sub = self.create_subscription(
            Twist, 
            '/tracking/cmd_vel', 
            self.tracking_callback, 
            sensor_qos
        )
        
        self.mode_sub = self.create_subscription(
            ControlMode, 
            '/control_mode', 
            self.mode_callback, 
            sensor_qos
        )
        
        # Subscribe to cmd_vel to display in telemetry
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            sensor_qos
        )
        
        # Gesture annotated image subscriber (skeleton overlay)
        self.gesture_annotated_sub = self.create_subscription(
            Image,
            '/gesture/annotated',
            self.gesture_annotated_callback,
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
        
        self.mode_switch_client = self.create_client(
            SetControlMode,
            '/mode_switch'
        )
        if not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode switch service not available yet')
        
        # Gesture camera switch service client
        self.gesture_camera_client = self.create_client(
            SetGestureCamera,
            '/gesture/set_camera'
        )
        if not self.gesture_camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Gesture camera switch service not available yet')
        
        # Service Clients
        self.tello_action_client = self.create_client(
            TelloAction,
            '/tello_action'
        )
        
        self.get_logger().info('TelloAction service client created')
        
        # CV Bridge
        self.bridge = CvBridge()

        # Webcam publisher: publish webcam frames to a dedicated topic by
        # default so the local webcam doesn't override the drone '/image_raw'
        # feed when a drone is not present. Set the ROS parameter
        # 'mirror_webcam_to_image_raw' to True to mirror webcam frames into
        # '/image_raw' for backwards compatibility with nodes expecting that
        # topic.
        self.declare_parameter('mirror_webcam_to_image_raw', False)
        self.mirror_webcam = self.get_parameter('mirror_webcam_to_image_raw').value

        try:
            self.webcam_pub = self.create_publisher(Image, '/webcam/image_raw', sensor_qos)
        except Exception:
            self.webcam_pub = None

        # Optional mirror publisher to '/image_raw' if requested
        try:
            if self.mirror_webcam:
                self.webcam_mirror_pub = self.create_publisher(Image, '/webcam/image_raw/mirror', sensor_qos)
            else:
                self.webcam_mirror_pub = None
        except Exception:
            self.webcam_mirror_pub = None
        
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

    def publish_webcam_frame(self, cv_image):
        """Publish a cv2 BGR image as a ROS Image message on `/image_raw`.

        This allows external gesture recognition nodes to process the laptop
        webcam frames as if they were the drone feed.
        """
        if self.webcam_pub is None:
            return
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            # Publish to dedicated webcam topic
            self.webcam_pub.publish(ros_img)
            # Mirror to '/image_raw' only if explicitly enabled
            try:
                if getattr(self, 'webcam_mirror_pub', None) is not None:
                    self.webcam_mirror_pub.publish(ros_img)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'Error publishing webcam frame: {e}')
    
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

    def detection_callback(self, msg):
        self.signals.detections_signal.emit(msg.detections)

    def aruco_callback(self, msg):
        self.signals.aruco_signal.emit(msg.poses)

    def distance_callback(self, msg):
        self.signals.distance_signal.emit(msg.distances)

    def tracking_callback(self, msg):
        data = {'linear': {'x': msg.linear.x, 'y': msg.linear.y}, 'angular': {'z': msg.angular.z}}
        self.signals.tracking_signal.emit(data)

    def mode_callback(self, msg):
        self.signals.mode_signal.emit(msg.mode)
    
    def cmd_vel_callback(self, msg):
        """Callback untuk cmd_vel display"""
        self.signals.cmd_vel_signal.emit(msg)

    def gesture_annotated_callback(self, msg):
        """Callback untuk gesture annotated image (skeleton overlay)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.gesture_annotated_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting gesture annotated image: {e}')

    def set_mode(self, mode_name):
        if not self.mode_switch_client.service_is_ready():
            self.get_logger().warn(f'Mode switch service unavailable, cannot send: {mode_name}')
            return

        request = SetControlMode.Request()
        request.mode = mode_name
        request.camera_source = ''

        self.get_logger().info(f'Requesting mode switch: {mode_name}')
        future = self.mode_switch_client.call_async(request)
        future.add_done_callback(lambda f: self._mode_switch_response_callback(f, mode_name))
    
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

    def _mode_switch_response_callback(self, future, mode_name):
        """Log result of SetControlMode service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Mode switch to "{mode_name}" accepted: {response.message}'
                )
            else:
                self.get_logger().warn(
                    f'Mode switch to "{mode_name}" rejected: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(
                f'Service call failed for mode "{mode_name}": {str(e)}'
            )
    
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
        
        # Caption label (overlayed on top of video_label)
        self.caption_label = QLabel(self.video_label)
        self.caption_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.caption_label.setStyleSheet("background-color: rgba(0,0,0,120); color: white; padding: 4px; border-radius: 4px;")
        self.caption_label.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.caption_label.setText("")
        self.caption_label.move(0, 4)
        self.caption_label.resize(self.video_label.width(), 28)

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
        # Whether this widget should draw telemetry overlay when frames are present
        self.overlay_enabled = True
        # Has received recent frames (used to hide overlay/keep black when no drone)
        self.has_frame = False
        # Number of consecutive FPS ticks with no frames received
        self._no_frame_count = 0
        # Draw initial overlay on an empty background so the text is visible
        try:
            self.update_overlay_telemetry({})
        except Exception:
            pass

        # Whether overlay should be shown even when no frames are present
        self.show_overlay_always = False
        
        # Show FPS overlay on video
        self.show_fps_overlay = True

        # Data storage for annotations
        self.detections = []
        self.aruco_poses = []
        self.distances = []
        self.tracking_cmd = None
        self.gesture_data = {}
        
        # Caption text for overlay
        self.caption_text = ""
        
        # View flags
        self.show_yolo = True
        self.show_aruco = True
        self.show_tracking = True
        self.show_gesture = True
        self.show_distance = True
        self.is_webcam = False  # Flag to identify if this widget shows webcam/gesture

    def update_detections(self, detections):
        self.detections = detections
    
    def update_aruco(self, poses):
        self.aruco_poses = poses

    def update_distances(self, distances):
        self.distances = distances

    def update_tracking(self, cmd):
        self.tracking_cmd = cmd

    def update_gesture_data(self, data_str):
        try:
            self.gesture_data = json.loads(data_str)
        except:
            pass
    
    def update_image(self, cv_image):
        """Update displayed image with FPS and annotations overlay."""
        try:
            self.frame_count += 1
            # Reset no-frame counter and mark we have a frame
            self._no_frame_count = 0
            self.has_frame = True
            
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

            # Store last pixmap and always draw overlay (FPS + annotations)
            self._last_pixmap = scaled_pixmap
            overlayed = self._draw_overlay_on(scaled_pixmap)
            self.video_label.setPixmap(overlayed)
            
        except Exception as e:
            print(f"Error updating image: {e}")
    
    def update_fps(self):
        """Update FPS counter and refresh overlay."""
        self.current_fps = self.frame_count
        self.frame_count = 0
        
        # Track consecutive empty ticks
        if self.current_fps == 0:
            self._no_frame_count += 1
        else:
            self._no_frame_count = 0

        # If no frames for 2 seconds, show black screen
        if self._no_frame_count >= 2:
            self.has_frame = False
            self._last_pixmap = None
            blank = self._make_blank_pixmap()
            # Draw FPS on blank too
            overlayed = self._draw_overlay_on(blank)
            self.video_label.setPixmap(overlayed)
            return

        # Refresh overlay with updated FPS
        if self._last_pixmap is not None:
            overlayed = self._draw_overlay_on(self._last_pixmap)
            self.video_label.setPixmap(overlayed)

    def update_overlay_telemetry(self, flight_data: dict):
        """Update the telemetry dict used for on-screen overlay and repaint if possible."""
        try:
            self.overlay_telemetry = flight_data or {}
            # Refresh overlay if we have a pixmap
            if self._last_pixmap is not None:
                overlayed = self._draw_overlay_on(self._last_pixmap)
                self.video_label.setPixmap(overlayed)
        except Exception as e:
            print(f"Error updating overlay telemetry: {e}")

    def _draw_overlay_on(self, pixmap: QPixmap) -> QPixmap:
        """Return a copy of pixmap with FPS and annotations overlay drawn."""
        try:
            p = QPixmap(pixmap)
            painter = QPainter(p)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setRenderHint(QPainter.TextAntialiasing)

            # Draw annotations based on source type
            if not self.is_webcam:
                # Drone widget: draw YOLO, ArUco, tracking, distance annotations
                if self.show_yolo:
                    self._draw_yolo(painter, p.width(), p.height())
                if self.show_aruco:
                    self._draw_aruco(painter, p.width(), p.height())
                if self.show_distance:
                    self._draw_distance(painter, p.width(), p.height())
                if self.show_tracking:
                    self._draw_tracking(painter, p.width(), p.height())

            # Draw FPS overlay (top-left corner for better visibility)
            if self.show_fps_overlay:
                font = QFont("Arial", 12, QFont.Bold)
                painter.setFont(font)
                fps_text = f"FPS: {self.current_fps}"
                
                # Position at top-left corner
                x = 10
                y = 20
                
                # Draw background box
                bg_rect = QRect(5, 5, 70, 22)
                painter.fillRect(bg_rect, QColor(0, 0, 0, 200))
                
                # Draw text (bright green for visibility)
                if self.current_fps > 0:
                    painter.setPen(QColor(0, 255, 0))  # Green when active
                else:
                    painter.setPen(QColor(255, 165, 0))  # Orange when no frames
                painter.drawText(x, y, fps_text)
            
            # Draw caption overlay at top center if set
            if hasattr(self, 'caption_text') and self.caption_text:
                font = QFont("Arial", 11, QFont.Bold)
                painter.setFont(font)
                fm = painter.fontMetrics()
                text_width = fm.horizontalAdvance(self.caption_text)
                x = (p.width() - text_width) // 2
                
                # Background
                bg_rect = QRect(x - 5, 5, text_width + 10, 22)
                painter.fillRect(bg_rect, QColor(0, 0, 0, 180))
                
                # Text
                painter.setPen(QColor(255, 255, 255))
                painter.drawText(x, 20, self.caption_text)

            painter.end()
            return p
        except Exception as e:
            print(f"Error drawing overlay: {e}")
            return pixmap

    def _draw_yolo(self, painter, widget_w, widget_h):
        """Draw YOLO detection boxes with proper scaling."""
        # Original image dimensions (Tello: 960x720)
        orig_w, orig_h = 960, 720
        scale_x = widget_w / orig_w
        scale_y = widget_h / orig_h
        
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        fm = painter.fontMetrics()
        
        # Status indicator at top-right
        status_x = widget_w - 150
        status_y = 35
        
        if not self.detections:
            status_text = "YOLO: No objects"
            painter.fillRect(status_x - 4, status_y - 14, fm.horizontalAdvance(status_text) + 8, 20, QColor(0, 0, 0, 180))
            painter.setPen(QColor(128, 128, 128))
            painter.drawText(status_x, status_y, status_text)
            return
        
        status_text = f"YOLO: {len(self.detections)} objects"
        painter.fillRect(status_x - 4, status_y - 14, fm.horizontalAdvance(status_text) + 8, 20, QColor(0, 0, 0, 180))
        painter.setPen(QColor(0, 255, 0))
        painter.drawText(status_x, status_y, status_text)
        
        # Draw detection boxes
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.setBrush(Qt.NoBrush)
        
        for d in self.detections:
            # Scale coordinates to widget size
            x = int(d.x * scale_x)
            y = int(d.y * scale_y)
            w = int(d.width * scale_x)
            h = int(d.height * scale_y)
            
            # Draw bounding box
            painter.drawRect(x, y, w, h)
            
            # Draw label background
            label = f"{d.class_name} {d.confidence:.2f}"
            text_w = fm.horizontalAdvance(label)
            painter.fillRect(x, y - 18, text_w + 6, 18, QColor(0, 255, 0, 180))
            
            # Draw label text
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(x + 3, y - 5, label)
            painter.setPen(QPen(QColor(0, 255, 0), 2))

    def _draw_aruco(self, painter, widget_w, widget_h):
        """Draw ArUco marker info."""
        painter.setPen(QPen(QColor(0, 255, 255), 2))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        
        # Position at bottom-left
        x = 10
        y = widget_h - 60
        fm = painter.fontMetrics()
        
        # Always show ArUco status (even if no markers detected)
        if not self.aruco_poses:
            info_text = "ArUco: No markers"
            text_w = fm.horizontalAdvance(info_text)
            painter.fillRect(x - 2, y - 14, text_w + 8, 20, QColor(0, 0, 0, 180))
            painter.setPen(QColor(128, 128, 128))  # Gray when no markers
            painter.drawText(x, y, info_text)
            return
        
        # Draw marker count indicator
        info_text = f"ArUco: {len(self.aruco_poses)} markers"
        text_w = fm.horizontalAdvance(info_text)
        painter.fillRect(x - 2, y - 14, text_w + 8, 20, QColor(0, 0, 0, 180))
        
        painter.setPen(QColor(0, 255, 255))
        painter.drawText(x, y, info_text)
        
        # Show distance for each marker
        for i, pose in enumerate(self.aruco_poses):
            dist = pose.position.z  # Distance in meters
            marker_info = f"  M{i}: {dist:.2f}m"
            y += 18
            painter.fillRect(x - 2, y - 14, fm.horizontalAdvance(marker_info) + 8, 20, QColor(0, 0, 0, 180))
            painter.drawText(x, y, marker_info)

    def _draw_distance(self, painter, widget_w, widget_h):
        """Draw distance measurements between objects and markers."""
        if not self.distances:
            return
        
        # Original image dimensions (Tello: 960x720)
        orig_w, orig_h = 960, 720
        scale_x = widget_w / orig_w
        scale_y = widget_h / orig_h
        
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        
        for dist_obj in self.distances:
            # Scale coordinates
            obj_x = int(dist_obj.object_center_x * scale_x)
            obj_y = int(dist_obj.object_center_y * scale_y)
            marker_x = int(dist_obj.marker_center_x * scale_x)
            marker_y = int(dist_obj.marker_center_y * scale_y)
            
            # Draw line between object and marker
            painter.setPen(QPen(QColor(255, 255, 0), 2, Qt.DashLine))
            painter.drawLine(obj_x, obj_y, marker_x, marker_y)
            
            # Draw distance label at midpoint
            mid_x = (obj_x + marker_x) // 2
            mid_y = (obj_y + marker_y) // 2
            
            dist_cm = dist_obj.distance_meters * 100
            dist_text = f"{dist_cm:.1f}cm"
            
            fm = painter.fontMetrics()
            text_w = fm.horizontalAdvance(dist_text)
            painter.fillRect(mid_x - 2, mid_y - 12, text_w + 6, 16, QColor(255, 255, 0, 200))
            
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(mid_x + 2, mid_y, dist_text)
            
            # Draw object point
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(obj_x - 4, obj_y - 4, 8, 8)
            
            # Draw marker point
            painter.setPen(QPen(QColor(0, 255, 0), 2))
            painter.setBrush(QColor(0, 255, 0))
            painter.drawEllipse(marker_x - 4, marker_y - 4, 8, 8)
            
            painter.setBrush(Qt.NoBrush)

    def _draw_tracking(self, painter, w, h):
        if not self.tracking_cmd:
            return
        
        cx, cy = w // 2, h // 2
        # Visualize control command as a vector from center
        # linear.y is left/right, linear.x is fwd/back (up/down in image)
        # Scale factor for visualization
        scale = 100.0
        dx = -int(self.tracking_cmd['linear']['y'] * scale) 
        dy = -int(self.tracking_cmd['linear']['x'] * scale)
        
        # Only draw if there's significant movement
        if abs(dx) > 5 or abs(dy) > 5:
            painter.setPen(QPen(QColor(255, 100, 100), 3))
            painter.drawLine(cx, cy, cx + dx, cy + dy)
            
            # Draw arrowhead
            painter.setBrush(QColor(255, 100, 100))
            painter.drawEllipse(cx + dx - 5, cy + dy - 5, 10, 10)
        
        # Draw center crosshair
        painter.setPen(QPen(QColor(255, 100, 100), 2))
        painter.drawLine(cx - 15, cy, cx + 15, cy)
        painter.drawLine(cx, cy - 15, cx, cy + 15)
        
        # Draw angular velocity (yaw) as an arc
        az = self.tracking_cmd['angular']['z']
        if abs(az) > 0.1:
            painter.setPen(QPen(QColor(255, 255, 0), 2))
            # Draw rotation indicator
            arc_radius = 40
            start_angle = 0 if az > 0 else 180 * 16
            span_angle = int(abs(az) * 16 * 90)  # 16 units per degree
            painter.drawArc(cx - arc_radius, cy - arc_radius, 
                           arc_radius * 2, arc_radius * 2, 
                           start_angle, span_angle)
        
        # Draw tracking info text
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        info_text = f"Track: X={self.tracking_cmd['linear']['x']:.2f} Y={self.tracking_cmd['linear']['y']:.2f}"
        painter.fillRect(w - 180, 30, 170, 20, QColor(0, 0, 0, 180))
        painter.setPen(QColor(255, 100, 100))
        painter.drawText(w - 175, 45, info_text)

    def _draw_gesture(self, painter):
        if not self.gesture_data: return
        pos = self.gesture_data.get('hand_position')
        sign = self.gesture_data.get('hand_sign')
        
        if pos:
            x, y = pos
            painter.setPen(QPen(Qt.magenta, 3))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(int(x)-15, int(y)-15, 30, 30)
            
        if sign:
            painter.setFont(QFont("Arial", 14, QFont.Bold))
            painter.setPen(Qt.magenta)
            painter.drawText(20, 50, f"Gesture: {sign}")

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

    def set_overlay_enabled(self, enabled: bool):
        """Enable or disable telemetry overlay drawing for this widget."""
        self.overlay_enabled = bool(enabled)

    def set_show_overlay_always(self, enabled: bool):
        """When True, overlay is drawn even if no video frames have been received."""
        self.show_overlay_always = bool(enabled)

    def set_caption(self, text: str):
        """Set the small caption label shown over the video."""
        try:
            self.caption_text = text  # Store for overlay drawing
            self.caption_label.setText(text)
            # Resize caption to match video width
            self.caption_label.resize(self.video_label.width(), 28)
        except Exception:
            pass


class WebcamThread(QThread):
    """Thread to capture webcam frames with OpenCV and emit them via signal.

    It will also optionally publish frames into ROS via the provided node so
    external gesture nodes can process the laptop camera feed.
    """
    def __init__(self, signal_emitter: SignalEmitter, ros_node: TelloControlNode = None, device=0):
        super().__init__()
        self.signals = signal_emitter
        self.ros_node = ros_node
        self.device = device
        self.running = True

    def run(self):
        try:
            cap = cv2.VideoCapture(self.device)
        except Exception:
            cap = None

        if cap is None or not cap.isOpened():
            # Try device 0 fallback failure handled silently
            return

        while self.running:
            try:
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.msleep(30)
                    continue

                # Emit to GUI
                self.signals.webcam_image_signal.emit(frame)

                # Also publish into ROS topic for gesture recognition if available
                try:
                    if self.ros_node is not None:
                        self.ros_node.publish_webcam_frame(frame)
                except Exception:
                    pass

                # Small sleep to avoid pegging CPU
                self.msleep(30)
            except Exception:
                self.msleep(100)

        try:
            cap.release()
        except Exception:
            pass

    def stop(self):
        self.running = False


class TelemetryWidget(QWidget):
    """Widget untuk menampilkan telemetry data dengan layout 3 kolom penuh"""
    
    def __init__(self):
        super().__init__()
        self.flight_time_seconds = 0
        self.setup_ui()
        
    def setup_ui(self):
        # Main horizontal layout for 3 columns
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(8, 6, 8, 6)
        main_layout.setSpacing(8)
        
        # =====================================================
        # COLUMN 1: SYSTEM & STATE
        # =====================================================
        system_frame = QFrame()
        system_frame.setObjectName("systemFrame")
        system_layout = QVBoxLayout()
        system_layout.setContentsMargins(10, 6, 10, 6)
        system_layout.setSpacing(4)
        
        # Title
        system_title = QLabel("⚙️  SYSTEM & STATE")
        system_title.setFont(QFont("Arial", 9, QFont.Bold))
        system_title.setAlignment(Qt.AlignCenter)
        system_layout.addWidget(system_title)
        
        # Status indicator (Online/Offline)
        self.status_label = QLabel("🔴 OFFLINE")
        self.status_label.setFont(QFont("Arial", 11, QFont.Bold))
        self.status_label.setObjectName("statusLabel")
        self.status_label.setAlignment(Qt.AlignCenter)
        system_layout.addWidget(self.status_label)
        
        # Battery section
        battery_container = QWidget()
        battery_layout = QVBoxLayout()
        battery_layout.setContentsMargins(0, 2, 0, 2)
        battery_layout.setSpacing(2)
        
        battery_label = QLabel("Battery")
        battery_label.setFont(QFont("Arial", 8))
        battery_label.setObjectName("subLabel")
        battery_label.setAlignment(Qt.AlignCenter)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setMinimum(0)
        self.battery_bar.setMaximum(100)
        self.battery_bar.setValue(0)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setFixedHeight(18)
        self._update_battery_style(0)
        
        battery_layout.addWidget(battery_label)
        battery_layout.addWidget(self.battery_bar)
        battery_container.setLayout(battery_layout)
        system_layout.addWidget(battery_container)
        
        # Mode indicator
        self.mode_label = QLabel("MANUAL")
        self.mode_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.mode_label.setObjectName("modeLabel")
        self.mode_label.setAlignment(Qt.AlignCenter)
        system_layout.addWidget(self.mode_label)
        
        system_frame.setLayout(system_layout)
        main_layout.addWidget(system_frame, 1)  # stretch factor 1
        
        # =====================================================
        # COLUMN 2: POSITION & SENSORS
        # =====================================================
        sensors_frame = QFrame()
        sensors_frame.setObjectName("sensorsFrame")
        sensors_layout = QVBoxLayout()
        sensors_layout.setContentsMargins(10, 6, 10, 6)
        sensors_layout.setSpacing(2)
        
        # Title
        sensors_title = QLabel("POSITION & SENSORS")
        sensors_title.setFont(QFont("Arial", 9, QFont.Bold))
        sensors_title.setAlignment(Qt.AlignCenter)
        sensors_layout.addWidget(sensors_title)
        
        # Create grid for sensor data
        sensor_grid = QWidget()
        sensor_grid_layout = QGridLayout()
        sensor_grid_layout.setContentsMargins(4, 4, 4, 4)
        sensor_grid_layout.setSpacing(2)
        sensor_grid_layout.setHorizontalSpacing(6)
        
        # Row 0: Attitude
        attitude_name = QLabel("Attitude :")
        attitude_name.setFont(QFont("Arial", 9))
        attitude_name.setObjectName("labelName")
        sensor_grid_layout.addWidget(attitude_name, 0, 0)
        
        self.attitude_values = QLabel("P: 0°  R: 0°  Y: 0°")
        self.attitude_values.setFont(QFont("Arial", 9, QFont.Bold))
        self.attitude_values.setObjectName("labelValue")
        sensor_grid_layout.addWidget(self.attitude_values, 0, 1)
        
        # Row 1: Altitude
        altitude_name = QLabel("Altitude :")
        altitude_name.setFont(QFont("Arial", 9))
        altitude_name.setObjectName("labelName")
        sensor_grid_layout.addWidget(altitude_name, 1, 0)
        
        self.altitude_values = QLabel("Bar: -- cm  /  ToF: -- cm")
        self.altitude_values.setFont(QFont("Arial", 9, QFont.Bold))
        self.altitude_values.setObjectName("labelValue")
        sensor_grid_layout.addWidget(self.altitude_values, 1, 1)
        
        # Row 2: Flight Time
        time_name = QLabel("Flight T :")
        time_name.setFont(QFont("Arial", 9))
        time_name.setObjectName("labelName")
        sensor_grid_layout.addWidget(time_name, 2, 0)
        
        self.flight_time_label = QLabel("00:00:00")
        self.flight_time_label.setFont(QFont("Arial", 9, QFont.Bold))
        self.flight_time_label.setObjectName("labelValue")
        sensor_grid_layout.addWidget(self.flight_time_label, 2, 1)
        
        # Row 3: Temperature
        temp_name = QLabel("Temp :")
        temp_name.setFont(QFont("Arial", 9))
        temp_name.setObjectName("labelName")
        sensor_grid_layout.addWidget(temp_name, 3, 0)
        
        self.temp_label = QLabel("--°C")
        self.temp_label.setFont(QFont("Arial", 9, QFont.Bold))
        self.temp_label.setObjectName("labelValue")
        sensor_grid_layout.addWidget(self.temp_label, 3, 1)
        
        sensor_grid.setLayout(sensor_grid_layout)
        sensors_layout.addWidget(sensor_grid)
        sensors_frame.setLayout(sensors_layout)
        main_layout.addWidget(sensors_frame, 2)  # stretch factor 2
        
        # =====================================================
        # COLUMN 3: MOTION LOOP (cmd_vel ⇋ drone velocity)
        # =====================================================
        motion_frame = QFrame()
        motion_frame.setObjectName("motionFrame")
        motion_layout = QVBoxLayout()
        motion_layout.setContentsMargins(10, 6, 10, 6)
        motion_layout.setSpacing(2)
        
        # Title
        motion_title = QLabel("MOTION LOOP")
        motion_title.setFont(QFont("Arial", 9, QFont.Bold))
        motion_title.setObjectName("motionTitle")
        motion_title.setAlignment(Qt.AlignCenter)
        motion_layout.addWidget(motion_title)
        
        # Header row
        header_widget = QWidget()
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)
        
        header_axis = QLabel("Axis")
        header_axis.setFont(QFont("Arial", 8))
        header_axis.setObjectName("headerLabel")
        header_axis.setFixedWidth(70)
        
        header_cmd = QLabel("Cmd (m/s)")
        header_cmd.setFont(QFont("Arial", 8))
        header_cmd.setObjectName("headerCmd")
        header_cmd.setAlignment(Qt.AlignCenter)
        
        header_arrow = QLabel("⇋")
        header_arrow.setFont(QFont("Arial", 9))
        header_arrow.setObjectName("headerArrow")
        header_arrow.setAlignment(Qt.AlignCenter)
        header_arrow.setFixedWidth(16)
        
        header_vel = QLabel("Drone (cm/s)")
        header_vel.setFont(QFont("Arial", 8))
        header_vel.setObjectName("headerVel")
        header_vel.setAlignment(Qt.AlignCenter)
        
        header_layout.addWidget(header_axis)
        header_layout.addWidget(header_cmd, 1)
        header_layout.addWidget(header_arrow)
        header_layout.addWidget(header_vel, 1)
        header_widget.setLayout(header_layout)
        motion_layout.addWidget(header_widget)
        
        # Motion data rows
        self.motion_rows = {}
        motion_items = [
            ("Lin X (Fwd)", "lx"),
            ("Lin Y (Lat)", "ly"),
            ("Lin Z (Up)", "lz"),
            ("Ang Z (Yaw)", "az")
        ]
        
        for label_text, key in motion_items:
            row_widget = QWidget()
            row_layout = QHBoxLayout()
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(4)
            
            axis_label = QLabel(label_text)
            axis_label.setFont(QFont("Arial", 9))
            axis_label.setObjectName("axisLabel")
            axis_label.setFixedWidth(70)
            
            cmd_label = QLabel("0.00")
            cmd_label.setFont(QFont("Arial", 9, QFont.Bold))
            cmd_label.setObjectName("cmdValue")
            cmd_label.setAlignment(Qt.AlignCenter)
            
            arrow_label = QLabel("⇋")
            arrow_label.setFont(QFont("Arial", 9))
            arrow_label.setObjectName("arrowLabel")
            arrow_label.setAlignment(Qt.AlignCenter)
            arrow_label.setFixedWidth(16)
            
            vel_label = QLabel("0")
            vel_label.setFont(QFont("Arial", 9, QFont.Bold))
            vel_label.setObjectName("velValue")
            vel_label.setAlignment(Qt.AlignCenter)
            
            row_layout.addWidget(axis_label)
            row_layout.addWidget(cmd_label, 1)
            row_layout.addWidget(arrow_label)
            row_layout.addWidget(vel_label, 1)
            row_widget.setLayout(row_layout)
            motion_layout.addWidget(row_widget)
            
            # Store references
            self.motion_rows[key] = {
                'cmd': cmd_label,
                'vel': vel_label
            }
        
        motion_frame.setLayout(motion_layout)
        main_layout.addWidget(motion_frame, 2)  # stretch factor 2
        
        self.setLayout(main_layout)
        self.setFixedHeight(130)
        
        # Apply centralized stylesheet
        self.setStyleSheet("""
            TelemetryWidget {
                background-color: #0d0d1a;
            }
            QFrame#systemFrame, QFrame#sensorsFrame {
                background-color: #1a1a2e;
                border: 1px solid #2a2a4a;
                border-radius: 6px;
            }
            QFrame#motionFrame {
                background-color: #251a2e;
                border: 1px solid #3a2a4a;
                border-radius: 6px;
            }
            QLabel {
                color: #00bcd4;
                background: transparent;
                border: none;
            }
            QLabel#statusLabel {
                color: #F44336;
            }
            QLabel#subLabel {
                color: #666;
            }
            QLabel#modeLabel {
                color: #2196F3;
            }
            QLabel#labelName {
                color: #888;
            }
            QLabel#labelValue {
                color: #4CAF50;
            }
            QLabel#motionTitle {
                color: #E91E63;
            }
            QLabel#headerLabel, QLabel#headerArrow {
                color: #555;
            }
            QLabel#headerCmd {
                color: #E91E63;
            }
            QLabel#headerVel {
                color: #4CAF50;
            }
            QLabel#axisLabel {
                color: #999;
            }
            QLabel#cmdValue, QLabel#arrowLabel, QLabel#velValue {
                color: #666;
            }
            QWidget {
                background: transparent;
            }
        """)
    
    def _update_battery_style(self, battery):
        """Update battery bar color based on level"""
        if battery > 50:
            color = "#4CAF50"  # Green
            text_color = "white"
        elif battery > 20:
            color = "#FF9800"  # Orange
            text_color = "black"
        else:
            color = "#F44336"  # Red
            text_color = "white"
        
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #444;
                border-radius: 5px;
                text-align: center;
                background-color: #333;
                color: {text_color};
                font-weight: bold;
                font-size: 10px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 4px;
            }}
        """)
    
    def _format_flight_time(self, seconds):
        """Format flight time as HH:MM:SS"""
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    
    def _update_motion_value(self, label, value, is_cmd=True):
        """Update motion label with color coding based on value"""
        if is_cmd:
            label.setText(f"{value:.2f}")
            if abs(value) > 0.05:
                label.setStyleSheet("color: #E91E63; font-weight: bold; border: none;")  # Pink when active
            else:
                label.setStyleSheet("color: #666; border: none;")  # Gray when zero
        else:
            label.setText(f"{int(value)}")
            if abs(value) > 1:
                label.setStyleSheet("color: #4CAF50; font-weight: bold; border: none;")  # Green when active
            else:
                label.setStyleSheet("color: #666; border: none;")  # Gray when zero
    
    def update_telemetry(self, flight_data):
        """Update telemetry display"""
        try:
            # Battery
            battery = flight_data.get('battery', 0)
            self.battery_bar.setValue(battery)
            self._update_battery_style(battery)
            
            # Attitude
            pitch = flight_data.get('pitch', 0)
            roll = flight_data.get('roll', 0)
            yaw = flight_data.get('yaw', 0)
            self.attitude_values.setText(f"P: {pitch}°  R: {roll}°  Y: {yaw}°")
            
            # Altitude
            altitude = flight_data.get('altitude', 0)
            tof = flight_data.get('tof', 0)
            self.altitude_values.setText(f"Bar: {altitude} cm  /  ToF: {tof} cm")
            
            # Flight time
            flight_time = flight_data.get('flight_time', 0)
            self.flight_time_seconds = flight_time
            self.flight_time_label.setText(self._format_flight_time(flight_time))
            
            # Temperature
            temp_low = flight_data.get('temperature_low', 0)
            temp_high = flight_data.get('temperature_high', 0)
            self.temp_label.setText(f"{temp_low}-{temp_high}°C")
            
            # Velocity (for motion loop drone velocity)
            vx = flight_data.get('velocity_x', 0)
            vy = flight_data.get('velocity_y', 0)
            vz = flight_data.get('velocity_z', 0)
            
            # Update drone velocity in motion rows
            if 'lx' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lx']['vel'], vx, is_cmd=False)
            if 'ly' in self.motion_rows:
                self._update_motion_value(self.motion_rows['ly']['vel'], vy, is_cmd=False)
            if 'lz' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lz']['vel'], vz, is_cmd=False)
            
            # Update status
            if battery > 0:
                self.status_label.setText("🟢 ONLINE")
                self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold; border: none;")
            else:
                self.status_label.setText("🔴 OFFLINE")
                self.status_label.setStyleSheet("color: #F44336; font-weight: bold; border: none;")
                
        except Exception as e:
            print(f"Error updating telemetry: {e}")
    
    def update_cmd_vel(self, twist_msg):
        """Update cmd_vel display in motion loop"""
        try:
            # Linear velocities
            lx = twist_msg.linear.x
            ly = twist_msg.linear.y
            lz = twist_msg.linear.z
            
            # Angular velocities
            az = twist_msg.angular.z
            
            # Update cmd values in motion rows
            if 'lx' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lx']['cmd'], lx, is_cmd=True)
            if 'ly' in self.motion_rows:
                self._update_motion_value(self.motion_rows['ly']['cmd'], ly, is_cmd=True)
            if 'lz' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lz']['cmd'], lz, is_cmd=True)
            if 'az' in self.motion_rows:
                self._update_motion_value(self.motion_rows['az']['cmd'], az, is_cmd=True)
                # For angular, also show in deg/s equivalent (approx * 57.3)
                deg_per_sec = az * 57.3
                self._update_motion_value(self.motion_rows['az']['vel'], deg_per_sec, is_cmd=False)
                
        except Exception as e:
            print(f"Error updating cmd_vel: {e}")
    
    def update_mode(self, mode):
        """Update mode display"""
        self.mode_label.setText(mode.upper())
        
        colors = {
            'manual': '#2196F3',    # Blue
            'joystick': '#9C27B0',  # Purple
            'gesture': '#E91E63',   # Pink
            'tracking': '#FF9800'   # Orange
        }
        color = colors.get(mode.lower(), '#2196F3')
        self.mode_label.setStyleSheet(f"color: {color}; font-weight: bold; border: none;")


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
        
        # Keyboard state tracking
        self.keys_pressed = set()
        
        # Enable keyboard focus
        self.setFocusPolicy(Qt.StrongFocus)
        
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
        
        self.takeoff_btn = self.create_action_button("Takeoff (T)", "#4CAF50", "takeoff")
        self.land_btn = self.create_action_button("Land (L)", "#2196F3", "land")
        self.emergency_btn = self.create_action_button("EMERGENCY", "#F44336", "emergency")
        self.hover_btn = self.create_action_button("Hover (H)", "#FF9800", "hover")
        
        # Flip buttons
        self.flip_f_btn = self.create_action_button("Flip ↑", "#9C27B0", "flip_f")
        self.flip_b_btn = self.create_action_button("Flip ↓", "#9C27B0", "flip_b")
        self.flip_l_btn = self.create_action_button("Flip ←", "#9C27B0", "flip_l")
        self.flip_r_btn = self.create_action_button("Flip →", "#9C27B0", "flip_r")
        
        action_layout.addWidget(self.takeoff_btn, 0, 0)
        action_layout.addWidget(self.land_btn, 0, 1)
        action_layout.addWidget(self.hover_btn, 1, 0)
        action_layout.addWidget(self.emergency_btn, 1, 1)
        action_layout.addWidget(self.flip_f_btn, 2, 0)
        action_layout.addWidget(self.flip_b_btn, 2, 1)
        action_layout.addWidget(self.flip_l_btn, 3, 0)
        action_layout.addWidget(self.flip_r_btn, 3, 1)
        
        action_group.setLayout(action_layout)
        layout.addWidget(action_group)
        
        # Keyboard status and instructions (compact)
        keyboard_group = QGroupBox("⌨️ Controls (WASD + QEHL + T/Space/Shift)")
        keyboard_layout = QVBoxLayout()
        
        self.keyboard_status = QLabel("🎮 Click panel to enable keyboard control")
        self.keyboard_status.setAlignment(Qt.AlignCenter)
        self.keyboard_status.setStyleSheet("color: #666; padding: 5px; font-size: 11px;")
        
        keyboard_layout.addWidget(self.keyboard_status)
        keyboard_group.setLayout(keyboard_layout)
        layout.addWidget(keyboard_group)
        
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
    
    def update_speed(self, value):
        """Update speed factor"""
        self.speed_factor = value / 10.0
        self.speed_label.setText(f"Speed: {int(self.speed_factor * 100)}%")
    
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
    
    def keyPressEvent(self, event):
        """Handle keyboard press for drone control"""
        key = event.key()
        
        if key in self.keys_pressed:
            return  # Ignore key repeat
        
        self.keys_pressed.add(key)
        
        # Movement keys
        if key == Qt.Key_W:
            self.vx = self.speed_factor
        elif key == Qt.Key_S:
            self.vx = -self.speed_factor
        elif key == Qt.Key_A:
            self.vy = self.speed_factor
        elif key == Qt.Key_D:
            self.vy = -self.speed_factor
        elif key == Qt.Key_Space:
            self.vz = self.speed_factor
        elif key == Qt.Key_Shift:
            self.vz = -self.speed_factor
        elif key == Qt.Key_Q:
            self.vw = self.speed_factor
        elif key == Qt.Key_E:
            self.vw = -self.speed_factor
        # Action keys
        elif key == Qt.Key_T:
            self.action_requested.emit('takeoff')
        elif key == Qt.Key_L:
            self.action_requested.emit('land')
        elif key == Qt.Key_H:
            self.stop_movement()
            self.action_requested.emit('hover')
        elif key == Qt.Key_Escape:
            self.stop_movement()
            self.action_requested.emit('emergency')
        
        event.accept()
    
    def keyReleaseEvent(self, event):
        """Handle keyboard release"""
        key = event.key()
        
        if key not in self.keys_pressed:
            return
        
        self.keys_pressed.discard(key)
        
        # Stop movement for released key
        if key == Qt.Key_W and self.vx > 0:
            self.vx = 0.0
        elif key == Qt.Key_S and self.vx < 0:
            self.vx = 0.0
        elif key == Qt.Key_A and self.vy > 0:
            self.vy = 0.0
        elif key == Qt.Key_D and self.vy < 0:
            self.vy = 0.0
        elif key == Qt.Key_Space and self.vz > 0:
            self.vz = 0.0
        elif key == Qt.Key_Shift and self.vz < 0:
            self.vz = 0.0
        elif key == Qt.Key_Q and self.vw > 0:
            self.vw = 0.0
        elif key == Qt.Key_E and self.vw < 0:
            self.vw = 0.0
        
        event.accept()
    
    def focusInEvent(self, event):
        """Update status when keyboard focus gained"""
        super().focusInEvent(event)
        self.keyboard_status.setText("⌨️ Keyboard Active - Use WASD")
        self.keyboard_status.setStyleSheet("color: #4CAF50; padding: 5px; font-weight: bold;")
    
    def focusOutEvent(self, event):
        """Update status when keyboard focus lost"""
        super().focusOutEvent(event)
        self.keys_pressed.clear()
        self.stop_movement()
        self.keyboard_status.setText("🎮 Click here to enable keyboard")
        self.keyboard_status.setStyleSheet("color: #666; padding: 5px;")


class GestureControlWidget(QWidget):
    """Widget untuk gesture control status"""
    
    camera_switched = pyqtSignal(str)  # 'webcam' or 'drone'
    
    def __init__(self):
        super().__init__()
        self.using_webcam = True  # Default to webcam for gesture
        self.gesture_mode = False  # Keep for compatibility
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Gesture Mode")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Camera source toggle
        camera_group = QGroupBox("Camera Source")
        camera_layout = QVBoxLayout()
        
        self.camera_btn = QPushButton("Switch to Drone Camera")
        self.camera_btn.setMinimumHeight(60)
        self.camera_btn.setCheckable(True)
        self.camera_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
            QPushButton:checked {
                background-color: #E91E63;
            }
        """)
        self.camera_btn.clicked.connect(self.toggle_camera)
        
        self.camera_status = QLabel("Using Webcam for Gesture")
        self.camera_status.setFont(QFont("Arial", 12))
        self.camera_status.setAlignment(Qt.AlignCenter)
        self.camera_status.setStyleSheet("color: #9C27B0; padding: 10px;")
        
        camera_layout.addWidget(self.camera_btn)
        camera_layout.addWidget(self.camera_status)
        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)
        
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
        instructions_group = QGroupBox("Gesture Instructions")
        instructions_layout = QVBoxLayout()
        
        instructions = QLabel(
            "<b>Gesture Controls:</b><br>"
            "• ✋ Open hand: Hover/Stop<br>"
            "• ✊ Closed fist: Land<br>"
            "• 👆 Point up: Takeoff<br>"
            "• 👈 Point left: Move left<br>"
            "• 👉 Point right: Move right<br>"
            "• ✌️ Peace sign: Emergency stop<br><br>"
            "<b>Camera Source:</b><br>"
            "• Webcam: Use laptop camera<br>"
            "• Drone: Use drone camera"
        )
        instructions.setWordWrap(True)
        instructions.setStyleSheet("padding: 10px;")
        
        instructions_layout.addWidget(instructions)
        instructions_group.setLayout(instructions_layout)
        layout.addWidget(instructions_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def toggle_camera(self):
        """Toggle between webcam and drone camera for gesture recognition"""
        self.using_webcam = not self.camera_btn.isChecked()
        
        if self.using_webcam:
            self.camera_btn.setText("Switch to Drone Camera")
            self.camera_status.setText("Using Webcam for Gesture")
            self.camera_status.setStyleSheet("color: #9C27B0; padding: 10px;")
            self.camera_switched.emit('webcam')
        else:
            self.camera_btn.setText("Switch to Webcam")
            self.camera_status.setText("Using Drone Camera for Gesture")
            self.camera_status.setStyleSheet("color: #E91E63; padding: 10px; font-weight: bold;")
            self.camera_switched.emit('drone')
    
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
        
        self.setup_ui()
        self.setWindowTitle("Tello Drone Control Center")
        self.resize(1400, 800)
        
        # Track current control mode for gesture/raw webcam display logic
        self.current_mode = 'manual'
        
        # Connect signals from signal emitter
        self.signal_emitter.image_signal.connect(self.update_video)
        # Webcam frames (local laptop camera) - for gesture widget when not in gesture mode
        self.signal_emitter.webcam_image_signal.connect(self.update_webcam)
        self.signal_emitter.flight_data_signal.connect(self.update_telemetry)
        self.signal_emitter.gesture_status_signal.connect(self.update_gesture)
        
        # Connect annotation signals to drone video widget
        self.signal_emitter.detections_signal.connect(self.drone_video_widget.update_detections)
        self.signal_emitter.aruco_signal.connect(self.drone_video_widget.update_aruco)
        self.signal_emitter.distance_signal.connect(self.drone_video_widget.update_distances)
        self.signal_emitter.tracking_signal.connect(self.drone_video_widget.update_tracking)
        self.signal_emitter.gesture_status_signal.connect(self.gesture_video_widget.update_gesture_data)
        self.signal_emitter.mode_signal.connect(self.update_mode_ui)
        
        # Connect cmd_vel signal to telemetry widget
        self.signal_emitter.cmd_vel_signal.connect(self.update_cmd_vel)
        
        # Gesture annotated image (skeleton) - display on gesture video widget
        self.signal_emitter.gesture_annotated_signal.connect(self.update_gesture_annotated)
        
        # Track which camera is used for gesture (webcam by default)
        self.gesture_camera_source = 'webcam'
    
    def setup_ui(self):
        """Setup main UI"""
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Left panel - Video displays and telemetry
        left_panel = QVBoxLayout()
        
        # Create two video widgets: drone (left) with annotations, gesture/webcam (right)
        video_row = QHBoxLayout()
        
        # Drone video widget - shows drone camera with YOLO/ArUco/distance/tracking annotations
        self.drone_video_widget = VideoWidget()
        self.drone_video_widget.setMinimumSize(480, 360)
        
        # Gesture/Webcam video widget - shows gesture skeleton or raw webcam
        self.gesture_video_widget = VideoWidget()
        self.gesture_video_widget.setMinimumSize(480, 360)
        
        video_row.addWidget(self.drone_video_widget)
        video_row.addWidget(self.gesture_video_widget)
        left_panel.addLayout(video_row, stretch=3)

        # Telemetry widget below video displays
        self.telemetry_widget = TelemetryWidget()
        self.telemetry_widget.setMaximumHeight(200)
        left_panel.addWidget(self.telemetry_widget, stretch=1)

        # Configure captions and overlay behavior
        try:
            self.drone_video_widget.set_caption("Drone Camera")
            self.drone_video_widget.set_overlay_enabled(False)  # Telemetry now in separate widget below
            self.drone_video_widget.show_fps_overlay = True  # Show FPS on video
            
            self.gesture_video_widget.set_caption("Gesture (Webcam)")
            self.gesture_video_widget.set_overlay_enabled(False)
            self.gesture_video_widget.show_fps_overlay = True  # Show FPS on video
            self.gesture_video_widget.is_webcam = True
        except Exception:
            pass
        
        # Right panel - Controls
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        # Mode Selection
        mode_select_layout = QHBoxLayout()
        mode_label = QLabel("Current Mode:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Manual", "Joystick", "Gesture", "Tracking"])
        self.mode_combo.currentTextChanged.connect(self.change_control_mode)
        
        mode_select_layout.addWidget(mode_label)
        mode_select_layout.addWidget(self.mode_combo)
        right_layout.addLayout(mode_select_layout)
        
        # View Options - Annotation toggles
        view_group = QGroupBox("View Options")
        view_layout = QGridLayout()
        view_layout.setSpacing(8)
        
        self.chk_yolo = QCheckBox("YOLO")
        self.chk_aruco = QCheckBox("ArUco")
        self.chk_distance = QCheckBox("Distance")
        self.chk_tracking = QCheckBox("Tracking")
        
        # Style checkboxes
        self.chk_yolo.setStyleSheet("QCheckBox { color: #4CAF50; }")
        self.chk_aruco.setStyleSheet("QCheckBox { color: #00BCD4; }")
        self.chk_distance.setStyleSheet("QCheckBox { color: #FFEB3B; }")
        self.chk_tracking.setStyleSheet("QCheckBox { color: #FF5722; }")
        
        self.chk_yolo.setChecked(True)
        self.chk_aruco.setChecked(True)
        self.chk_distance.setChecked(True)
        self.chk_tracking.setChecked(True)
        
        self.chk_yolo.toggled.connect(lambda v: setattr(self.drone_video_widget, 'show_yolo', v))
        self.chk_aruco.toggled.connect(lambda v: setattr(self.drone_video_widget, 'show_aruco', v))
        self.chk_distance.toggled.connect(lambda v: setattr(self.drone_video_widget, 'show_distance', v))
        self.chk_tracking.toggled.connect(lambda v: setattr(self.drone_video_widget, 'show_tracking', v))
        
        view_layout.addWidget(self.chk_yolo, 0, 0)
        view_layout.addWidget(self.chk_aruco, 0, 1)
        view_layout.addWidget(self.chk_distance, 0, 2)
        view_layout.addWidget(self.chk_tracking, 0, 3)
        view_group.setLayout(view_layout)
        right_layout.addWidget(view_group)

        # Tab widget for different control modes
        self.tab_widget = QTabWidget()
        
        self.manual_control = ManualControlWidget()
        self.manual_control.velocity_changed.connect(self.send_velocity)
        self.manual_control.action_requested.connect(self.send_action)
        
        self.gesture_control = GestureControlWidget()
        self.gesture_control.camera_switched.connect(self.switch_gesture_camera)
        
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
        
        # Start webcam capture thread so laptop camera is available immediately.
        try:
            self.webcam_thread = WebcamThread(self.signal_emitter, self.ros_node, device=0)
            self.webcam_thread.start()
        except Exception:
            self.webcam_thread = None
        
    def update_video(self, cv_image):
        """Update drone video display with annotations."""
        try:
            self.drone_video_widget.update_image(cv_image)
        except Exception:
            pass
    
    def update_webcam(self, cv_image):
        """Update webcam display - show on gesture widget if not in gesture mode."""
        try:
            # Publish to ROS so gesture detector receives webcam frames
            try:
                self.ros_node.publish_webcam_frame(cv_image)
            except Exception:
                pass
            
            # If not in gesture mode, display raw webcam on gesture widget
            # In gesture mode, the gesture_annotated signal will update it
            if self.current_mode != 'gesture':
                self.gesture_video_widget.update_image(cv_image)
        except Exception:
            pass
    
    def update_telemetry(self, flight_data):
        """Update telemetry display widget below video."""
        try:
            self.telemetry_widget.update_telemetry(flight_data)
        except Exception:
            pass
    
    def update_cmd_vel(self, twist_msg):
        """Update cmd_vel display in telemetry widget."""
        try:
            self.telemetry_widget.update_cmd_vel(twist_msg)
        except Exception:
            pass
    
    def update_gesture(self, gesture):
        """Update gesture status"""
        self.gesture_control.update_gesture_status(gesture)
    
    def update_gesture_annotated(self, cv_image):
        """Update the gesture video widget with skeleton annotated image."""
        try:
            # Always display skeleton on dedicated gesture widget
            self.gesture_video_widget.update_image(cv_image)
        except Exception:
            pass
    
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
    
    def change_control_mode(self, mode_text, send_request=True):
        """Handle mode change from UI or other widgets."""
        # If boolean (from gesture widget toggle), convert to string
        if isinstance(mode_text, bool):
            mode = 'gesture' if mode_text else 'manual'
        else:
            mode = mode_text.lower()

        # Track current mode for gesture/raw webcam display logic
        self.current_mode = mode

        if send_request:
            self.ros_node.set_mode(mode)
        
        # Update UI based on mode
        if mode == 'gesture':
            # Highlight gesture widget
            self.gesture_video_widget.setStyleSheet("border: 3px solid #E91E63;")  # Pink
            self.drone_video_widget.setStyleSheet("border: 2px solid #2196F3;")
            # Update caption to show gesture source
            src = self.gesture_camera_source.capitalize()
            self.gesture_video_widget.set_caption(f"Gesture ({src})")
            # Switch tab
            self.tab_widget.setCurrentWidget(self.gesture_control)
        elif mode == 'tracking':
            self.drone_video_widget.setStyleSheet("border: 3px solid #FF9800;")  # Orange
            self.gesture_video_widget.setStyleSheet("border: 2px solid #2196F3;")
            self.gesture_video_widget.set_caption("Webcam")
        else:
            self.drone_video_widget.setStyleSheet("border: 3px solid #2196F3;")  # Blue
            self.gesture_video_widget.setStyleSheet("border: 2px solid #2196F3;")
            self.gesture_video_widget.set_caption("Webcam")
            # Switch tab
            self.tab_widget.setCurrentWidget(self.manual_control)
            
        # Enable/disable manual control
        self.manual_control.setEnabled(mode == 'manual')
        
        print(f"Switched to {mode} Control Mode")

    def update_mode_ui(self, mode):
        """Update UI elements when mode changes externally."""
        
        # Update telemetry widget mode display
        try:
            self.telemetry_widget.update_mode(mode)
        except Exception:
            pass
        
        # Update visual indicators
        self.change_control_mode(mode, send_request=False)
    
    def switch_gesture_camera(self, source):
        """Switch camera source for gesture recognition using dedicated service."""
        print(f"Switching gesture camera to: {source}")
        
        # Update local tracking of gesture camera source
        self.gesture_camera_source = source
        
        # Update gesture widget caption to show current source
        if self.current_mode == 'gesture':
            if source == 'webcam':
                self.gesture_video_widget.set_caption("Gesture (Webcam)")
            else:
                self.gesture_video_widget.set_caption("Gesture (Drone)")
        
        # Send camera source change via SetGestureCamera service
        if self.ros_node.gesture_camera_client.service_is_ready():
            request = SetGestureCamera.Request()
            request.camera_source = source
            future = self.ros_node.gesture_camera_client.call_async(request)
            future.add_done_callback(self._gesture_camera_callback)
        else:
            self.ros_node.get_logger().warn('Gesture camera switch service not available')
    
    def _gesture_camera_callback(self, future):
        """Handle gesture camera switch service response."""
        try:
            result = future.result()
            if result.success:
                self.ros_node.get_logger().info(f'Gesture camera: {result.message}')
            else:
                self.ros_node.get_logger().warn(f'Gesture camera switch failed: {result.message}')
        except Exception as e:
            self.ros_node.get_logger().error(f'Gesture camera switch error: {e}')

    def closeEvent(self, event):
        """Handle window close"""
        print("Shutting down...")
        
        try:
            if getattr(self, 'webcam_thread', None) is not None:
                self.webcam_thread.stop()
                self.webcam_thread.wait()
        except Exception:
            pass

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
