"""Main window for Tello Control GUI application."""

import rclpy
import qtawesome as qta

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QGroupBox, QGridLayout,
    QTabWidget, QPushButton, QCheckBox
)
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QTimer

from tello_interfaces.srv import SetGestureCamera

from .core import SignalEmitter, ROSThread, TelloControlNode
from .widgets import (
    VideoWidget, TelemetryWidget, ManualControlWidget,
    GestureControlWidget, ExportRecordWidget, SpeedControlWidget,
    ArucoNamingDialog
)
from .threads import WebcamThread


class MainWindow(QMainWindow):
    """Main application window.
    
    Features:
    - Dual video display (drone + gesture/webcam)
    - Telemetry monitoring
    - Multiple control modes (Manual, Joystick, Gesture, Tracking)
    - Video recording and export
    """
    
    def __init__(self, cv2_module=None):
        """Initialize main window.
        
        Args:
            cv2_module: OpenCV module reference for video operations
        """
        super().__init__()
        
        self._cv2 = cv2_module
        
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
        
        # Apply dark theme stylesheet
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #e0e0e0;
            }
            QGroupBox {
                font-weight: bold;
                color: #888;
                border: 1px solid #333;
                border-radius: 5px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel {
                color: #e0e0e0;
            }
            QComboBox {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #444;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QComboBox:hover {
                border-color: #555;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #2d2d2d;
                color: #e0e0e0;
                selection-background-color: #3d3d3d;
            }
            QPushButton {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #444;
                border-radius: 4px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #3d3d3d;
                border-color: #555;
            }
            QPushButton:pressed {
                background-color: #252525;
            }
            QPushButton:disabled {
                background-color: #1a1a1a;
                color: #666;
            }
            QCheckBox {
                color: #e0e0e0;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 1px solid #555;
                border-radius: 3px;
                background-color: #2d2d2d;
            }
            QCheckBox::indicator:checked {
                background-color: #4CAF50;
                border-color: #4CAF50;
            }
            QTabWidget::pane {
                border: 1px solid #333;
                background-color: #252525;
                border-radius: 4px;
            }
            QTabBar::tab {
                background-color: #2d2d2d;
                color: #aaa;
                border: 1px solid #333;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #252525;
                color: #e0e0e0;
                border-bottom-color: #252525;
            }
            QTabBar::tab:hover:!selected {
                background-color: #353535;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background-color: #333;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background-color: #4CAF50;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background-color: #66BB6A;
            }
            QScrollBar:vertical {
                background-color: #1e1e1e;
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background-color: #444;
                border-radius: 6px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #555;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
        """)
        
        # Track current control mode for gesture/raw webcam display logic
        self.current_mode = 'manual'
        
        # ArUco marker names dictionary {marker_id: name}
        self.aruco_marker_names = {}
        
        # Connect signals from signal emitter
        self.signal_emitter.image_signal.connect(self.update_video)
        self.signal_emitter.flight_data_signal.connect(self.update_telemetry)
        self.signal_emitter.gesture_status_signal.connect(self.update_gesture)
        
        # Connect annotation signals to drone video widget
        self.signal_emitter.detections_signal.connect(
            self.drone_video_widget.update_detections
        )
        self.signal_emitter.aruco_signal.connect(
            self.drone_video_widget.update_aruco
        )
        self.signal_emitter.aruco_ids_signal.connect(
            self.drone_video_widget.update_aruco_ids
        )
        self.signal_emitter.distance_signal.connect(
            self.drone_video_widget.update_distances
        )
        self.signal_emitter.tracking_signal.connect(
            self.drone_video_widget.update_tracking
        )
        self.signal_emitter.gesture_status_signal.connect(
            self.gesture_video_widget.update_gesture_data
        )
        self.signal_emitter.mode_signal.connect(self.update_mode_ui)
        
        # Connect cmd_vel signal to telemetry widget
        self.signal_emitter.cmd_vel_signal.connect(self.update_cmd_vel)
        
        # Gesture annotated image (skeleton) - display on gesture video widget
        self.signal_emitter.gesture_annotated_signal.connect(
            self.update_gesture_annotated
        )
        
        # Webcam raw frames - display on gesture widget when not in gesture mode
        self.signal_emitter.webcam_image_signal.connect(
            self.update_webcam
        )
        
        # Track which camera is used for gesture (webcam by default)
        self.gesture_camera_source = 'webcam'
    
    def setup_ui(self):
        """Setup main UI."""
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Left panel - Video displays and telemetry
        left_panel = QVBoxLayout()
        
        # Create two video widgets: drone (left), gesture/webcam (right)
        video_row = QHBoxLayout()
        
        # Drone video widget - shows drone camera with YOLO/ArUco/distance/tracking
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
        self.telemetry_widget.reconnect_clicked.connect(self.on_reconnect_clicked)
        left_panel.addWidget(self.telemetry_widget, stretch=1)

        # Configure captions and overlay behavior
        try:
            self.drone_video_widget.set_caption("Drone Camera")
            self.drone_video_widget.set_overlay_enabled(False)
            self.drone_video_widget.show_fps_overlay = True
            
            self.gesture_video_widget.set_caption("Gesture (Webcam)")
            self.gesture_video_widget.set_overlay_enabled(False)
            self.gesture_video_widget.show_fps_overlay = True
            self.gesture_video_widget.is_webcam = True
            
            self.gesture_video_widget._no_frame_count = 0
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
        
        self.chk_yolo.toggled.connect(
            lambda v: setattr(self.drone_video_widget, 'show_yolo', v)
        )
        self.chk_aruco.toggled.connect(
            lambda v: setattr(self.drone_video_widget, 'show_aruco', v)
        )
        self.chk_distance.toggled.connect(
            lambda v: setattr(self.drone_video_widget, 'show_distance', v)
        )
        self.chk_tracking.toggled.connect(
            lambda v: setattr(self.drone_video_widget, 'show_tracking', v)
        )
        
        # Drone camera mirror button (toggle)
        self.mirror_btn = QPushButton()
        self.mirror_btn.setIcon(qta.icon('mdi.flip-horizontal', color='white'))
        self.mirror_btn.setToolTip("Mirror drone camera")
        self.mirror_btn.setFixedSize(32, 32)
        self.mirror_btn.setCheckable(True)
        self.mirror_btn.setStyleSheet("""
            QPushButton {
                background-color: #555;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #666;
            }
            QPushButton:checked {
                background-color: #9C27B0;
            }
            QPushButton:checked:hover {
                background-color: #7B1FA2;
            }
        """)
        self.mirror_btn.toggled.connect(self.toggle_drone_mirror)
        
        # ArUco naming button
        self.aruco_naming_btn = QPushButton()
        self.aruco_naming_btn.setIcon(qta.icon('mdi.tag-text-outline', color='white'))
        self.aruco_naming_btn.setToolTip("ArUco Marker Names")
        self.aruco_naming_btn.setFixedSize(32, 32)
        self.aruco_naming_btn.setStyleSheet("""
            QPushButton {
                background-color: #00BCD4;
                color: white;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #0097A7;
            }
        """)
        self.aruco_naming_btn.clicked.connect(self.open_aruco_naming_dialog)
        
        view_layout.addWidget(self.chk_yolo, 0, 0)
        view_layout.addWidget(self.chk_aruco, 0, 1)
        view_layout.addWidget(self.chk_distance, 0, 2)
        view_layout.addWidget(self.chk_tracking, 0, 3)
        view_layout.addWidget(self.mirror_btn, 1, 0)
        view_layout.addWidget(self.aruco_naming_btn, 1, 1)
        view_group.setLayout(view_layout)
        right_layout.addWidget(view_group)
        
        # Global speed control widget
        self.speed_control = SpeedControlWidget()
        self.speed_control.linear_speed_changed.connect(self.on_linear_speed_changed)
        self.speed_control.angular_speed_changed.connect(self.on_angular_speed_changed)
        right_layout.addWidget(self.speed_control)

        # Tab widget for different control modes
        self.tab_widget = QTabWidget()
        
        self.manual_control = ManualControlWidget()
        self.manual_control.velocity_changed.connect(self.send_velocity)
        self.manual_control.action_requested.connect(self.send_action)
        
        self.gesture_control = GestureControlWidget()
        self.gesture_control.camera_switched.connect(self.switch_gesture_camera)
        self.gesture_control.mirror_toggled.connect(self.toggle_webcam_mirror)
        
        self.tab_widget.addTab(self.manual_control, "Manual Control")
        self.tab_widget.addTab(self.gesture_control, "Gesture Mode")
        
        right_layout.addWidget(self.tab_widget)
        
        # Add stretch to push export widget to bottom
        right_layout.addStretch()
        
        # Export/Record widget at bottom
        export_group = QGroupBox("Export & Record")
        export_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #888;
                border: 1px solid #333;
                border-radius: 5px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        export_layout = QVBoxLayout()
        export_layout.setContentsMargins(4, 4, 4, 4)
        
        self.export_widget = ExportRecordWidget(
            self.get_drone_frame_with_overlay,
            self.get_current_telemetry
        )
        if self._cv2 is not None:
            self.export_widget.set_cv2(self._cv2)
        
        export_layout.addWidget(self.export_widget)
        export_group.setLayout(export_layout)
        right_layout.addWidget(export_group)
        
        right_panel.setLayout(right_layout)
        right_panel.setMaximumWidth(450)
        
        # Add to main layout
        main_layout.addLayout(left_panel, stretch=2)
        main_layout.addWidget(right_panel, stretch=1)
        
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Start webcam capture thread so laptop camera is available immediately
        try:
            self.webcam_thread = WebcamThread(
                self.signal_emitter, 
                self.ros_node, 
                device=0,
                cv2_module=self._cv2
            )
            self.webcam_thread.start()
        except Exception:
            self.webcam_thread = None
    
    def get_drone_frame_with_overlay(self):
        """Get current drone video frame with all overlays as QPixmap."""
        try:
            if (hasattr(self.drone_video_widget, '_last_pixmap') and 
                self.drone_video_widget._last_pixmap):
                # Return the overlayed version
                return self.drone_video_widget._draw_overlay_on(
                    self.drone_video_widget._last_pixmap
                )
            return None
        except Exception:
            return None
    
    def get_current_telemetry(self):
        """Get current telemetry data as dict."""
        try:
            if hasattr(self.ros_node, 'current_flight_data'):
                return self.ros_node.current_flight_data
            return None
        except Exception:
            return None
    
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
        """Update gesture status."""
        self.gesture_control.update_gesture_status(gesture)
    
    def update_gesture_annotated(self, cv_image):
        """Update the gesture video widget with skeleton annotated image."""
        try:
            # Always display skeleton on dedicated gesture widget
            self.gesture_video_widget.update_image(cv_image)
        except Exception:
            pass
    
    def send_velocity(self, vx, vy, vz, vw):
        """Send velocity command to drone."""
        if not self.gesture_control.gesture_mode:
            self.ros_node.send_velocity(vx, vy, vz, vw)
    
    def send_action(self, action):
        """Send action command to drone."""
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

        # Skip if mode hasn't changed (prevents spam)
        if hasattr(self, 'current_mode') and self.current_mode == mode:
            return

        # Track current mode for gesture/raw webcam display logic
        self.current_mode = mode

        if send_request:
            self.ros_node.set_mode(mode)
        
        # Update UI based on mode
        if mode == 'gesture':
            # Highlight gesture widget
            self.gesture_video_widget.setStyleSheet("border: 3px solid #E91E63;")
            self.drone_video_widget.setStyleSheet("border: 2px solid #2196F3;")
            # Update caption to show gesture source
            src = self.gesture_camera_source.capitalize()
            # Switch tab
            self.tab_widget.setCurrentWidget(self.gesture_control)
        elif mode == 'tracking':
            self.drone_video_widget.setStyleSheet("border: 3px solid #FF9800;")
            self.gesture_video_widget.setStyleSheet("border: 2px solid #2196F3;")
        else:
            self.drone_video_widget.setStyleSheet("border: 3px solid #2196F3;")
            self.gesture_video_widget.setStyleSheet("border: 2px solid #2196F3;")
            # Switch tab
            self.tab_widget.setCurrentWidget(self.manual_control)
            
        # Enable/disable manual control
        self.manual_control.setEnabled(mode == 'manual')
        
        # Set focus to manual control when in manual mode
        if mode == 'manual':
            self.manual_control.setFocus()
        
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
            self.ros_node.get_logger().warn(
                'Gesture camera switch service not available'
            )
    
    def _gesture_camera_callback(self, future):
        """Handle gesture camera switch service response."""
        try:
            result = future.result()
            if result.success:
                self.ros_node.get_logger().info(
                    f'Gesture camera: {result.message}'
                )
            else:
                self.ros_node.get_logger().warn(
                    f'Gesture camera switch failed: {result.message}'
                )
        except Exception as e:
            self.ros_node.get_logger().error(
                f'Gesture camera switch error: {e}'
            )
    
    def toggle_webcam_mirror(self, is_mirrored):
        """Toggle webcam mirror mode."""
        print(f"Webcam mirror: {'ON' if is_mirrored else 'OFF'}")
        
        if self.webcam_thread is not None:
            self.webcam_thread.set_mirror(is_mirrored)
    
    def toggle_drone_mirror(self, is_mirrored):
        """Toggle drone camera mirror mode."""
        print(f"Drone camera mirror: {'ON' if is_mirrored else 'OFF'}")
        self.ros_node.set_drone_camera_mirror(is_mirrored)

    def on_reconnect_clicked(self):
        """Handler for reconnect button click."""
        self.telemetry_widget.reconnect_btn.setEnabled(False)
        try:
            self.ros_node.reconnect_drone()
        except Exception:
            pass
        # re-enable after short timeout in case callback isn't wired to GUI
        QTimer.singleShot(3000, lambda: self.telemetry_widget.reconnect_btn.setEnabled(True))
    
    def on_linear_speed_changed(self, speed: float):
        """Handle global linear speed change."""
        # Update manual control widget speed factor
        self.manual_control.speed_factor = speed
        self.manual_control.speed_label.setText(f"Speed: {int(speed * 100)}%")
        self.manual_control.speed_slider.setValue(int(speed * 10))
        print(f"Linear speed set to: {speed:.1f} m/s")
    
    def on_angular_speed_changed(self, speed: float):
        """Handle global angular speed change."""
        # Store for use by other modes
        self._angular_speed = speed
        print(f"Angular speed set to: {speed:.1f} rad/s")
    
    def get_linear_speed(self) -> float:
        """Get current global linear speed."""
        return self.speed_control.linear_speed
    
    def get_angular_speed(self) -> float:
        """Get current global angular speed."""
        return self.speed_control.angular_speed
    
    def open_aruco_naming_dialog(self):
        """Open dialog to name ArUco markers."""
        dialog = ArucoNamingDialog(self.aruco_marker_names, self)
        dialog.names_updated.connect(self.update_aruco_names)
        dialog.exec_()
    
    def update_aruco_names(self, names: dict):
        """Update ArUco marker names from dialog.
        
        Args:
            names: Dictionary mapping marker IDs to names {id: name}
        """
        self.aruco_marker_names = names
        
        # Update video widget with new names
        self.drone_video_widget.aruco_marker_names = names
        
        # Update export widget with new names
        if hasattr(self.export_widget, 'set_aruco_names'):
            self.export_widget.set_aruco_names(names)
        
        print(f"ArUco names updated: {names}")

    def closeEvent(self, event):
        """Handle window close."""
        print("Shutting down...")
        
        # Cleanup export widget (stop recording if active)
        try:
            if hasattr(self, 'export_widget'):
                self.export_widget.cleanup()
        except Exception:
            pass
        
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
