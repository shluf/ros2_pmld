"""Telemetry display widget for drone status and motion data."""

import qtawesome as qta

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QFrame, QGridLayout, QProgressBar, QPushButton
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


class TelemetryWidget(QWidget):
    """Widget untuk menampilkan telemetry data dengan layout 3 kolom.
    
    Displays:
    - Column 1: System status, battery, and control mode
    - Column 2: Position and sensor data (attitude, altitude, temp, flight time)
    - Column 3: Motion loop (cmd_vel vs drone velocity)
    """
    
    reconnect_clicked = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.flight_time_seconds = 0
        self.motion_rows = {}
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the widget UI with 3-column layout."""
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(8, 6, 8, 6)
        main_layout.setSpacing(8)
        
        # Column 1: System & State
        self._setup_system_column(main_layout)
        
        # Column 2: Position & Sensors
        self._setup_sensors_column(main_layout)
        
        # Column 3: Motion Loop
        self._setup_motion_column(main_layout)
        
        self.setLayout(main_layout)
        self.setFixedHeight(130)
        
        # Apply stylesheet
        self._apply_stylesheet()
    
    def _setup_system_column(self, main_layout):
        """Setup system & state column."""
        system_frame = QFrame()
        system_frame.setObjectName("systemFrame")
        system_layout = QVBoxLayout()
        system_layout.setContentsMargins(10, 6, 10, 6)
        system_layout.setSpacing(4)
        
        # Title
        system_title = QLabel("SYSTEM & STATE")
        system_title.setFont(QFont("Arial", 9, QFont.Bold))
        system_title.setAlignment(Qt.AlignCenter)
        system_layout.addWidget(system_title)
        
        # Status row with reconnect button
        status_row = QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(6)
        
        # Status indicator
        self.status_label = QLabel("OFFLINE")
        self.status_label.setFont(QFont("Arial", 11, QFont.Bold))
        self.status_label.setObjectName("statusLabel")
        self.status_label.setAlignment(Qt.AlignCenter)
        status_row.addWidget(self.status_label, 1)
        
        # Reconnect button
        self.reconnect_btn = QPushButton()
        self.reconnect_btn.setIcon(qta.icon('mdi.refresh', color='white'))
        self.reconnect_btn.setToolTip("Reconnect to drone")
        self.reconnect_btn.setFixedSize(24, 24)
        self.reconnect_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
            QPushButton:disabled {
                background-color: #555;
            }
        """)
        self.reconnect_btn.clicked.connect(self.reconnect_clicked.emit)
        status_row.addWidget(self.reconnect_btn)
        
        system_layout.addLayout(status_row)
        
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
        main_layout.addWidget(system_frame, 1)
    
    def _setup_sensors_column(self, main_layout):
        """Setup position & sensors column."""
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
        
        # Sensor grid
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
        main_layout.addWidget(sensors_frame, 2)
    
    def _setup_motion_column(self, main_layout):
        """Setup motion loop column."""
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
        
        header_arrow = QLabel("|")
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
            
            arrow_label = QLabel("|")
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
            
            self.motion_rows[key] = {
                'cmd': cmd_label,
                'vel': vel_label
            }
        
        motion_frame.setLayout(motion_layout)
        main_layout.addWidget(motion_frame, 2)
    
    def _apply_stylesheet(self):
        """Apply centralized stylesheet."""
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
    
    def _update_battery_style(self, battery: int):
        """Update battery bar color based on level."""
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
    
    def _format_flight_time(self, seconds: int) -> str:
        """Format flight time as HH:MM:SS."""
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    
    def _update_motion_value(self, label: QLabel, value: float, is_cmd: bool = True):
        """Update motion label with color coding based on value."""
        if is_cmd:
            label.setText(f"{value:.2f}")
            if abs(value) > 0.05:
                label.setStyleSheet("color: #E91E63; font-weight: bold; border: none;")
            else:
                label.setStyleSheet("color: #666; border: none;")
        else:
            label.setText(f"{int(value)}")
            if abs(value) > 1:
                label.setStyleSheet("color: #4CAF50; font-weight: bold; border: none;")
            else:
                label.setStyleSheet("color: #666; border: none;")
    
    def update_telemetry(self, flight_data: dict):
        """Update telemetry display with new flight data."""
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
            
            if 'lx' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lx']['vel'], vx, is_cmd=False)
            if 'ly' in self.motion_rows:
                self._update_motion_value(self.motion_rows['ly']['vel'], vy, is_cmd=False)
            if 'lz' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lz']['vel'], vz, is_cmd=False)
            
            # Update status
            if battery > 0:
                self.status_label.setText("ONLINE")
                self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold; border: none;")
            else:
                self.status_label.setText("OFFLINE")
                self.status_label.setStyleSheet("color: #F44336; font-weight: bold; border: none;")
                
        except Exception as e:
            print(f"Error updating telemetry: {e}")
    
    def update_cmd_vel(self, twist_msg):
        """Update cmd_vel display in motion loop."""
        try:
            lx = twist_msg.linear.x
            ly = twist_msg.linear.y
            lz = twist_msg.linear.z
            az = twist_msg.angular.z
            
            if 'lx' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lx']['cmd'], lx, is_cmd=True)
            if 'ly' in self.motion_rows:
                self._update_motion_value(self.motion_rows['ly']['cmd'], ly, is_cmd=True)
            if 'lz' in self.motion_rows:
                self._update_motion_value(self.motion_rows['lz']['cmd'], lz, is_cmd=True)
            if 'az' in self.motion_rows:
                self._update_motion_value(self.motion_rows['az']['cmd'], az, is_cmd=True)
                deg_per_sec = az * 57.3
                self._update_motion_value(self.motion_rows['az']['vel'], deg_per_sec, is_cmd=False)
                
        except Exception as e:
            print(f"Error updating cmd_vel: {e}")
    
    def update_mode(self, mode: str):
        """Update mode display."""
        self.mode_label.setText(mode.upper())
        
        colors = {
            'manual': '#2196F3',
            'joystick': '#9C27B0',
            'gesture': '#E91E63',
            'tracking': '#FF9800'
        }
        color = colors.get(mode.lower(), '#2196F3')
        self.mode_label.setStyleSheet(f"color: {color}; font-weight: bold; border: none;")
