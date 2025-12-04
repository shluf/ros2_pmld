"""Global speed control widget for all control modes."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


class SpeedControlWidget(QWidget):
    """Global widget untuk mengatur kecepatan linear dan angular drone.
    
    Widget ini dapat diakses oleh semua mode kontrol (Manual, Gesture, Tracking).
    
    Features:
    - Linear speed slider (0.1 - 1.0 m/s)
    - Angular speed slider (0.1 - 1.0 rad/s)
    - Signals emitted when speeds change
    """
    
    linear_speed_changed = pyqtSignal(float)
    angular_speed_changed = pyqtSignal(float)
    
    def __init__(self):
        super().__init__()
        
        # Default speed values
        self._linear_speed = 0.5
        self._angular_speed = 0.5
        
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the widget UI."""
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        
        # Speed Control Group
        speed_group = QGroupBox("Speed Control")
        speed_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #FF9800;
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
        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(8)
        
        # Linear speed
        linear_layout = QHBoxLayout()
        linear_label = QLabel("Linear:")
        linear_label.setFixedWidth(55)
        linear_label.setStyleSheet("color: #4CAF50;")
        
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setMinimum(1)
        self.linear_slider.setMaximum(10)
        self.linear_slider.setValue(5)
        self.linear_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px;
                background: #333;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4CAF50;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QSlider::sub-page:horizontal {
                background: #4CAF50;
                border-radius: 3px;
            }
        """)
        self.linear_slider.valueChanged.connect(self._on_linear_changed)
        
        self.linear_value_label = QLabel("0.5 m/s")
        self.linear_value_label.setFixedWidth(55)
        self.linear_value_label.setAlignment(Qt.AlignRight)
        self.linear_value_label.setStyleSheet("color: #4CAF50;")
        
        linear_layout.addWidget(linear_label)
        linear_layout.addWidget(self.linear_slider)
        linear_layout.addWidget(self.linear_value_label)
        speed_layout.addLayout(linear_layout)
        
        # Angular speed
        angular_layout = QHBoxLayout()
        angular_label = QLabel("Angular:")
        angular_label.setFixedWidth(55)
        angular_label.setStyleSheet("color: #2196F3;")
        
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setMinimum(1)
        self.angular_slider.setMaximum(10)
        self.angular_slider.setValue(5)
        self.angular_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px;
                background: #333;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #2196F3;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QSlider::sub-page:horizontal {
                background: #2196F3;
                border-radius: 3px;
            }
        """)
        self.angular_slider.valueChanged.connect(self._on_angular_changed)
        
        self.angular_value_label = QLabel("0.5 rad/s")
        self.angular_value_label.setFixedWidth(55)
        self.angular_value_label.setAlignment(Qt.AlignRight)
        self.angular_value_label.setStyleSheet("color: #2196F3;")
        
        angular_layout.addWidget(angular_label)
        angular_layout.addWidget(self.angular_slider)
        angular_layout.addWidget(self.angular_value_label)
        speed_layout.addLayout(angular_layout)
        
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        self.setLayout(layout)
    
    def _on_linear_changed(self, value: int):
        """Handle linear speed slider change."""
        self._linear_speed = value / 10.0
        self.linear_value_label.setText(f"{self._linear_speed:.1f} m/s")
        self.linear_speed_changed.emit(self._linear_speed)
    
    def _on_angular_changed(self, value: int):
        """Handle angular speed slider change."""
        self._angular_speed = value / 10.0
        self.angular_value_label.setText(f"{self._angular_speed:.1f} rad/s")
        self.angular_speed_changed.emit(self._angular_speed)
    
    @property
    def linear_speed(self) -> float:
        """Get current linear speed."""
        return self._linear_speed
    
    @property
    def angular_speed(self) -> float:
        """Get current angular speed."""
        return self._angular_speed
    
    def set_linear_speed(self, value: float):
        """Set linear speed programmatically."""
        slider_value = int(value * 10)
        self.linear_slider.setValue(slider_value)
    
    def set_angular_speed(self, value: float):
        """Set angular speed programmatically."""
        slider_value = int(value * 10)
        self.angular_slider.setValue(slider_value)
