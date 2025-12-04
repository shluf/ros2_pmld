"""Manual control widget for keyboard-based drone control."""

import qtawesome as qta

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel,
    QPushButton, QGroupBox, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont


class ManualControlWidget(QWidget):
    """Widget untuk kontrol manual drone via GUI dan keyboard.
    
    Features:
    - WASD + Q/E keyboard control
    - Speed slider
    - Action buttons (takeoff, land, hover, emergency)
    - Flip buttons
    """
    
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
        """Setup the widget UI."""
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Manual Control")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Action buttons
        self._setup_action_buttons(layout)
        
        # Keyboard status
        self._setup_keyboard_status(layout)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _setup_action_buttons(self, layout):
        """Setup action and flip buttons."""
        action_group = QGroupBox("Quick Actions")
        action_layout = QGridLayout()
        
        self.takeoff_btn = self._create_action_button("Takeoff (T)", "#4CAF50", "takeoff")
        self.land_btn = self._create_action_button("Land (L)", "#2196F3", "land")
        self.emergency_btn = self._create_action_button("EMERGENCY", "#F44336", "emergency")
        self.hover_btn = self._create_action_button("Hover (H)", "#FF9800", "hover")
        
        # Flip buttons
        self.flip_f_btn = self._create_action_button("Flip ↑", "#9C27B0", "flip_f")
        self.flip_b_btn = self._create_action_button("Flip ↓", "#9C27B0", "flip_b")
        self.flip_l_btn = self._create_action_button("Flip ←", "#9C27B0", "flip_l")
        self.flip_r_btn = self._create_action_button("Flip →", "#9C27B0", "flip_r")
        
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
    
    def _setup_keyboard_status(self, layout):
        """Setup keyboard status display."""
        keyboard_group = QGroupBox("Keyboard Controls")
        keyboard_layout = QHBoxLayout()
        
        # Keyboard icon
        self.keyboard_icon = QLabel()
        self.keyboard_icon.setPixmap(qta.icon('mdi.keyboard', color='#666').pixmap(20, 20))
        self.keyboard_icon.setFixedSize(24, 24)
        
        self.keyboard_status = QLabel("Click panel to enable")
        self.keyboard_status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.keyboard_status.setStyleSheet("color: #666; padding: 5px; font-size: 11px;")
        
        keyboard_layout.addWidget(self.keyboard_icon)
        keyboard_layout.addWidget(self.keyboard_status)
        keyboard_layout.addStretch()
        keyboard_group.setLayout(keyboard_layout)
        layout.addWidget(keyboard_group)
    
    def _create_action_button(self, text: str, color: str, action: str) -> QPushButton:
        """Create a styled action button."""
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
    
    def stop_movement(self):
        """Stop all movement."""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vw = 0.0
    
    def send_velocity(self):
        """Send velocity command if there's any movement."""
        if self.vx != 0 or self.vy != 0 or self.vz != 0 or self.vw != 0:
            self.velocity_changed.emit(self.vx, self.vy, self.vz, self.vw)
    
    def keyPressEvent(self, event):
        """Handle keyboard press for drone control."""
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
        """Handle keyboard release."""
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
        """Update status when keyboard focus gained."""
        super().focusInEvent(event)
        self.keyboard_icon.setPixmap(qta.icon('mdi.keyboard', color='#4CAF50').pixmap(20, 20))
        self.keyboard_status.setText("WASD + Q/E + Space/Shift")
        self.keyboard_status.setStyleSheet("color: #4CAF50; padding: 5px; font-weight: bold;")
    
    def focusOutEvent(self, event):
        """Update status when keyboard focus lost."""
        super().focusOutEvent(event)
        self.keys_pressed.clear()
        self.stop_movement()
        self.keyboard_icon.setPixmap(qta.icon('mdi.keyboard', color='#666').pixmap(20, 20))
        self.keyboard_status.setText("Click here to enable")
        self.keyboard_status.setStyleSheet("color: #666; padding: 5px;")
