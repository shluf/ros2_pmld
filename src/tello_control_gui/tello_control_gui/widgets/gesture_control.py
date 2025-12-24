"""Gesture control widget for gesture-based drone control."""

import json
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QGroupBox, QScrollArea, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


# Gesture mapping based on gesture_mapping.yaml
HAND_SIGNS = {
    0: ("move_forward", "👆", "Forward"),
    1: ("hover", "✋", "Stop/Hover"),
    2: ("move_up", "☝️", "Up"),
    3: ("land", "✊", "Land"),
    4: ("move_down", "👇", "Down"),
    5: ("move_backward", "🤚", "Backward"),
    6: ("move_left", "👈", "Left"),
    7: ("move_right", "👉", "Right"),
}

FINGER_GESTURES = {
    0: ("hover", "✋", "Stop"),
    1: ("flip", "🔄", "Flip"),
    2: ("rotate_ccw", "↺", "Rotate CCW"),
    3: ("move", "🖐️", "Move"),
}

class GestureControlWidget(QWidget):
    """Widget untuk gesture control status dan konfigurasi.
    
    Features:
    - Camera source toggle (webcam/drone)
    - Mirror toggle for webcam
    - Gesture status display with current action
    - Complete gesture mapping reference
    """
    
    camera_switched = pyqtSignal(str)  # 'webcam' or 'drone'
    mirror_toggled = pyqtSignal(bool)  # True = mirrored
    
    def __init__(self):
        super().__init__()
        self.using_webcam = False  # Default to webcam for gesture
        self.gesture_mode = False  # Keep for compatibility
        self.is_mirrored = True  # Default webcam mirrored
        self.setup_ui()
    
    def setup_ui(self):
        """Setup the widget UI."""
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Title
        title = QLabel("Gesture Mode")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Camera source toggle
        self._setup_camera_toggle(layout)
        
        # Gesture status
        self._setup_gesture_status(layout)
        
        # Gesture mapping reference (scrollable)
        self._setup_gesture_mapping(layout)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _setup_camera_toggle(self, layout):
        """Setup camera source toggle buttons with mirror option."""
        camera_group = QGroupBox("Camera Source")
        camera_layout = QVBoxLayout()
        
        # Camera switch button
        self.camera_btn = QPushButton("Switch to Drone Camera")
        self.camera_btn.setMinimumHeight(50)
        self.camera_btn.setCheckable(True)
        self.camera_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 13px;
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
        camera_layout.addWidget(self.camera_btn)
        
        # Mirror button (only for webcam)
        self.mirror_btn = QPushButton("Mirror: ON")
        self.mirror_btn.setMinimumHeight(40)
        self.mirror_btn.setCheckable(True)
        self.mirror_btn.setChecked(True)  # Default mirrored
        self.mirror_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:checked {
                background-color: #4CAF50;
            }
            QPushButton:disabled {
                background-color: #666;
                color: #999;
            }
        """)
        self.mirror_btn.clicked.connect(self.toggle_mirror)
        camera_layout.addWidget(self.mirror_btn)
        
        # Camera status
        self.camera_status = QLabel("Using Webcam for Gesture")
        self.camera_status.setFont(QFont("Arial", 11))
        self.camera_status.setAlignment(Qt.AlignCenter)
        self.camera_status.setStyleSheet("color: #9C27B0; padding: 8px;")
        camera_layout.addWidget(self.camera_status)
        
        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)
    
    def _setup_gesture_status(self, layout):
        """Setup gesture status display."""
        gesture_group = QGroupBox("Current Gesture")
        gesture_layout = QVBoxLayout()
        
        # Main gesture label
        self.gesture_label = QLabel("No gesture detected")
        self.gesture_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.gesture_label.setAlignment(Qt.AlignCenter)
        self.gesture_label.setMinimumHeight(50)
        self.gesture_label.setStyleSheet("""
            background-color: #37474F;
            padding: 15px;
            border-radius: 8px;
            color: #90A4AE;
        """)
        gesture_layout.addWidget(self.gesture_label)
        
        # Action label (what the gesture does)
        self.action_label = QLabel("")
        self.action_label.setFont(QFont("Arial", 10))
        self.action_label.setAlignment(Qt.AlignCenter)
        self.action_label.setStyleSheet("color: #888; padding: 5px;")
        gesture_layout.addWidget(self.action_label)
        
        gesture_group.setLayout(gesture_layout)
        layout.addWidget(gesture_group)
    
    def _setup_gesture_mapping(self, layout):
        """Setup gesture mapping reference with scrollable list."""
        mapping_group = QGroupBox("Gesture Reference")
        mapping_layout = QVBoxLayout()
        mapping_layout.setSpacing(4)
        
        # Create scrollable area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMaximumHeight(200)
        scroll.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollBar:vertical {
                width: 8px;
                background: #2a2a2a;
            }
            QScrollBar::handle:vertical {
                background: #555;
                border-radius: 4px;
            }
        """)
        
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout()
        scroll_layout.setSpacing(2)
        scroll_layout.setContentsMargins(4, 4, 4, 4)
        
        # Hand signs section
        
        for idx, (action, emoji, desc) in HAND_SIGNS.items():
            row = self._create_gesture_row(emoji, desc, action)
            scroll_layout.addWidget(row)
        
        scroll_layout.addStretch()
        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)
        
        mapping_layout.addWidget(scroll)
        mapping_group.setLayout(mapping_layout)
        layout.addWidget(mapping_group)
    
    def _create_gesture_row(self, emoji, desc, action):
        """Create a row showing gesture -> action mapping."""
        row = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(4, 2, 4, 2)
        row_layout.setSpacing(8)
        
        emoji_label = QLabel(emoji)
        emoji_label.setFont(QFont("Arial", 14))
        emoji_label.setFixedWidth(30)
        
        desc_label = QLabel(desc)
        desc_label.setFont(QFont("Arial", 9))
        desc_label.setStyleSheet("color: #ccc;")
        
        action_label = QLabel(f"→ {action}")
        action_label.setFont(QFont("Arial", 9))
        action_label.setStyleSheet("color: #888;")
        action_label.setAlignment(Qt.AlignRight)
        
        row_layout.addWidget(emoji_label)
        row_layout.addWidget(desc_label, stretch=1)
        row_layout.addWidget(action_label)
        
        row.setLayout(row_layout)
        return row
    
    def toggle_camera(self):
        """Toggle between webcam and drone camera for gesture recognition."""
        self.using_webcam = not self.camera_btn.isChecked()
        
        if self.using_webcam:
            self.camera_btn.setText("Switch to Drone Camera")
            self.camera_status.setText("Using Webcam for Gesture")
            self.camera_status.setStyleSheet("color: #9C27B0; padding: 8px;")
            self.mirror_btn.setEnabled(True)
            self.camera_switched.emit('webcam')
        else:
            self.camera_btn.setText("Switch to Webcam")
            self.camera_status.setText("Using Drone Camera for Gesture")
            self.camera_status.setStyleSheet(
                "color: #E91E63; padding: 8px; font-weight: bold;"
            )
            self.mirror_btn.setEnabled(False)
            self.camera_switched.emit('drone')
    
    def toggle_mirror(self):
        """Toggle webcam mirror mode."""
        self.is_mirrored = self.mirror_btn.isChecked()
        
        if self.is_mirrored:
            self.mirror_btn.setText("Mirror: ON")
        else:
            self.mirror_btn.setText("Mirror: OFF")
        
        self.mirror_toggled.emit(self.is_mirrored)
    
    def update_gesture_status(self, gesture_data: str):
        """Update gesture status display from JSON gesture data.
        
        Args:
            gesture_data: JSON string with format:
                {
                    "hand_sign": int or None,
                    "finger_gesture": int or None,
                    "hand_position": dict or None,
                    "camera_source": str,
                    "timestamp": int
                }
        """
        try:
            # Parse JSON data
            data = json.loads(gesture_data)
            
            hand_sign_idx = data.get('hand_sign')
            finger_gesture_idx = data.get('finger_gesture')
            
            # Determine which gesture to display
            gesture_name = None
            emoji = "🤚"
            desc = ""
            
            # Priority: hand_sign > finger_gesture
            if hand_sign_idx is not None and hand_sign_idx in HAND_SIGNS:
                action, emoji, desc = HAND_SIGNS[hand_sign_idx]
                gesture_name = action
            elif finger_gesture_idx is not None and finger_gesture_idx in FINGER_GESTURES:
                action, emoji, desc = FINGER_GESTURES[finger_gesture_idx]
                gesture_name = action
            
            if gesture_name:
                self.gesture_label.setText(f"{emoji} {desc}")
                self.gesture_label.setStyleSheet("""
                    background-color: #4CAF50;
                    padding: 15px;
                    border-radius: 8px;
                    color: white;
                    font-weight: bold;
                """)
                self.action_label.setText(f"Action: {gesture_name}")
                self.action_label.setStyleSheet("color: #4CAF50; padding: 5px;")
            else:
                self.gesture_label.setText("No gesture detected")
                self.gesture_label.setStyleSheet("""
                    background-color: #37474F;
                    padding: 15px;
                    border-radius: 8px;
                    color: #90A4AE;
                """)
                self.action_label.setText("")
                
        except json.JSONDecodeError:
            # Fallback: treat as plain string (legacy format)
            self._update_gesture_legacy(gesture_data)
        except Exception as e:
            # Any other error, show raw data
            self.gesture_label.setText("Error parsing gesture")
            self.action_label.setText(str(e)[:50])
    
    def _update_gesture_legacy(self, gesture: str):
        """Legacy update for plain string gesture names."""
        if gesture:
            # Find the action description
            action_desc = ""
            emoji = "🤚"
            
            # Check hand signs by action name
            for idx, (action, e, desc) in HAND_SIGNS.items():
                if action == gesture or gesture.lower() == action.lower():
                    action_desc = desc
                    emoji = e
                    break
            
            # Check finger gestures if not found
            if not action_desc:
                for idx, (action, e, desc) in FINGER_GESTURES.items():
                    if action == gesture or gesture.lower() == action.lower():
                        action_desc = desc
                        emoji = e
                        break
            
            self.gesture_label.setText(f"{emoji} {gesture}")
            self.gesture_label.setStyleSheet("""
                background-color: #4CAF50;
                padding: 15px;
                border-radius: 8px;
                color: white;
                font-weight: bold;
            """)
            
            if action_desc:
                self.action_label.setText(f"Action: {action_desc}")
                self.action_label.setStyleSheet("color: #4CAF50; padding: 5px;")
            else:
                self.action_label.setText(f"Action: {gesture}")
                self.action_label.setStyleSheet("color: #888; padding: 5px;")
        else:
            self.gesture_label.setText("No gesture detected")
            self.gesture_label.setStyleSheet("""
                background-color: #37474F;
                padding: 15px;
                border-radius: 8px;
                color: #90A4AE;
            """)
            self.action_label.setText("")
