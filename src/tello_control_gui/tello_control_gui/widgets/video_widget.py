"""Video display widget with annotation overlay support."""

import json
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer, QRect
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QColor, QPen

# Will be set by main.py
cv2 = None


def set_cv2_module(cv2_module):
    """Set cv2 module after import."""
    global cv2
    cv2 = cv2_module


class VideoWidget(QWidget):
    """Widget untuk menampilkan video stream dengan annotation overlay.
    
    Features:
    - Video display with FPS counter
    - YOLO detection box overlay
    - ArUco marker info overlay
    - Distance measurement overlay
    - Tracking visualization overlay
    - Caption display
    """
    
    def __init__(self):
        super().__init__()
        self.setup_ui()
        
        # Delay initialization untuk memastikan widget sudah ter-layout
        QTimer.singleShot(100, self._initialize_display)
        
    def setup_ui(self):
        """Setup the widget UI."""
        layout = QVBoxLayout()
        
        # Video display label
        self.video_label = QLabel()
        # self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet(
            "background-color: black; border: 2px solid #4CAF50; border-radius: 10px;"
        )
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setText("Waiting for video stream...")
        
        # Caption label (overlayed on top of video_label)
        self.caption_label = QLabel(self.video_label)
        self.caption_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.caption_label.setStyleSheet(
            "background-color: #4CAF50; color: white; "
            "margin: 4px; border-radius: 4px;"
        )
        self.caption_label.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.caption_label.setText("")
        self.caption_label.move(5, 5)

        layout.addWidget(self.video_label)
        self.setLayout(layout)
        
        # Stats
        self.frame_count = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(1000)
        self.current_fps = 0
        
        # Overlay state
        self.overlay_telemetry = {}
        self._last_pixmap = None
        self.overlay_enabled = True
        self.has_frame = False
        self._no_frame_count = 0
        self.show_overlay_always = False
        self.show_fps_overlay = True
        
        # Data storage for annotations
        self.detections = []
        self.aruco_poses = []
        self.aruco_ids = []  # List of detected marker IDs
        self.distances = []
        self.tracking_cmd = None
        self.gesture_data = {}
        self.caption_text = ""
        
        # View flags
        self.show_yolo = True
        self.show_aruco = True
        self.show_tracking = True
        self.show_gesture = True
        self.show_distance = True
        self.is_webcam = False  # Flag to identify if this widget shows webcam/gesture
        
        # ArUco marker names dictionary {marker_id: name}
        self.aruco_marker_names = {}

        # Initialize overlay
        try:
            self.update_overlay_telemetry({})
        except Exception:
            pass

    # ==================== Data Update Methods ====================

    def update_detections(self, detections):
        """Update YOLO detection data."""
        self.detections = detections
    
    def update_aruco(self, poses):
        """Update ArUco marker poses."""
        self.aruco_poses = poses
    
    def update_aruco_ids(self, ids):
        """Update ArUco marker IDs."""
        self.aruco_ids = ids

    def update_distances(self, distances):
        """Update distance measurements."""
        self.distances = distances

    def update_tracking(self, cmd):
        """Update tracking command data."""
        self.tracking_cmd = cmd

    def update_gesture_data(self, data_str):
        """Update gesture data from JSON string."""
        try:
            self.gesture_data = json.loads(data_str)
        except:
            pass
    
    def update_image(self, cv_image):
        """Update displayed image with FPS and annotations overlay."""
        try:
            self.frame_count += 1
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

            # Store last pixmap and draw overlay
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
            overlayed = self._draw_overlay_on(blank)
            self.video_label.setPixmap(overlayed)
            return

        # Refresh overlay with updated FPS
        if self._last_pixmap is not None:
            overlayed = self._draw_overlay_on(self._last_pixmap)
            self.video_label.setPixmap(overlayed)

    def update_overlay_telemetry(self, flight_data: dict):
        """Update the telemetry dict used for on-screen overlay."""
        try:
            self.overlay_telemetry = flight_data or {}
            if self._last_pixmap is not None:
                overlayed = self._draw_overlay_on(self._last_pixmap)
                self.video_label.setPixmap(overlayed)
        except Exception as e:
            print(f"Error updating overlay telemetry: {e}")

    # ==================== Drawing Methods ====================

    def _draw_overlay_on(self, pixmap: QPixmap) -> QPixmap:
        """Return a copy of pixmap with FPS and annotations overlay drawn."""
        # Validate pixmap
        if pixmap is None or pixmap.isNull():
            return pixmap
        
        try:
            # Create a deep copy using copy constructor (preserves content)
            p = QPixmap(pixmap)
            
            painter = QPainter()
            if not painter.begin(p):
                return pixmap
            
            try:
                painter.setRenderHint(QPainter.Antialiasing)
                painter.setRenderHint(QPainter.TextAntialiasing)

                # Draw annotations based on source type
                if not self.is_webcam:
                    if self.show_yolo:
                        self._draw_yolo(painter, p.width(), p.height())
                    if self.show_aruco:
                        self._draw_aruco(painter, p.width(), p.height())
                    if self.show_distance:
                        self._draw_distance(painter, p.width(), p.height())
                    if self.show_tracking:
                        self._draw_tracking(painter, p.width(), p.height())

                # Draw FPS overlay
                if self.show_fps_overlay:
                    self._draw_fps(painter)
                
                # Draw caption overlay
                if hasattr(self, 'caption_text') and self.caption_text:
                    self._draw_caption(painter, p.width())

            finally:
                # Always end painter to prevent crash
                painter.end()
            
            return p
        except Exception as e:
            print(f"Error drawing overlay: {e}")
            return pixmap

    def _draw_fps(self, painter):
        """Draw FPS counter on top-left corner."""
        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        fps_text = f"FPS: {self.current_fps}"
        
        x, y = 10, 20
        bg_rect = QRect(5, 5, 70, 22)
        painter.fillRect(bg_rect, QColor(0, 0, 0, 200))
        
        if self.current_fps > 0:
            painter.setPen(QColor(0, 255, 0))  # Green when active
        else:
            painter.setPen(QColor(255, 165, 0))  # Orange when no frames
        painter.drawText(x, y, fps_text)

    def _draw_caption(self, painter, width):
        """Draw caption text at top center."""
        font = QFont("Arial", 11, QFont.Bold)
        painter.setFont(font)
        fm = painter.fontMetrics()
        text_width = fm.horizontalAdvance(self.caption_text)
        x = (width - text_width) // 2
        
        bg_rect = QRect(x - 5, 5, text_width + 10, 22)
        painter.fillRect(bg_rect, QColor(0, 0, 0, 180))
        
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x, 20, self.caption_text)

    def _draw_yolo(self, painter, widget_w, widget_h):
        """Draw YOLO detection boxes with proper scaling."""
        orig_w, orig_h = 960, 720
        scale_x = widget_w / orig_w
        scale_y = widget_h / orig_h
        
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        fm = painter.fontMetrics()
        
        status_x = widget_w - 150
        status_y = 35
        
        if not self.detections:
            status_text = "YOLO: No objects"
            painter.fillRect(
                status_x - 4, status_y - 14, 
                fm.horizontalAdvance(status_text) + 8, 20, 
                QColor(0, 0, 0, 180)
            )
            painter.setPen(QColor(128, 128, 128))
            painter.drawText(status_x, status_y, status_text)
            return
        
        status_text = f"YOLO: {len(self.detections)} objects"
        painter.fillRect(
            status_x - 4, status_y - 14,
            fm.horizontalAdvance(status_text) + 8, 20,
            QColor(0, 0, 0, 180)
        )
        painter.setPen(QColor(0, 255, 0))
        painter.drawText(status_x, status_y, status_text)
        
        # Draw detection boxes
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.setBrush(Qt.NoBrush)
        
        for d in self.detections:
            x = int(d.x * scale_x)
            y = int(d.y * scale_y)
            w = int(d.width * scale_x)
            h = int(d.height * scale_y)
            
            painter.drawRect(x, y, w, h)
            
            label = f"{d.class_name} {d.confidence:.2f}"
            text_w = fm.horizontalAdvance(label)
            painter.fillRect(x, y - 18, text_w + 6, 18, QColor(0, 255, 0, 180))
            
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(x + 3, y - 5, label)
            painter.setPen(QPen(QColor(0, 255, 0), 2))

    def _draw_aruco(self, painter, widget_w, widget_h):
        """Draw ArUco marker info with custom names."""
        painter.setPen(QPen(QColor(0, 255, 255), 2))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        
        x = 10
        y = widget_h - 60
        fm = painter.fontMetrics()
        
        if not self.aruco_poses:
            info_text = "ArUco: No markers"
            text_w = fm.horizontalAdvance(info_text)
            painter.fillRect(x - 2, y - 14, text_w + 8, 20, QColor(0, 0, 0, 180))
            painter.setPen(QColor(128, 128, 128))
            painter.drawText(x, y, info_text)
            return
        
        info_text = f"ArUco: {len(self.aruco_poses)} markers"
        text_w = fm.horizontalAdvance(info_text)
        painter.fillRect(x - 2, y - 14, text_w + 8, 20, QColor(0, 0, 0, 180))
        
        painter.setPen(QColor(0, 255, 255))
        painter.drawText(x, y, info_text)
        
        for i, pose in enumerate(self.aruco_poses):
            dist = pose.position.z
            
            # Get actual marker ID from aruco_ids list
            marker_id = self.aruco_ids[i] if i < len(self.aruco_ids) else i
            
            # Get custom name or use default
            if marker_id in self.aruco_marker_names:
                marker_name = self.aruco_marker_names[marker_id]
                marker_info = f"  #{marker_id} '{marker_name}': {dist:.2f}m"
            else:
                marker_info = f"  ID{marker_id}: {dist:.2f}m"
            
            y += 18
            painter.fillRect(
                x - 2, y - 14,
                fm.horizontalAdvance(marker_info) + 8, 20,
                QColor(0, 0, 0, 180)
            )
            painter.drawText(x, y, marker_info)

    def _draw_distance(self, painter, widget_w, widget_h):
        """Draw distance measurements between objects and markers."""
        if not self.distances:
            return
        
        orig_w, orig_h = 960, 720
        scale_x = widget_w / orig_w
        scale_y = widget_h / orig_h
        
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        
        for dist_obj in self.distances:
            obj_x = int(dist_obj.object_center_x * scale_x)
            obj_y = int(dist_obj.object_center_y * scale_y)
            marker_x = int(dist_obj.marker_center_x * scale_x)
            marker_y = int(dist_obj.marker_center_y * scale_y)
            
            # Draw line
            painter.setPen(QPen(QColor(255, 255, 0), 2, Qt.DashLine))
            painter.drawLine(obj_x, obj_y, marker_x, marker_y)
            
            # Draw distance label
            mid_x = (obj_x + marker_x) // 2
            mid_y = (obj_y + marker_y) // 2
            
            dist_cm = dist_obj.distance_meters * 100
            dist_text = f"{dist_cm:.1f}cm"
            
            fm = painter.fontMetrics()
            text_w = fm.horizontalAdvance(dist_text)
            painter.fillRect(mid_x - 2, mid_y - 12, text_w + 6, 16, QColor(255, 255, 0, 200))
            
            painter.setPen(QColor(0, 0, 0))
            painter.drawText(mid_x + 2, mid_y, dist_text)
            
            # Draw points
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(obj_x - 4, obj_y - 4, 8, 8)
            
            painter.setPen(QPen(QColor(0, 255, 0), 2))
            painter.setBrush(QColor(0, 255, 0))
            painter.drawEllipse(marker_x - 4, marker_y - 4, 8, 8)
            
            painter.setBrush(Qt.NoBrush)

    def _draw_tracking(self, painter, w, h):
        """Draw tracking visualization."""
        if not self.tracking_cmd:
            return
        
        cx, cy = w // 2, h // 2
        scale = 100.0
        
        # Correct mapping:
        # linear.y = left(+)/right(-) in drone frame -> screen horizontal (dx)
        # linear.z = up(+)/down(-) in drone frame -> screen vertical (dy), negated for screen coords
        dx = -int(self.tracking_cmd['linear']['y'] * scale)   # Left-right
        dy = -int(self.tracking_cmd['linear']['z'] * scale)   # Up-down (fixed: was linear.x)
        
        if abs(dx) > 5 or abs(dy) > 5:
            painter.setPen(QPen(QColor(255, 100, 100), 3))
            painter.drawLine(cx, cy, cx + dx, cy + dy)
            
            painter.setBrush(QColor(255, 100, 100))
            painter.drawEllipse(cx + dx - 5, cy + dy - 5, 10, 10)
        
        # Draw forward/backward indicator (linear.x) as circle size
        fwd = self.tracking_cmd['linear']['x']
        if abs(fwd) > 0.05:
            indicator_radius = int(25 + fwd * 40)
            indicator_radius = max(10, min(50, indicator_radius))
            painter.setPen(QPen(QColor(100, 255, 100), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(cx - indicator_radius, cy - indicator_radius,
                              indicator_radius * 2, indicator_radius * 2)
        
        # Draw center crosshair
        painter.setPen(QPen(QColor(255, 100, 100), 2))
        painter.drawLine(cx - 15, cy, cx + 15, cy)
        painter.drawLine(cx, cy - 15, cx, cy + 15)
        
        # Draw yaw arc
        az = self.tracking_cmd['angular']['z']
        if abs(az) > 0.1:
            painter.setPen(QPen(QColor(255, 255, 0), 2))
            arc_radius = 40
            start_angle = 0 if az > 0 else 180 * 16
            span_angle = int(abs(az) * 16 * 90)
            painter.drawArc(
                cx - arc_radius, cy - arc_radius,
                arc_radius * 2, arc_radius * 2,
                start_angle, span_angle
            )
        
        # Draw tracking info with all axes
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        info_text = f"FB={self.tracking_cmd['linear']['x']:.2f} LR={self.tracking_cmd['linear']['y']:.2f} UD={self.tracking_cmd['linear']['z']:.2f}"
        fm = painter.fontMetrics()
        text_w = fm.horizontalAdvance(info_text) + 10
        painter.fillRect(w - text_w - 10, 55, text_w, 20, QColor(0, 0, 0, 180))
        painter.setPen(QColor(255, 100, 100))
        painter.drawText(w - text_w - 5, 70, info_text)

    def _draw_gesture(self, painter):
        """Draw gesture overlay (not used when is_webcam=True)."""
        if not self.gesture_data:
            return
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
        """Create a black pixmap for when no video frames are available."""
        try:
            size = self.video_label.size()
            if size.width() <= 0 or size.height() <= 0:
                w, h = 640, 480
            else:
                w, h = size.width(), size.height()
            
            p = QPixmap(w, h)
            p.fill(QColor(0, 0, 0))
            
            # Draw waiting text
            painter = QPainter(p)
            painter.setPen(QColor(100, 100, 100))
            painter.setFont(QFont("Arial", 12))
            text = "Waiting for video stream..."
            fm = painter.fontMetrics()
            text_x = (w - fm.horizontalAdvance(text)) // 2
            text_y = h // 2
            painter.drawText(text_x, text_y, text)
            painter.end()
            
            return p
        except Exception as e:
            print(f"Error creating blank pixmap: {e}")
            return QPixmap(640, 480)

    def _initialize_display(self):
        """Initialize display after widget is properly laid out."""
        try:
            blank = self._make_blank_pixmap()
            if blank and not blank.isNull():
                self._last_pixmap = blank
                overlayed = self._draw_overlay_on(blank)
                self.video_label.setPixmap(overlayed)
            
            if hasattr(self, 'caption_label') and self.caption_label:
                self.caption_label.resize(self.video_label.width()-10, 36)
        except Exception as e:
            print(f"Error initializing display: {e}")
    
    def resizeEvent(self, event):
        """Handle resize event."""
        super().resizeEvent(event)
        try:
            if hasattr(self, 'caption_label') and self.caption_label:
                self.caption_label.resize(self.video_label.width()-10, 36)
            
            if not self.has_frame and self._last_pixmap is not None:
                blank = self._make_blank_pixmap()
                if blank and not blank.isNull():
                    self._last_pixmap = blank
                    overlayed = self._draw_overlay_on(blank)
                    self.video_label.setPixmap(overlayed)
        except Exception:
            pass

    # ==================== Settings Methods ====================

    def set_overlay_enabled(self, enabled: bool):
        """Enable or disable telemetry overlay drawing."""
        self.overlay_enabled = bool(enabled)

    def set_show_overlay_always(self, enabled: bool):
        """When True, overlay is drawn even if no video frames."""
        self.show_overlay_always = bool(enabled)

    def set_caption(self, text: str):
        """Set the caption label shown over the video."""
        try:
            self.caption_text = text
            self.caption_label.setText(text)
            self.caption_label.resize(self.video_label.width()-10, 36)
        except Exception:
            pass
