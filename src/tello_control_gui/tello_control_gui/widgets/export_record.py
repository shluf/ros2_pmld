"""Export and recording widget for video capture and reports."""

import os
from pathlib import Path
from datetime import datetime

import numpy as np
import qtawesome as qta

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QToolButton,
    QLabel, QMenu, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QFont, QPainter, QColor, QImage, QPen, QBrush, QLinearGradient
from PyQt5.QtPrintSupport import QPrinter


class ExportRecordWidget(QWidget):
    """Widget for export and video recording functionality.
    
    Features:
    - PNG image export with annotations
    - PDF report generation with telemetry
    - Video recording to MP4
    """
    
    def __init__(self, get_frame_callback, get_telemetry_callback):
        """Initialize export/record widget.
        
        Args:
            get_frame_callback: Callable that returns current video frame as QPixmap
            get_telemetry_callback: Callable that returns current telemetry data as dict
        """
        super().__init__()
        self.get_frame = get_frame_callback
        self.get_telemetry = get_telemetry_callback
        self.export_format = 'png'  # Default format
        self.is_recording = False
        self.video_writer = None
        self.record_start_time = None
        self.record_filename = None
        self.downloads_path = str(Path.home() / 'Downloads')
        
        # cv2 will be set dynamically from main module
        self._cv2 = None
        
        # ArUco marker names dictionary
        self.aruco_marker_names = {}
        
        self.setup_ui()
        
        # Timer for recording
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.write_frame)
    
    def set_cv2(self, cv2_module):
        """Set cv2 module reference for video operations."""
        self._cv2 = cv2_module
    
    def set_aruco_names(self, names: dict):
        """Set ArUco marker names for PDF export.
        
        Args:
            names: Dictionary mapping marker IDs to names {id: name}
        """
        self.aruco_marker_names = names
    
    def setup_ui(self):
        """Setup the widget UI."""
        layout = QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(0)  # No spacing between export and format buttons
        
        # Export button (left side of combined button)
        self.export_btn = QPushButton()
        self.export_btn.setIcon(qta.icon('mdi.download', color='white'))
        self.export_btn.setToolTip("Export frame")
        self.export_btn.setMinimumHeight(36)
        self.export_btn.setMinimumWidth(36)
        self.export_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px 0px 0px 5px;
                padding: 6px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        self.export_btn.clicked.connect(self.do_export)
        layout.addWidget(self.export_btn)
        
        # Format dropdown button (right side of combined button)
        self.format_btn = QToolButton()
        self.format_btn.setText("PNG")
        self.format_btn.setMinimumHeight(46)
        self.format_btn.setMinimumWidth(70)
        self.format_btn.setPopupMode(QToolButton.InstantPopup)
        self.format_btn.setStyleSheet("""
            QToolButton {
                background-color: #1565C0;
                color: white;
                border: none;
                border-radius: 0px 5px 5px 0px;
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
            }
            QToolButton:hover {
                background-color: #0D47A1;
            }
            QToolButton::menu-indicator {
                image: none;
            }
        """)
        
        # Format menu
        format_menu = QMenu(self)
        format_menu.setStyleSheet("""
            QMenu {
                background-color: #1a1a2e;
                color: white;
                border: 1px solid #333;
            }
            QMenu::item:selected {
                background-color: #2196F3;
            }
        """)
        
        png_action = format_menu.addAction("PNG (Image)")
        png_action.triggered.connect(lambda: self.set_format('png'))
        pdf_action = format_menu.addAction("PDF (Report)")
        pdf_action.triggered.connect(lambda: self.set_format('pdf'))
        
        self.format_btn.setMenu(format_menu)
        layout.addWidget(self.format_btn)
        
        # Add spacing before record button
        layout.addSpacing(8)
        
        # Record button
        self.record_btn = QPushButton()
        self.record_btn.setIcon(qta.icon('mdi.record-circle-outline', color='white'))
        self.record_btn.setToolTip("Start/Stop recording")
        self.record_btn.setMinimumHeight(36)
        self.record_btn.setMinimumWidth(36)
        self.record_btn.setCheckable(True)
        self.record_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 6px;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
            QPushButton:checked {
                background-color: #F44336;
            }
            QPushButton:checked:hover {
                background-color: #D32F2F;
            }
        """)
        self.record_btn.clicked.connect(self.toggle_recording)
        layout.addWidget(self.record_btn)
        
        # Recording time label
        self.record_time_label = QLabel("00:00")
        self.record_time_label.setStyleSheet("""
            color: #888;
            font-size: 11px;
            font-weight: bold;
        """)
        self.record_time_label.setFixedWidth(45)
        self.record_time_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.record_time_label)
        
        self.setLayout(layout)
    
    def set_format(self, fmt):
        """Set export format.
        
        Args:
            fmt: Format string ('png' or 'pdf')
        """
        self.export_format = fmt
        if fmt == 'png':
            self.format_btn.setText("PNG")
        else:
            self.format_btn.setText("PDF")
    
    def do_export(self):
        """Export current frame with annotations."""
        try:
            # Get current frame from drone video widget
            pixmap = self.get_frame()
            if pixmap is None or pixmap.isNull():
                QMessageBox.warning(
                    self, "Export Error", 
                    "No video frame available to export."
                )
                return
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            if self.export_format == 'png':
                self.export_png(pixmap, timestamp)
            else:
                self.export_pdf(pixmap, timestamp)
                
        except Exception as e:
            QMessageBox.critical(
                self, "Export Error", 
                f"Failed to export: {str(e)}"
            )
    
    def export_png(self, pixmap, timestamp):
        """Export as PNG image.
        
        Args:
            pixmap: QPixmap to export
            timestamp: Timestamp string for filename
        """
        filename = os.path.join(
            self.downloads_path, 
            f"tello_capture_{timestamp}.png"
        )
        
        if pixmap.save(filename, "PNG"):
            QMessageBox.information(
                self, "Export Success", 
                f"Image saved to:\n{filename}"
            )
        else:
            QMessageBox.warning(self, "Export Error", "Failed to save image.")
    
    def export_pdf(self, pixmap, timestamp):
        """Export as PDF report with telemetry (simple, non-overlapping layout)."""
        filename = os.path.join(
            self.downloads_path,
            f"tello_report_{timestamp}.pdf"
        )

        telemetry = self.get_telemetry()

        printer = QPrinter(QPrinter.HighResolution)
        printer.setOutputFormat(QPrinter.PdfFormat)
        printer.setOutputFileName(filename)
        printer.setPageSize(printer.A4)

        painter = QPainter()
        if not painter.begin(printer):
            QMessageBox.warning(self, "Export Error", "Failed to create PDF.")
            return

        try:
            painter.setRenderHint(QPainter.Antialiasing)

            page_rect = printer.pageRect()
            page_width = int(page_rect.width())
            page_height = int(page_rect.height())
            margin = 72  # ~1 inch
            content_width = page_width - margin * 2

            # ===== BACKGROUND =====
            painter.fillRect(0, 0, page_width, page_height, QColor(255, 255, 255))

            # ===== HEADER =====
            header_height = 110
            grad = QLinearGradient(0, 0, page_width, 0)
            grad.setColorAt(0, QColor(33, 150, 243))
            grad.setColorAt(1, QColor(0, 188, 212))
            painter.fillRect(0, margin, page_width, header_height, grad)

            # Title
            painter.setFont(QFont("Arial", 20, QFont.Bold))
            painter.setPen(QColor(0, 0, 0))
            title_y = header_height // 2 + 5
            painter.drawText(margin, title_y+margin, "TELLO DRONE FLIGHT REPORT")

            # Timestamp (right)
            painter.setFont(QFont("Arial", 9))
            painter.setPen(QColor(255, 255, 255, 220))
            time_str = datetime.now().strftime("%Y-%m-%d  |  %H:%M:%S")
            fm_time = painter.fontMetrics()
            tw = fm_time.horizontalAdvance(time_str)
            painter.drawText(page_width - margin - tw, title_y+margin, time_str)

            # Start content below header
            y_pos = header_height + 30 + margin

            # ===== CAPTURED FRAME =====
            self._draw_section_label(painter, margin, y_pos, "CAPTURED FRAME")
            painter.setFont(QFont("Arial", 10))
            y_pos += painter.fontMetrics().height() + 8

            # Scale image
            img_max_w = int(content_width * 0.7)
            img_w = img_max_w
            img_h = int(img_w * pixmap.height() / max(1, pixmap.width()))
            img_max_h = int(page_height * 0.27)
            if img_h > img_max_h:
                img_h = img_max_h
                img_w = int(img_h * pixmap.width() / max(1, pixmap.height()))

            scaled = pixmap.scaled(
                img_w, img_h,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            img_x = margin + (content_width - scaled.width()) // 2

            # Shadow
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(0, 0, 0, 30))
            painter.drawRoundedRect(
                QRectF(img_x + 5, y_pos + 5, scaled.width(), scaled.height()),
                6, 6
            )

            # Border
            painter.setPen(QPen(QColor(220, 220, 220), 1))
            painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(
                QRectF(img_x - 2, y_pos - 2, scaled.width() + 4, scaled.height() + 4),
                6, 6
            )

            # Image
            painter.drawPixmap(int(img_x), int(y_pos), scaled)
            y_pos += scaled.height() + 30

            # ===== TELEMETRY =====
            self._draw_section_label(painter, margin, y_pos, "TELEMETRY DATA")
            painter.setFont(QFont("Arial", 10))
            fm = painter.fontMetrics()
            line_h = fm.height() + 3
            y_pos += fm.height() + 8

            if telemetry:
                # disusun sebagai list vertikal agar aman
                lines = [
                    ("Battery", f"{telemetry.get('battery', '--')} %"),
                    ("Flight time", f"{telemetry.get('flight_time', '--')} s"),
                    ("Altitude (ToF)", f"{telemetry.get('tof', '--')} cm"),
                    ("Barometer", f"{telemetry.get('barometer', '--')} cm"),
                    (
                        "Temperature",
                        f"{telemetry.get('temperature_low', '--')} – {telemetry.get('temperature_high', '--')} °C",
                    ),
                    (
                        "Attitude",
                        f"P: {telemetry.get('pitch', '--')}°  "
                        f"R: {telemetry.get('roll', '--')}°  "
                        f"Y: {telemetry.get('yaw', '--')}°",
                    ),
                ]

                for label, value in lines:
                    text = f"{label}: {value}"
                    rect = QRectF(margin, y_pos, content_width, line_h)
                    painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, text)
                    y_pos += line_h

                y_pos += 15
            else:
                painter.setPen(QColor(120, 120, 120))
                rect = QRectF(margin, y_pos, content_width, line_h)
                painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, "No telemetry data available")
                y_pos += line_h + 15

            # ===== VELOCITY =====
            self._draw_section_label(painter, margin, y_pos, "VELOCITY")
            painter.setFont(QFont("Arial", 10))
            fm = painter.fontMetrics()
            line_h = fm.height() + 3
            y_pos += fm.height() + 8

            vel_lines = [
                ("X (Forward)", f"{telemetry.get('velocity_x', '--')} cm/s" if telemetry else "--"),
                ("Y (Lateral)", f"{telemetry.get('velocity_y', '--')} cm/s" if telemetry else "--"),
                ("Z (Vertical)", f"{telemetry.get('velocity_z', '--')} cm/s" if telemetry else "--"),
            ]
            for label, value in vel_lines:
                rect = QRectF(margin, y_pos, content_width, line_h)
                painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, f"{label}: {value}")
                y_pos += line_h

            y_pos += 15

            # ===== ARUCO MARKERS =====
            if self.aruco_marker_names:
                self._draw_section_label(painter, margin, y_pos, "ARUCO MARKERS")
                painter.setFont(QFont("Arial", 10))
                fm = painter.fontMetrics()
                line_h = fm.height() + 3
                y_pos += fm.height() + 8

                if not self.aruco_marker_names:
                    rect = QRectF(margin, y_pos, content_width, line_h)
                    painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, "No markers detected.")
                    y_pos += line_h
                else:
                    for marker_id, name in sorted(self.aruco_marker_names.items()):
                        txt = f"Marker #{marker_id}: {name}"
                        rect = QRectF(margin, y_pos, content_width, line_h)
                        painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, txt)
                        y_pos += line_h

                y_pos += 10

            # ===== FOOTER =====
            footer_y = page_height - 50
            painter.setPen(QPen(QColor(200, 200, 200), 1))
            painter.drawLine(margin, footer_y, page_width - margin, footer_y)

            painter.setFont(QFont("Arial", 8))
            painter.setPen(QColor(150, 150, 150))
            painter.drawText(margin, footer_y + 22, "Generated by Tello Drone Control Center")

            fm = painter.fontMetrics()
            report_id = f"Report ID: {timestamp}"
            rw = fm.horizontalAdvance(report_id)
            painter.drawText(page_width - margin - rw, footer_y + 22, report_id)

        finally:
            painter.end()

        QMessageBox.information(
            self, "Export Success",
            f"PDF report saved to:\n{filename}"
        )


    def _draw_section_label(self, painter, x, y, text):
        """Draw a section label with underline (improved positions)."""
        label_font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(label_font)
        painter.setPen(QColor(33, 150, 243))
        fm = painter.fontMetrics()
        painter.drawText(x, y + fm.ascent(), text)

        text_width = fm.horizontalAdvance(text)
        underline_y = y + fm.ascent() + 6
        painter.setPen(QPen(QColor(33, 150, 243), 2))
        painter.drawLine(x, underline_y, x + text_width, underline_y)


    def _draw_card_light(self, painter, x, y, width, height, label, value, accent_color):
        """Draw a styled card with stable text layout (uses QRect for vertical centering)."""
        # Shadow
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 20))
        painter.drawRoundedRect(QRectF(x + 3, y + 3, width, height), 4, 4)

        # Card background
        painter.setBrush(QBrush(QColor(250, 250, 250)))
        painter.setPen(QPen(QColor(230, 230, 230), 1))
        painter.drawRoundedRect(QRectF(x, y, width, height), 4, 4)

        # Accent bar left
        painter.setBrush(accent_color)
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(QRectF(x, y, 6, height), 2, 2)

        # Label (small)
        label_font = QFont("Arial", 8)
        painter.setFont(label_font)
        painter.setPen(QColor(100, 100, 100))
        label_rect = QRectF(x + 12, y + 8, width - 24, height / 2 - 8)
        painter.drawText(label_rect, Qt.AlignLeft | Qt.AlignVCenter, label)

        # Value (bolder, vertically centered in bottom half)
        value_font = QFont("Arial", 11, QFont.Bold)
        painter.setFont(value_font)
        painter.setPen(QColor(40, 40, 40))
        value_rect = QRectF(x + 12, y + height / 2 - 2, width - 24, height / 2 + 2)
        painter.drawText(value_rect, Qt.AlignLeft | Qt.AlignVCenter, str(value))
  
  
    def toggle_recording(self):
        """Toggle video recording."""
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()
    
    def start_recording(self):
        """Start video recording."""
        if self._cv2 is None:
            QMessageBox.warning(
                self, "Record Error", 
                "OpenCV not initialized."
            )
            self.record_btn.setChecked(False)
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(
                self.downloads_path, 
                f"tello_flight_{timestamp}.mp4"
            )
            
            # Get frame size from current frame
            pixmap = self.get_frame()
            if pixmap is None or pixmap.isNull():
                QMessageBox.warning(
                    self, "Record Error", 
                    "No video frame available."
                )
                self.record_btn.setChecked(False)
                return
            
            width = pixmap.width()
            height = pixmap.height()
            
            # Initialize video writer
            fourcc = self._cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = self._cv2.VideoWriter(
                filename, fourcc, 10.0, (width, height)
            )
            
            if not self.video_writer.isOpened():
                QMessageBox.warning(
                    self, "Record Error", 
                    "Failed to create video file."
                )
                self.record_btn.setChecked(False)
                return
            
            self.is_recording = True
            self.record_start_time = datetime.now()
            self.record_filename = filename
            self.record_btn.setText("Stop")
            self.record_time_label.setStyleSheet(
                "color: #F44336; font-size: 11px; font-weight: bold;"
            )
            
            # Start recording timer (10 FPS)
            self.record_timer.start(100)
            
            print(f"Recording started: {filename}")
            
        except Exception as e:
            QMessageBox.critical(
                self, "Record Error", 
                f"Failed to start recording: {str(e)}"
            )
            self.record_btn.setChecked(False)
    
    def write_frame(self):
        """Write current frame to video."""
        if self._cv2 is None:
            return
            
        if not self.is_recording or self.video_writer is None:
            return
        
        try:
            # Update recording time
            elapsed = (datetime.now() - self.record_start_time).total_seconds()
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            self.record_time_label.setText(f"{mins:02d}:{secs:02d}")
            
            # Get current frame
            pixmap = self.get_frame()
            if pixmap is None or pixmap.isNull():
                return
            
            # Convert QPixmap to cv2 image
            qimage = pixmap.toImage().convertToFormat(QImage.Format_RGB888)
            width = qimage.width()
            height = qimage.height()
            ptr = qimage.bits()
            ptr.setsize(height * width * 3)
            arr = np.array(ptr).reshape(height, width, 3)
            
            # Convert RGB to BGR for OpenCV
            bgr_frame = self._cv2.cvtColor(arr, self._cv2.COLOR_RGB2BGR)
            
            # Write frame
            self.video_writer.write(bgr_frame)
            
        except Exception as e:
            print(f"Error writing frame: {e}")
    
    def stop_recording(self):
        """Stop video recording."""
        self.is_recording = False
        self.record_timer.stop()
        
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            
            QMessageBox.information(
                self, "Recording Saved", 
                f"Video saved to:\n{self.record_filename}"
            )
        
        self.record_btn.setText("Record")
        self.record_btn.setChecked(False)
        self.record_time_label.setText("00:00")
        self.record_time_label.setStyleSheet(
            "color: #888; font-size: 11px; font-weight: bold;"
        )
        
        print("Recording stopped")
    
    def cleanup(self):
        """Cleanup resources."""
        if self.is_recording:
            self.stop_recording()
