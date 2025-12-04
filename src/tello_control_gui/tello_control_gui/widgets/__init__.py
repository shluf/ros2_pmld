"""Widget components for Tello Control GUI."""

from .video_widget import VideoWidget
from .telemetry_widget import TelemetryWidget
from .manual_control import ManualControlWidget
from .gesture_control import GestureControlWidget
from .export_record import ExportRecordWidget
from .speed_control import SpeedControlWidget
from .aruco_naming_dialog import ArucoNamingDialog

__all__ = [
    'VideoWidget',
    'TelemetryWidget', 
    'ManualControlWidget',
    'GestureControlWidget',
    'ExportRecordWidget',
    'SpeedControlWidget',
    'ArucoNamingDialog'
]
