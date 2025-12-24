"""Signal emitter for Qt signals from ROS2 callbacks."""

import numpy as np
from PyQt5.QtCore import pyqtSignal, QThread


class SignalEmitter(QThread):
    """QObject wrapper for emitting Qt signals from ROS2 callbacks.
    
    This class bridges ROS2 callbacks to Qt signals, allowing safe
    communication between ROS2 threads and the Qt GUI thread.
    """
    
    # Video signals
    image_signal = pyqtSignal(np.ndarray)
    webcam_image_signal = pyqtSignal(np.ndarray)
    gesture_annotated_signal = pyqtSignal(np.ndarray)
    
    # Telemetry signals
    flight_data_signal = pyqtSignal(dict)
    gesture_status_signal = pyqtSignal(str)
    
    # Annotation signals
    detections_signal = pyqtSignal(list)
    aruco_signal = pyqtSignal(list)
    aruco_ids_signal = pyqtSignal(list)  # List of marker IDs
    distance_signal = pyqtSignal(list)
    tracking_signal = pyqtSignal(dict)
    
    # Control signals
    mode_signal = pyqtSignal(str)
    cmd_vel_signal = pyqtSignal(object)  # For Twist message
