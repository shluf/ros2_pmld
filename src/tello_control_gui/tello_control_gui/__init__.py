"""Tello Control GUI package.

This package provides a PyQt5-based GUI for controlling Tello drones
with ROS2 integration.

Modules:
    core: Core components (signal emitter, ROS thread, ROS node)
    widgets: UI widgets (video, telemetry, controls)
    threads: Background threads (webcam capture)
    main_window: Main application window
    main: Application entry point
"""

from .main import main

__all__ = ['main']
