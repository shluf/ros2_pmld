#!/usr/bin/env python3
"""Main entry point for Tello Control GUI application.

This module initializes the Qt application, imports OpenCV and CvBridge,
and launches the main window.
"""

import sys
from PyQt5.QtWidgets import QApplication

# Global cv2 reference for modules that need it
cv2 = None
CvBridge = None


def main():
    """Main entry point."""
    global cv2, CvBridge
    
    app = QApplication(sys.argv)
    
    # Import cv2 and CvBridge after QApplication is created
    import cv2 as cv2_module
    from cv_bridge import CvBridge as CvBridge_class
    
    cv2 = cv2_module
    CvBridge = CvBridge_class
    
    # Set cv2 module for widgets that need it
    from .widgets import video_widget
    video_widget.set_cv2_module(cv2)
    
    # Set cv2 and CvBridge for ros_node
    from .core import ros_node
    ros_node.set_cv_modules(cv2, CvBridge)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Import and create main window
    from .main_window import MainWindow
    
    window = MainWindow(cv2_module=cv2)
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
