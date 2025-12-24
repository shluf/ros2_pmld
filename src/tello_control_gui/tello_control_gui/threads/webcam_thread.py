"""Webcam capture thread for laptop camera input."""

from PyQt5.QtCore import QThread


class WebcamThread(QThread):
    """Thread to capture webcam frames with OpenCV and emit them via signal.

    It will also optionally publish frames into ROS via the provided node so
    external gesture nodes can process the laptop camera feed.
    
    Attributes:
        signals: SignalEmitter instance for emitting webcam frames
        ros_node: TelloControlNode instance for publishing to ROS (optional)
        device: Camera device index (default 0)
        running: Thread running flag
        mirror: Whether to mirror the webcam image horizontally
    """
    
    def __init__(self, signal_emitter, ros_node=None, device=0, cv2_module=None):
        """Initialize webcam thread.
        
        Args:
            signal_emitter: SignalEmitter instance for Qt signals
            ros_node: TelloControlNode for ROS publishing (optional)
            device: Camera device index
            cv2_module: OpenCV module reference
        """
        super().__init__()
        self.signals = signal_emitter
        self.ros_node = ros_node
        self.device = device
        self.running = True
        self._cv2 = cv2_module
        self.mirror = True  # Default to mirrored for natural interaction
    
    def set_cv2(self, cv2_module):
        """Set cv2 module reference."""
        self._cv2 = cv2_module
    
    def set_mirror(self, enabled):
        """Enable or disable webcam mirroring."""
        self.mirror = enabled

    def run(self):
        """Main thread loop - capture and emit frames."""
        if self._cv2 is None:
            return
            
        try:
            cap = self._cv2.VideoCapture(self.device)
        except Exception:
            cap = None

        if cap is None or not cap.isOpened():
            # Try device 0 fallback failure handled silently
            return

        while self.running:
            try:
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.msleep(30)
                    continue

                # Apply mirror if enabled
                if self.mirror:
                    frame = self._cv2.flip(frame, 1)  # 1 = horizontal flip

                # Emit to GUI
                self.signals.webcam_image_signal.emit(frame)

                # Also publish into ROS topic for gesture recognition if available
                try:
                    if self.ros_node is not None:
                        self.ros_node.publish_webcam_frame(frame)
                except Exception:
                    pass

                # Small sleep to avoid pegging CPU
                self.msleep(30)
            except Exception:
                self.msleep(100)

        try:
            cap.release()
        except Exception:
            pass

    def stop(self):
        """Stop the webcam thread."""
        self.running = False
