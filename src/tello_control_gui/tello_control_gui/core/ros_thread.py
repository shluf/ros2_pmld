"""ROS2 thread for spinning the node."""

import rclpy
from PyQt5.QtCore import QThread


class ROSThread(QThread):
    """Thread untuk menjalankan ROS2 spin.
    
    This thread handles the ROS2 event loop separately from the Qt event loop,
    preventing blocking and ensuring responsive GUI updates.
    """
    
    def __init__(self, node):
        """Initialize ROS thread.
        
        Args:
            node: ROS2 node to spin
        """
        super().__init__()
        self.node = node
        self.running = True
        
    def run(self):
        """Run the ROS2 spin loop."""
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
    
    def stop(self):
        """Stop the ROS2 spin loop."""
        self.running = False
