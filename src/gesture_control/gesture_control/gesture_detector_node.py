#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from tello_interfaces.srv import SetGestureCamera, SetNodeActive
from tello_interfaces.msg import ControlMode
import os
import cv2
import json

from .gesture_recognition import GestureRecognition

class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__('gesture_detector')
        
        # Parameters
        self.declare_parameter('use_drone_camera', False)  # Default: webcam
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('webcam_id', 0)
        self.declare_parameter('show_camera', True)
        
        self.use_drone_camera = self.get_parameter('use_drone_camera').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.webcam_id = self.get_parameter('webcam_id').value
        self.show_camera = self.get_parameter('show_camera').value
        
        # Camera source tracking: 'webcam' or 'drone'
        self.camera_source = 'drone' if self.use_drone_camera else 'webcam'
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # QoS Profiles
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers for both camera sources
        self.drone_image_sub = self.create_subscription(
            Image,
            '/image_raw',  # Drone camera
            self.drone_image_callback,
            qos_best_effort
        )
        
        self.webcam_image_sub = self.create_subscription(
            Image,
            '/webcam/image_raw',  # Webcam from GUI
            self.webcam_image_callback,
            qos_best_effort
        )
        
        self.get_logger().info('Subscribed to /image_raw and /webcam/image_raw')
        
        # Publishers
        self.gesture_pub = self.create_publisher(
            String,
            '/gesture_recognition/status',
            qos_reliable
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            '/gesture/annotated',
            qos_reliable
        )
        
        # Service for switching camera source
        self.camera_switch_srv = self.create_service(
            SetGestureCamera,
            '/gesture/set_camera',
            self.camera_switch_callback
        )
        self.get_logger().info('Camera switch service ready: /gesture/set_camera')
        
        # Service for enabling/disabling node processing
        self.active_srv = self.create_service(
            SetNodeActive,
            '/gesture/set_active',
            self.set_active_callback
        )
        self.get_logger().info('Node active service ready: /gesture/set_active')
        
        # Subscribe to control mode to auto-pause when not in gesture mode
        self.mode_sub = self.create_subscription(
            ControlMode,
            '/control_mode',
            self.mode_callback,
            qos_reliable
        )
        
        # Processing state - paused when not in gesture mode
        self.processing_active = True
        self.current_mode = 'manual'
        
        # Initialize gesture recognition
        pkg_share = get_package_share_directory('gesture_control')
        model_path = os.path.join(pkg_share, 'model')
        
        self.gesture_recognizer = GestureRecognition(
            model_path=model_path,
            debug=self.debug_mode
        )
        
        # Webcam for standalone mode (fallback if no ROS webcam topic)
        self.webcam = None
        self.use_local_webcam = False  # Will be set if ROS webcam topic isn't available
        
        # Frame storage for both sources
        self.drone_frame = None
        self.webcam_frame = None
        self.received_drone_frame = False
        self.received_webcam_frame = False
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_loop)  # 30 FPS
        
        self.get_logger().info(f'Gesture Detector Node initialized (camera: {self.camera_source})')

    def mode_callback(self, msg: ControlMode):
        """Auto-pause/resume based on control mode."""
        old_mode = self.current_mode
        self.current_mode = msg.mode
        
        # Auto enable when switching to gesture mode
        if msg.mode == 'gesture' and not self.processing_active:
            self.processing_active = True
            self.get_logger().info('Gesture mode active - processing resumed')
        # Auto disable when leaving gesture mode
        elif msg.mode != 'gesture' and old_mode == 'gesture':
            self.processing_active = False
            self.get_logger().info(f'Switched to {msg.mode} - gesture processing paused')

    def set_active_callback(self, request, response):
        """Handle node active/pause service requests."""
        self.processing_active = request.active
        status = "enabled" if request.active else "paused"
        response.success = True
        response.message = f"Gesture detector processing {status}"
        self.get_logger().info(response.message)
        return response

    def camera_switch_callback(self, request, response):
        """Handle camera switch service requests."""
        source = request.camera_source.lower()
        
        if source not in ['webcam', 'drone']:
            response.success = False
            response.current_source = self.camera_source
            response.message = f"Invalid camera source: {source}. Use 'webcam' or 'drone'."
            self.get_logger().warn(response.message)
            return response
        
        old_source = self.camera_source
        self.camera_source = source
        
        response.success = True
        response.current_source = self.camera_source
        response.message = f"Camera source switched from '{old_source}' to '{source}'"
        self.get_logger().info(response.message)
        
        return response

    def drone_image_callback(self, msg):
        """Callback for drone camera images."""
        try:
            self.drone_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.received_drone_frame:
                self.received_drone_frame = True
                self.get_logger().info('First drone frame received')
        except Exception as e:
            self.get_logger().error(f'Error converting drone image: {e}')

    def webcam_image_callback(self, msg):
        """Callback for webcam images from GUI."""
        try:
            self.webcam_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.received_webcam_frame:
                self.received_webcam_frame = True
                self.get_logger().info('First webcam frame received')
        except Exception as e:
            self.get_logger().error(f'Error converting webcam image: {e}')

    def process_loop(self):
        """Main processing loop - process frame from selected camera source."""
        # Skip processing if paused (not in gesture mode)
        if not self.processing_active:
            return
        
        # Select frame based on camera source
        frame = None
        if self.camera_source == 'drone':
            frame = self.drone_frame
        else:  # webcam
            frame = self.webcam_frame
        
        # Skip if no frame available from selected source
        if frame is None:
            return
        
        # Update gesture recognizer with selected frame
        self.gesture_recognizer.update_frame(frame)

        # Process gestures
        hand_sign, finger_gesture, hand_position, annotated_img = self.gesture_recognizer.process(return_image=True)
        
        # Publish annotated image
        if annotated_img is not None:
            try:
                msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
                self.annotated_pub.publish(msg)
            except Exception as e:
                pass

        # Prepare gesture data
        gesture_data = {
            'hand_sign': hand_sign,
            'finger_gesture': finger_gesture,
            'hand_position': hand_position,
            'camera_source': self.camera_source,
            'timestamp': self.get_clock().now().nanoseconds
        }
        
        # Publish gesture status
        msg = String()
        msg.data = json.dumps(gesture_data)
        self.gesture_pub.publish(msg)

    def destroy_node(self):
        self.gesture_recognizer.release()
        if self.webcam is not None:
            self.webcam.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
