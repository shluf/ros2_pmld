#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import json

from .gesture_recognition import GestureRecognition

class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__('gesture_detector_node')
        
        # Parameters
        self.declare_parameter('use_drone_camera', True)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('webcam_id', 0)
        self.declare_parameter('show_camera', True)
        
        self.use_drone_camera = self.get_parameter('use_drone_camera').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.webcam_id = self.get_parameter('webcam_id').value
        self.show_camera = self.get_parameter('show_camera').value
        
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
        
        # Subscriber for drone camera
        if self.use_drone_camera:
            self.image_sub = self.create_subscription(
                Image,
                'image_raw', 
                self.image_callback,
                qos_best_effort
            )
            self.get_logger().info('Subscribed to image_raw')
        
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
        
        # Initialize gesture recognition
        pkg_share = get_package_share_directory('gesture_control')
        model_path = os.path.join(pkg_share, 'model')
        
        self.gesture_recognizer = GestureRecognition(
            model_path=model_path,
            debug=self.debug_mode
        )
        
        # Webcam for debug/standalone mode
        self.webcam = None
        if not self.use_drone_camera:
            self.get_logger().info(f'Using webcam {self.webcam_id}')
            self.webcam = cv2.VideoCapture(self.webcam_id)
            self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
            self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
        
        self.received_first_frame = False
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_loop)  # 30 FPS
        
        self.get_logger().info('Gesture Detector Node initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.gesture_recognizer.update_frame(cv_image)
            if not self.received_first_frame:
                self.received_first_frame = True
                self.get_logger().info('First frame received')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_loop(self):
        # Handle webcam if not using drone camera
        if not self.use_drone_camera and self.webcam is not None:
            ret, frame = self.webcam.read()
            if ret:
                self.gesture_recognizer.update_frame(frame)
        
        # Skip if waiting for drone camera
        if self.use_drone_camera and not self.received_first_frame:
            return

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
            'hand_position': hand_position, # [x, y] or None
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
