#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tello_msgs.srv import TelloAction
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import cv2
from pathlib import Path
import time

from .gesture_recognition import GestureRecognition


class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        
        # Parameters
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('config_file', 'config/gesture_mapping.yaml')
        self.declare_parameter('use_drone_camera', True)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('webcam_id', 0)
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('gesture_hold_time', 1.0)
        self.declare_parameter('no_gesture_timeout', 0.5)
        
        self.namespace = self.get_parameter('namespace').value
        self.config_file = self.get_parameter('config_file').value
        self.use_drone_camera = self.get_parameter('use_drone_camera').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.webcam_id = self.get_parameter('webcam_id').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.gesture_hold_time = self.get_parameter('gesture_hold_time').value
        self.no_gesture_timeout = self.get_parameter('no_gesture_timeout').value
        
        # Load gesture mapping configuration
        self.load_config()
        
        # Initialize CV Bridge for ROS Image conversion
        self.bridge = CvBridge()
        
        # QoS Profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to FlightData to monitor drone status
        try:
            from tello_msgs.msg import FlightData
            self.flight_data_sub = self.create_subscription(
                FlightData,
                'flight_data',
                self.flight_data_callback,
                qos_best_effort
            )
            self.get_logger().info('Subscribed to flight_data')
        except ImportError:
            self.get_logger().warn('Could not import FlightData, manual is_flying tracking only')
        
        # Subscriber for drone camera (BEST_EFFORT for sensor data)
        if self.use_drone_camera:
            self.image_sub = self.create_subscription(
                Image,
                'image_raw', 
                self.image_callback,
                qos_best_effort
            )
            self.get_logger().info(f'Subscribed to image_raw (namespace: {self.namespace})')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel',
            qos_reliable
        )
        
        self.gesture_status_pub = self.create_publisher(
            String,
            '/gesture_recognition/detected_gesture',
            qos_reliable
        )
        
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/gesture_recognition/safety_status',
            qos_reliable
        )
        
        # Service Clients
        self.tello_action_client = self.create_client(
            TelloAction,
            'tello_action'
        )
        
        # Wait for service
        self.get_logger().info('Waiting for tello_action service...')
        if not self.tello_action_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('tello_action service not available, continuing anyway...')
        else:
            self.get_logger().info('tello_action service connected')
        
        # Initialize gesture recognition with package model path
        pkg_share = get_package_share_directory('gesture_control')
        model_path = os.path.join(pkg_share, 'model')
        
        self.gesture_recognizer = GestureRecognition(
            model_path=model_path,
            debug=self.debug_mode
        )
        
        # Webcam for debug mode
        self.webcam = None
        if self.debug_mode and not self.use_drone_camera:
            self.get_logger().info(f'DEBUG MODE: Using webcam {self.webcam_id}')
            self.webcam = cv2.VideoCapture(self.webcam_id)
            self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
            self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
            if not self.webcam.isOpened():
                self.get_logger().error(f'Failed to open webcam {self.webcam_id}')
                self.webcam = None
        
        # State variables
        self.current_gesture = None
        self.gesture_start_time = None
        self.last_gesture_time = time.time()
        self.is_flying = False
        self.gesture_confirmed = False
        self.received_first_frame = False
        
        # NEW: no-gesture flag to track when we are enforcing hover
        self.no_gesture_active = False
        
        # Timer for gesture processing
        self.timer = self.create_timer(0.033, self.process_gestures)  # 30 FPS
        
        self.get_logger().info(f'Gesture Controller initialized for {self.namespace}')
        self.get_logger().info(f'Debug mode: {self.debug_mode}')
        if self.debug_mode and not self.use_drone_camera:
            self.get_logger().info(f'Using webcam {self.webcam_id} for debugging')
        else:
            self.get_logger().info(f'Using drone camera: {self.use_drone_camera}')
        self.get_logger().info(f'Safety mode: {self.enable_safety}')
        self.get_logger().info(f'Gesture hold time: {self.gesture_hold_time}s')
    
    def flight_data_callback(self, msg):
        """Monitor drone flight status from FlightData"""
        was_flying = self.is_flying
        
        try:
            self.is_flying = msg.h > 10
            if was_flying != self.is_flying:
                if self.is_flying:
                    self.get_logger().info(f'🛫 Flight detected! Height: {msg.h}cm')
                else:
                    self.get_logger().info(f'🛬 Landing detected! Height: {msg.h}cm')
        except Exception as e:
            self.get_logger().error(f'Error parsing FlightData: {e}')
        
        if was_flying != self.is_flying:
            status = "FLYING" if self.is_flying else "LANDED"
            self.get_logger().info(f'Drone status changed: {status}')
    
    def image_callback(self, msg):
        """
        Callback untuk ROS Image message dari drone camera
        Convert ROS Image to OpenCV format dan update gesture recognizer
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update frame in gesture recognizer
            self.gesture_recognizer.update_frame(cv_image)
            
            if not self.received_first_frame:
                self.received_first_frame = True
                self.get_logger().info('First frame received from drone camera')
                
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
        
    def load_config(self):
        """Load gesture mapping configuration from YAML file"""
        try:
            # Get package share directory
            pkg_share = get_package_share_directory('gesture_control')
            config_path = os.path.join(pkg_share, self.config_file)
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.hand_signs = config.get('hand_signs', {})
            self.finger_gestures = config.get('finger_gestures', {})
            self.velocity_settings = config.get('velocity_settings', {
                'linear_speed': 0.5,
                'angular_speed': 0.5,
                'vertical_speed': 0.3
            })
            
            self.get_logger().info('Configuration loaded successfully')
            self.get_logger().info(f'Hand signs: {self.hand_signs}')
            self.get_logger().info(f'Finger gestures: {self.finger_gestures}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            # Use default configuration
            self.hand_signs = {
                0: 'move_forward',
                1: 'move_backward',
                2: 'rotate_with_hand',
                3: 'land'
            }
            self.finger_gestures = {
                0: 'hover',
                1: 'flip',
                2: 'rotate_ccw',
                3: 'move'
            }
            self.velocity_settings = {
                'linear_speed': 0.5,
                'angular_speed': 0.5,
                'vertical_speed': 0.3
            }
    
    def process_gestures(self):
        """Main gesture processing loop"""
        # Debug mode with webcam
        if self.debug_mode and self.webcam is not None and not self.use_drone_camera:
            ret, frame = self.webcam.read()
            if ret:
                self.gesture_recognizer.update_frame(frame)
            else:
                return
        
        # Skip if using drone camera but no frame received yet
        if self.use_drone_camera and not self.received_first_frame:
            return
            
        # Get gesture from recognition system
        hand_sign, finger_gesture, hand_position = self.gesture_recognizer.process()
        
        # If no gesture detected, immediately send hover (stop movement) while drone is flying.
        # Keep sending hover each cycle until gesture returns.
        if hand_sign is None and finger_gesture is None:
            # update last_gesture_time for timeout logic too
            self.last_gesture_time = time.time()
            # Only enforce hover if safety enabled and drone is flying
            if self.enable_safety and self.is_flying:
                # send hover continuously (every timer tick) to ensure drone stops
                # but avoid spamming logs: only log when entering no-gesture state
                if not self.no_gesture_active:
                    self.get_logger().warn('No gesture detected — enforcing HOVER (safety).')
                    self.no_gesture_active = True
                # publish hover command
                self.send_hover_command()
                # also publish safety status
                status = Bool()
                status.data = True
                self.safety_status_pub.publish(status)
            return
        else:
            # we have gesture(s); clear no-gesture active flag and safety status
            if self.no_gesture_active:
                self.get_logger().info('Gesture detected again — resuming normal control.')
                self.no_gesture_active = False
                status = Bool()
                status.data = False
                self.safety_status_pub.publish(status)
        
        # Update last gesture time
        self.last_gesture_time = time.time()
        
        # Process hand sign (static gesture) - priority higher
        if hand_sign is not None:
            gesture_command = self.hand_signs.get(hand_sign)
            if gesture_command:
                self.handle_gesture(gesture_command, 'hand_sign', hand_sign, hand_position)
        
        # Process finger gesture (dynamic movement)
        elif finger_gesture is not None and finger_gesture != 0:  # 0 is stationary
            gesture_command = self.finger_gestures.get(finger_gesture)
            if gesture_command:
                self.handle_gesture(gesture_command, 'finger_gesture', finger_gesture, hand_position)
    
    def handle_gesture(self, command, gesture_type, gesture_id, hand_position=None):
        """Handle detected gesture and execute corresponding action"""
        
        # Check if this is a new gesture
        if self.current_gesture != (gesture_type, gesture_id):
            self.current_gesture = (gesture_type, gesture_id)
            self.gesture_start_time = time.time()
            self.gesture_confirmed = False
            return
        
        # Check if gesture has been held long enough
        hold_duration = time.time() - self.gesture_start_time
        
        if self.enable_safety and hold_duration < self.gesture_hold_time:
            return
        
        if not self.gesture_confirmed:
            self.gesture_confirmed = True
            self.get_logger().info(f'Gesture confirmed: {command}')
            
            # Publish gesture status
            status_msg = String()
            status_msg.data = f'{gesture_type}:{command}'
            self.gesture_status_pub.publish(status_msg)
        
        # Execute command
        if command == 'takeoff':
            self.send_takeoff()
        elif command == 'land':
            self.send_land()
        elif command == 'emergency':
            self.send_emergency_stop()
        elif command == 'hover':
            self.send_hover_command()
        elif command == 'flip':
            self.send_flip()
        elif command == 'rotate_cw':
            self.send_rotate(1.0)
        elif command == 'rotate_ccw':
            self.send_rotate(-1.0)
        elif command == 'move_forward':
            self.send_forward()
        elif command == 'move_backward':
            self.send_backward()
        elif command == 'rotate_with_hand' and hand_position:
            self.send_rotate_with_hand(hand_position)
        elif command == 'move' and hand_position:
            self.send_movement_from_position(hand_position)
    
    def send_takeoff(self):
        """Send takeoff command"""
        if not self.is_flying:
            self._call_action_service('takeoff')
            self.is_flying = True
            self.get_logger().info('Takeoff command sent')
    
    def send_land(self):
        """Send land command"""
        if self.is_flying:
            self._call_action_service('land')
            self.is_flying = False
            self.get_logger().info('Land command sent')
    
    def send_emergency_stop(self):
        """Send emergency stop command"""
        self._call_action_service('emergency')
        self.is_flying = False
        self.get_logger().warn('EMERGENCY STOP!')
        
        # Also stop all movement
        self.send_hover_command()
    
    def send_flip(self):
        """Send flip command"""
        if self.is_flying:
            self._call_action_service('flip f')
            self.get_logger().info('Flip command sent')
    
    def _call_action_service(self, cmd):
        """Helper method to call tello action service"""
        if not self.tello_action_client.service_is_ready():
            self.get_logger().warn(f'Action service not ready, cannot send: {cmd}')
            return
        
        request = TelloAction.Request()
        request.cmd = cmd
        
        # Call async and don't wait for response (fire and forget)
        self.tello_action_client.call_async(request)
    
    def send_hover_command(self):
        """Send hover command (stop all movement)"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def send_rotate(self, direction):
        """Send rotation command"""
        if self.is_flying:
            msg = Twist()
            msg.angular.z = direction * self.velocity_settings['angular_speed']
            self.cmd_vel_pub.publish(msg)
    
    def send_movement_from_position(self, hand_position):
        """
        Calculate and send movement command based on hand position
        hand_position: (x, y) normalized coordinates (0-1)
        """
        if not self.is_flying:
            return
        
        x, y = hand_position
        
        deadzone = 0.2
        
        msg = Twist()
        
        # Horizontal movement (left-right)
        if abs(x - 0.5) > deadzone:
            msg.linear.y = (x - 0.5) * 2 * self.velocity_settings['linear_speed']
        
        # Vertical movement (up-down)
        if abs(y - 0.5) > deadzone:
            msg.linear.z = -(y - 0.5) * 2 * self.velocity_settings['vertical_speed']
        
        self.cmd_vel_pub.publish(msg)
    
    def send_forward(self):
        """Send forward movement command (Open hand gesture)"""
        if self.is_flying:
            msg = Twist()
            msg.linear.x = self.velocity_settings['linear_speed']  # Move forward
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f'✈️  Moving forward (speed: {self.velocity_settings["linear_speed"]})')
        else:
            self.get_logger().warn('⚠️  Cannot move forward: Drone is not flying! Use manual takeoff first.')
    
    def send_backward(self):
        """Send backward movement command (Pointer gesture)"""
        if self.is_flying:
            msg = Twist()
            msg.linear.x = -self.velocity_settings['linear_speed']  # Move backward
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f'✈️  Moving backward (speed: {self.velocity_settings["linear_speed"]})')
        else:
            self.get_logger().warn('⚠️  Cannot move backward: Drone is not flying! Use manual takeoff first.')
    
    def send_rotate_with_hand(self, hand_position):
        """
        Rotate drone based on hand position (Close hand gesture)
        Drone hovers in place and rotates left/right following hand position
        
        Args:
            hand_position: (x, y) normalized coordinates (0-1)
                          x: 0 (left) to 1 (right)
        """
        if not self.is_flying:
            self.get_logger().warn('⚠️  Cannot rotate: Drone is not flying! Use manual takeoff first.')
            return
        
        x, y = hand_position
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        center = 0.5
        deadzone = 0.1
        max_rotation_speed = self.velocity_settings['angular_speed']
        
        if x < (center - deadzone):
            rotation_factor = (center - deadzone - x) / (center - deadzone)
            msg.angular.z = rotation_factor * max_rotation_speed
            self.get_logger().debug(f'Rotating CCW, factor: {rotation_factor:.2f}')
        elif x > (center + deadzone):
            rotation_factor = (x - center - deadzone) / (center - deadzone)
            msg.angular.z = -rotation_factor * max_rotation_speed
            self.get_logger().debug(f'Rotating CW, factor: {rotation_factor:.2f}')
        else:
            msg.angular.z = 0.0
            self.get_logger().debug('Hand centered, no rotation')
        
        self.cmd_vel_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.gesture_recognizer.release()
        # Release webcam if used
        if self.webcam is not None:
            self.webcam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = GestureController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
