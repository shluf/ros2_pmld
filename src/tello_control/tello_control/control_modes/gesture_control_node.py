#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import time
import json

try:
    from tello_interfaces.msg import ControlMode
    CONTROL_MODE_AVAILABLE = True
except ImportError:
    CONTROL_MODE_AVAILABLE = False

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        # Parameters
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('config_file', 'config/gesture_mapping.yaml')
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('gesture_hold_time', 1.0)
        self.declare_parameter('no_gesture_timeout', 0.5)
        
        self.namespace = self.get_parameter('namespace').value
        self.config_file = self.get_parameter('config_file').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.gesture_hold_time = self.get_parameter('gesture_hold_time').value
        self.no_gesture_timeout = self.get_parameter('no_gesture_timeout').value
        
        # Load gesture mapping configuration
        self.load_config()
        
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
        
        # Subscribe to FlightData
        try:
            from tello_msgs.msg import FlightData
            self.flight_data_sub = self.create_subscription(
                FlightData,
                'flight_data',
                self.flight_data_callback,
                qos_best_effort
            )
        except ImportError:
            self.get_logger().warn('Could not import FlightData')
        
        # Subscribe to Gesture Detection
        self.gesture_sub = self.create_subscription(
            String,
            '/gesture_recognition/status',
            self.gesture_callback,
            qos_reliable
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/gesture/cmd_vel',
            qos_reliable
        )
        
        self.gesture_status_pub = self.create_publisher(
            String,
            '/gesture_control/status',
            qos_reliable
        )
        
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/gesture_control/safety_status',
            qos_reliable
        )
        
        # Subscribe to control mode
        if CONTROL_MODE_AVAILABLE:
            self.mode_sub = self.create_subscription(
                ControlMode,
                '/control_mode',
                self.mode_callback,
                qos_reliable
            )
        
        # Service Clients
        self.tello_action_client = self.create_client(
            TelloAction,
            'tello_action'
        )
        
        # State variables
        self.current_gesture = None
        self.gesture_start_time = None
        self.last_gesture_time = time.time()
        self.is_flying = False
        self.gesture_confirmed = False
        self.no_gesture_active = False
        
        # Mode-aware state
        self.current_mode = 'manual'
        self.gesture_mode_active = False
        self.transitioning = False
        
        self.get_logger().info('Gesture Control Node initialized')
    
    def mode_callback(self, msg):
        if CONTROL_MODE_AVAILABLE:
            old_mode = self.current_mode
            self.current_mode = msg.mode
            self.transitioning = msg.transitioning
            self.gesture_mode_active = (msg.mode == 'gesture' and not msg.transitioning)
            
            if old_mode != msg.mode:
                self.get_logger().info(
                    f'Mode changed: {old_mode} -> {msg.mode}, gesture_active={self.gesture_mode_active}'
                )
    
    def flight_data_callback(self, msg):
        self.is_flying = msg.h > 10
    
    def load_config(self):
        try:
            pkg_share = get_package_share_directory('tello_control')
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
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            # Defaults
            self.hand_signs = {  0: "move_forward",
                1: "hover",
                2: "move_up",
                3: "land"             ,
                    4: "move_down"        ,
                    5: "move_backward"    ,
                    6: "move_left"        ,
                    7: "move_right"       ,
                }
            self.finger_gestures = {0: 'hover', 1: 'flip', 2: 'rotate_ccw', 3: 'move'}
            self.velocity_settings = {'linear_speed': 0.5, 'angular_speed': 0.5, 'vertical_speed': 0.3}

    def gesture_callback(self, msg):
        # Log untuk debugging
        if CONTROL_MODE_AVAILABLE and not self.gesture_mode_active:
            self.get_logger().debug(
                f'Gesture ignored: mode={self.current_mode}, gesture_mode_active={self.gesture_mode_active}'
            )
            return

        try:
            data = json.loads(msg.data)
            hand_sign = data.get('hand_sign')
            finger_gesture = data.get('finger_gesture')
            hand_position = data.get('hand_position')
            
            self.get_logger().debug(f'Received gesture: hand_sign={hand_sign}, finger_gesture={finger_gesture}')
            self.process_gesture_data(hand_sign, finger_gesture, hand_position)
        except Exception as e:
            self.get_logger().error(f'Error parsing gesture data: {e}')

    def process_gesture_data(self, hand_sign, finger_gesture, hand_position):
        # If no gesture detected
        if hand_sign is None and finger_gesture is None:
            self.last_gesture_time = time.time()
            if self.enable_safety and self.is_flying:
                if not self.no_gesture_active:
                    self.get_logger().warn('No gesture - enforcing HOVER')
                    self.no_gesture_active = True
                self.send_hover_command()
                
                status = Bool()
                status.data = True
                self.safety_status_pub.publish(status)
            return
        else:
            if self.no_gesture_active:
                self.no_gesture_active = False
                status = Bool()
                status.data = False
                self.safety_status_pub.publish(status)
        
        self.last_gesture_time = time.time()
        
        # Process gestures
        if hand_sign is not None:
            cmd = self.hand_signs.get(hand_sign)
            if cmd:
                self.handle_gesture(cmd, 'hand_sign', hand_sign, hand_position)
        elif finger_gesture is not None and finger_gesture != 0:
            cmd = self.finger_gestures.get(finger_gesture)
            if cmd:
                self.handle_gesture(cmd, 'finger_gesture', finger_gesture, hand_position)

    def handle_gesture(self, command, gesture_type, gesture_id, hand_position=None):
        # Check if new gesture
        if self.current_gesture != (gesture_type, gesture_id):
            self.current_gesture = (gesture_type, gesture_id)
            self.gesture_start_time = time.time()
            self.gesture_confirmed = False
            return
        
        # Check hold time
        if self.enable_safety and (time.time() - self.gesture_start_time < self.gesture_hold_time):
            return
        
        if not self.gesture_confirmed:
            self.gesture_confirmed = True
            self.get_logger().info(f'Gesture confirmed: {command}')
            msg = String()
            msg.data = f'{gesture_type}:{command}'
            self.gesture_status_pub.publish(msg)
        
        # Execute
        self.execute_command(command, hand_position)

    def execute_command(self, command, hand_position):
        self.get_loger().warn(f'Gesture received: {command}')
        
        if command == 'takeoff':
            self.send_takeoff()
        elif command == 'land':
            self.send_land()
        elif command == 'emergency':
            self.send_emergency_stop()
        elif command == 'move_forward':
            self.send_forward()
        elif command == 'hover':
            self.send_hover_command()
        elif command == 'move_up':
            self.send_up()
        elif command == 'flip':
            self.send_flip()
        elif command == 'move_down':
            self.send_down()
        elif command == 'move_backward':
            self.send_backward()
        elif command == 'move_left':
            self.send_left()
        elif command == 'move_right':
            self.send_right()
        # elif command == 'rotate_with_hand' and hand_position:
        #     self.send_rotate_with_hand(hand_position)
        # elif command == 'move' and hand_position:
        #     self.send_movement_from_position(hand_position)
        # elif command == 'rotate_cw':
        #     self.send_rotate(1.0)
        # elif command == 'rotate_ccw':
        #     self.send_rotate(-1.0)

    # --- Command Helpers (Copied/Adapted from original) ---
    def send_takeoff(self):
        if not self.is_flying:
            self._call_action('takeoff')
            self.is_flying = True

    def send_land(self):
        if self.is_flying:
            self._call_action('land')
            self.is_flying = False

    def send_emergency_stop(self):
        self._call_action('emergency')
        self.is_flying = False
        self.send_hover_command()

    def send_flip(self):
        if self.is_flying: self._call_action('flip f')

    def _call_action(self, cmd):
        if not self.tello_action_client.service_is_ready(): return
        req = TelloAction.Request()
        req.cmd = cmd
        self.tello_action_client.call_async(req)

    def send_hover_command(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def send_rotate(self, direction):
        if self.is_flying:
            msg = Twist()
            msg.angular.z = direction * self.velocity_settings['angular_speed']
            self.cmd_vel_pub.publish(msg)

    def send_forward(self):
        if self.is_flying:
            msg = Twist()
            msg.linear.x = self.velocity_settings['linear_speed']
            self.cmd_vel_pub.publish(msg)

    def send_backward(self):
        if self.is_flying:
            msg = Twist()
            msg.linear.x = -self.velocity_settings['linear_speed']
            self.cmd_vel_pub.publish(msg)

    def send_movement_from_position(self, hand_position):
        if not self.is_flying: return
        x, y = hand_position
        deadzone = 0.2
        msg = Twist()
        if abs(x - 0.5) > deadzone:
            msg.linear.y = (x - 0.5) * 2 * self.velocity_settings['linear_speed']
        if abs(y - 0.5) > deadzone:
            msg.linear.z = -(y - 0.5) * 2 * self.velocity_settings['vertical_speed']
        self.cmd_vel_pub.publish(msg)

    def send_rotate_with_hand(self, hand_position):
        if not self.is_flying: return
        x, y = hand_position
        msg = Twist()
        center = 0.5
        deadzone = 0.1
        max_speed = self.velocity_settings['angular_speed']
        
        if x < (center - deadzone):
            factor = (center - deadzone - x) / (center - deadzone)
            msg.angular.z = factor * max_speed
        elif x > (center + deadzone):
            factor = (x - center - deadzone) / (center - deadzone)
            msg.angular.z = -factor * max_speed
        self.cmd_vel_pub.publish(msg)
    def send_down(self):
            """Send downward movement command"""
            if self.is_flying:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = -self.velocity_settings['vertical_speed']  # turun
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info(f'⬇️  Moving DOWN (speed: {self.velocity_settings["vertical_speed"]})')
            else:
                self.get_logger().warn('⚠️ Cannot move down: Drone is not flying!')
    def send_up(self):
        """Send upward movement command"""
        if self.is_flying:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = self.velocity_settings['vertical_speed']  # naik
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(
                f'⬆️  Moving UP (speed: {self.velocity_settings["vertical_speed"]})'
            )
        else:
            self.get_logger().warn('⚠️ Cannot move up: Drone is not flying!')

    def send_left(self):
        """Send left movement command"""
        if self.is_flying:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = self.velocity_settings['linear_speed']  # kiri
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f'⬅️  Moving LEFT (speed: {self.velocity_settings["linear_speed"]})')
        else:
            self.get_logger().warn('⚠️ Cannot move left: Drone is not flying!')


    def send_right(self):
        """Send right movement command"""
        if self.is_flying:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = -self.velocity_settings['linear_speed']  # kanan
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f'➡️  Moving RIGHT (speed: {self.velocity_settings["linear_speed"]})')
        else:
            self.get_logger().warn('⚠️ Cannot move right: Drone is not flying!')


def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
