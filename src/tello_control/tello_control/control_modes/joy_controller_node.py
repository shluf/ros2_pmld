#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from tello_interfaces.msg import ControlMode
    CONTROL_MODE_AVAILABLE = True
except ImportError:
    CONTROL_MODE_AVAILABLE = False

class JoyControllerNode(Node):
    def __init__(self):
        super().__init__('joy_controller_node')
        
        # Parameters
        self.declare_parameter('deadman_button', 0)  # Trigger
        self.declare_parameter('takeoff_button', 2)  # Button 3 (Thumb)
        self.declare_parameter('land_button', 1)     # Button 2 (Thumb)
        self.declare_parameter('emergency_button', 6) # Base button
        self.declare_parameter('axis_linear_x', 1)   # Pitch
        self.declare_parameter('axis_linear_y', 0)   # Roll
        self.declare_parameter('axis_linear_z', 3)   # Throttle
        self.declare_parameter('axis_angular_z', 2)  # Yaw
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 0.5)
        
        self.deadman_btn = self.get_parameter('deadman_button').value
        self.takeoff_btn = self.get_parameter('takeoff_button').value
        self.land_btn = self.get_parameter('land_button').value
        self.emergency_btn = self.get_parameter('emergency_button').value
        
        self.axis_lin_x = self.get_parameter('axis_linear_x').value
        self.axis_lin_y = self.get_parameter('axis_linear_y').value
        self.axis_lin_z = self.get_parameter('axis_linear_z').value
        self.axis_ang_z = self.get_parameter('axis_angular_z').value
        
        self.scale_lin = self.get_parameter('scale_linear').value
        self.scale_ang = self.get_parameter('scale_angular').value
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/joy/cmd_vel',
            10
        )
        
        # Service Client
        self.tello_client = self.create_client(TelloAction, 'tello_action')
        
        # Mode handling
        self.current_mode = 'manual'
        if CONTROL_MODE_AVAILABLE:
            self.mode_sub = self.create_subscription(
                ControlMode,
                '/control_mode',
                self.mode_callback,
                10
            )
            
        self.get_logger().info('Joy Controller Node initialized')
        
    def mode_callback(self, msg):
        self.current_mode = msg.mode

    def joy_callback(self, msg):
        # Check buttons
        if msg.buttons[self.takeoff_btn]:
            self.send_action('takeoff')
        elif msg.buttons[self.land_btn]:
            self.send_action('land')
        elif msg.buttons[self.emergency_btn]:
            self.send_action('emergency')
            
        # Movement
        twist = Twist()
        
        # Only move if deadman switch is held
        if msg.buttons[self.deadman_btn]:
            twist.linear.x = msg.axes[self.axis_lin_x] * self.scale_lin
            twist.linear.y = msg.axes[self.axis_lin_y] * self.scale_lin
            twist.angular.z = msg.axes[self.axis_ang_z] * self.scale_ang
            
            # Throttle might be 0..1 or -1..1 depending on joystick
            # Assuming -1..1 where -1 is up? Or standard?
            # Usually up is -1 on some, 1 on others. Let's assume standard: forward/up is positive.
            # Logitech slider: -1 (up) to 1 (down) usually.
            # Let's invert it so up is positive.
            twist.linear.z = -msg.axes[self.axis_lin_z] * self.scale_lin
            
        self.cmd_vel_pub.publish(twist)

    def send_action(self, cmd):
        if not self.tello_client.service_is_ready():
            return
        req = TelloAction.Request()
        req.cmd = cmd
        self.tello_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = JoyControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
