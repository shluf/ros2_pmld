#!/usr/bin/env python3
"""Control arbitrator node for multiplexing cmd_vel commands.

Routes cmd_vel from different sources (manual, gesture, tracking) based on current mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from tello_interfaces.msg import ControlMode
import time


class ControlArbitratorNode(Node):
    """ROS 2 node for arbitrating control commands."""

    def __init__(self):
        super().__init__('control_arbitrator_node')

        # Parameters
        self.declare_parameter('command_timeout', 0.5)  # Timeout for cmd_vel (seconds)
        self.declare_parameter('publish_rate', 20.0)  # Hz

        self.command_timeout = self.get_parameter('command_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State
        self.current_mode = 'manual'  # Default mode
        self.transitioning = False
        
        # Last received commands
        self.last_manual_cmd = None
        self.last_manual_time = None

        self.last_joy_cmd = None
        self.last_joy_time = None
        
        self.last_gesture_cmd = None
        self.last_gesture_time = None
        
        self.last_tracking_cmd = None
        self.last_tracking_time = None

        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.mode_sub = self.create_subscription(
            ControlMode,
            '/control_mode',
            self.mode_callback,
            qos_reliable
        )

        self.manual_sub = self.create_subscription(
            Twist,
            '/manual/cmd_vel',
            self.manual_callback,
            qos_reliable
        )

        self.joy_sub = self.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.joy_callback,
            qos_reliable
        )

        self.gesture_sub = self.create_subscription(
            Twist,
            '/gesture/cmd_vel',
            self.gesture_callback,
            qos_reliable
        )

        self.tracking_sub = self.create_subscription(
            Twist,
            '/tracking/cmd_vel',
            self.tracking_callback,
            qos_reliable
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )

        # Timer for publishing selected command
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_selected_command
        )

        self.get_logger().info('Control Arbitrator initialized')
        self.get_logger().info(f'Command timeout: {self.command_timeout}s')
        self.get_logger().info(f'Publish rate: {self.publish_rate}Hz')

    def mode_callback(self, msg: ControlMode):
        """Update current mode."""
        if msg.mode != self.current_mode:
            self.get_logger().info(f'Mode changed: {self.current_mode} → {msg.mode}')
            self.current_mode = msg.mode
        
        self.transitioning = msg.transitioning

    def manual_callback(self, msg: Twist):
        """Receive manual control command."""
        self.last_manual_cmd = msg
        self.last_manual_time = time.time()

    def joy_callback(self, msg: Twist):
        """Receive joystick control command."""
        self.last_joy_cmd = msg
        self.last_joy_time = time.time()

    def gesture_callback(self, msg: Twist):
        """Receive gesture control command."""
        self.last_gesture_cmd = msg
        self.last_gesture_time = time.time()

    def tracking_callback(self, msg: Twist):
        """Receive tracking control command."""
        self.last_tracking_cmd = msg
        self.last_tracking_time = time.time()

    def publish_selected_command(self):
        """Select and publish appropriate cmd_vel based on current mode."""
        current_time = time.time()
        selected_cmd = None
        source = None

        # If transitioning, don't publish any command (mode_manager handles hover)
        if self.transitioning:
            return

        # Select command based on mode
        if self.current_mode == 'manual':
            if self.is_command_valid(self.last_manual_time, current_time):
                selected_cmd = self.last_manual_cmd
                source = 'manual'

        elif self.current_mode == 'joystick':
            if self.is_command_valid(self.last_joy_time, current_time):
                selected_cmd = self.last_joy_cmd
                source = 'joystick'

        elif self.current_mode == 'gesture':
            if self.is_command_valid(self.last_gesture_time, current_time):
                selected_cmd = self.last_gesture_cmd
                source = 'gesture'

        elif self.current_mode == 'tracking':
            if self.is_command_valid(self.last_tracking_time, current_time):
                selected_cmd = self.last_tracking_cmd
                source = 'tracking'

        # Publish selected command or stop command if none valid
        if selected_cmd is not None:
            self.cmd_vel_pub.publish(selected_cmd)
            self.get_logger().debug(
                f'Publishing from {source}: '
                f'x={selected_cmd.linear.x:.2f}, y={selected_cmd.linear.y:.2f}, '
                f'z={selected_cmd.linear.z:.2f}, yaw={selected_cmd.angular.z:.2f}'
            )
        else:
            # No valid command - publish stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().debug(f'No valid command for mode {self.current_mode} - publishing stop')

    def is_command_valid(self, cmd_time, current_time):
        """Check if command is still valid (not timed out)."""
        if cmd_time is None:
            return False
        return (current_time - cmd_time) < self.command_timeout


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ControlArbitratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
