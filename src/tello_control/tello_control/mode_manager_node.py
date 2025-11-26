#!/usr/bin/env python3
"""Mode manager node for Tello drone control system.

Manages control modes (manual, gesture, tracking) with graceful transitions.
Sends hover command before mode switching.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tello_interfaces.msg import ControlMode
from tello_interfaces.srv import SetControlMode
from enum import Enum
import time
from typing import Tuple


class Mode(Enum):
    """Control mode enumeration."""
    MANUAL = "manual"
    JOYSTICK = "joystick"
    GESTURE = "gesture"
    TRACKING = "tracking"


class TransitionState(Enum):
    """Mode transition state."""
    IDLE = "idle"
    HOVERING = "hovering"
    SWITCHING = "switching"


class ModeManagerNode(Node):
    """ROS 2 node for managing control modes with graceful transitions."""

    def __init__(self):
        super().__init__('mode_manager_node')

        # Parameters
        self.declare_parameter('initial_mode', 'manual')
        self.declare_parameter('hover_duration', 1.0)  # Hover time before switch (seconds)
        self.declare_parameter('camera_source', 'drone')  # 'drone' or 'webcam'

        initial_mode_str = self.get_parameter('initial_mode').value
        self.hover_duration = self.get_parameter('hover_duration').value
        camera_source_param = self.get_parameter('camera_source').value

        # State
        self.current_mode = Mode(initial_mode_str)
        self.target_mode = None
        self.transition_state = TransitionState.IDLE
        self.transition_start_time = None
        self.camera_source = camera_source_param

        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.mode_switch_srv = self.create_service(
            SetControlMode,
            '/mode_switch',
            self.mode_switch_service_callback
        )

        self.camera_switch_sub = self.create_subscription(
            String,
            '/camera_switch',
            self.camera_switch_callback,
            qos_reliable
        )

        # Publishers
        self.mode_status_pub = self.create_publisher(
            ControlMode,
            '/control_mode',
            qos_reliable
        )

        self.hover_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )

        # Timer for transition state machine
        self.transition_timer = self.create_timer(0.1, self.transition_update)

        # Timer for publishing mode status
        self.status_timer = self.create_timer(0.5, self.publish_mode_status)

        self.get_logger().info(f'Mode Manager initialized')
        self.get_logger().info(f'Initial mode: {self.current_mode.value}')
        self.get_logger().info(f'Camera source: {self.camera_source}')
        self.get_logger().info(f'Hover duration: {self.hover_duration}s')
        
        # Publish initial status
        self.publish_mode_status()

    def mode_switch_service_callback(self, request: SetControlMode.Request, response: SetControlMode.Response):
        """Handle SetControlMode service requests."""
        success, message = self._process_mode_request(request.mode)
        response.success = success
        response.current_mode = self.current_mode.value
        response.message = message

        if request.camera_source:
            cam_success, cam_message = self._process_camera_request(request.camera_source)
            if cam_message:
                response.message = f"{message} | {cam_message}" if message else cam_message
            response.success = success and cam_success

        return response

    def _process_mode_request(self, requested_mode: str) -> Tuple[bool, str]:
        """Validate and initiate a mode change request."""
        requested_mode = (requested_mode or '').strip().lower()

        if not requested_mode:
            msg = 'Empty mode request received'
            self.get_logger().error(msg)
            return False, msg

        try:
            new_mode = Mode(requested_mode)
        except ValueError:
            msg = (
                f'Invalid mode: {requested_mode}. '
                f'Valid modes: manual, joystick, gesture, tracking'
            )
            self.get_logger().error(msg)
            return False, msg

        if new_mode == self.current_mode:
            msg = f'Already in {new_mode.value} mode'
            self.get_logger().info(msg)
            return True, msg

        if self.transition_state != TransitionState.IDLE:
            target = self.target_mode.value if self.target_mode else 'unknown'
            msg = (
                f'Transition already in progress to {target}. '
                f'Ignoring request for {new_mode.value}'
            )
            self.get_logger().warn(msg)
            return False, msg

        self.get_logger().info(
            f'Mode switch requested: {self.current_mode.value} → {new_mode.value}'
        )
        self.target_mode = new_mode
        self.transition_state = TransitionState.HOVERING
        self.transition_start_time = time.time()

        self.send_hover_command()
        self.get_logger().info('Hovering before mode switch...')
        return True, f'Hovering before switching to {new_mode.value}'

    def camera_switch_callback(self, msg: String):
        """Handle camera source switch (for gesture mode)."""
        self._process_camera_request(msg.data)

    def _process_camera_request(self, requested_source: str) -> Tuple[bool, str]:
        requested_source = (requested_source or '').strip().lower()

        if not requested_source:
            msg = 'Empty camera source request received'
            self.get_logger().error(msg)
            return False, msg

        if requested_source not in ['drone', 'webcam']:
            msg = (
                f'Invalid camera source: {requested_source}. '
                f'Valid sources: drone, webcam'
            )
            self.get_logger().error(msg)
            return False, msg

        if requested_source != self.camera_source:
            self.get_logger().info(
                f'Camera source changed: {self.camera_source} → {requested_source}'
            )
            self.camera_source = requested_source
            self.publish_mode_status()
            return True, f'Camera source set to {requested_source}'

        msg = f'Camera source already set to {requested_source}'
        self.get_logger().info(msg)
        return True, msg

    def transition_update(self):
        """Update transition state machine."""
        if self.transition_state == TransitionState.IDLE:
            return

        if self.transition_state == TransitionState.HOVERING:
            # Check if hover duration elapsed
            elapsed = time.time() - self.transition_start_time

            if elapsed >= self.hover_duration:
                # Complete transition
                self.get_logger().info(
                    f'Hover complete. Switching to {self.target_mode.value} mode'
                )
                self.current_mode = self.target_mode
                self.target_mode = None
                self.transition_state = TransitionState.IDLE
                self.transition_start_time = None

                # Publish new mode status
                self.publish_mode_status()
                
                self.get_logger().info(f'✓ Now in {self.current_mode.value} mode')
            else:
                # Continue hovering
                self.send_hover_command()

    def send_hover_command(self):
        """Send hover command (all velocities to zero)."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.hover_cmd_pub.publish(msg)

    def publish_mode_status(self):
        """Publish current mode status."""
        status = ControlMode()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mode = self.current_mode.value
        status.camera_source = self.camera_source
        status.control_authority = self.current_mode.value
        
        if self.transition_state == TransitionState.HOVERING:
            status.transitioning = True
            status.target_mode = self.target_mode.value if self.target_mode else ""
        else:
            status.transitioning = False
            status.target_mode = ""

        self.mode_status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ModeManagerNode()
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
