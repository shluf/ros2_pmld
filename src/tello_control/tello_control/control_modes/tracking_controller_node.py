#!/usr/bin/env python3
"""Tracking controller node for autonomous object tracking.

Uses PID control to track detected objects in frame center.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tello_interfaces.msg import DetectionArray, ControlMode, ObjectDistanceArray
from tello_interfaces.srv import SetNodeActive
import time


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, dt=None):
        """Compute PID output."""
        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = time.time() - self.last_time
            self.last_time = time.time()

        if dt <= 0:
            dt = 0.01

        # Proportional
        p_term = self.kp * error

        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Sum and clamp
        output = p_term + i_term + d_term
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        self.prev_error = error

        return output

    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None


class TrackingControllerNode(Node):
    """ROS 2 node for PID-based autonomous tracking."""

    def __init__(self):
        super().__init__('tracking_controller_node')

        # Parameters
        self.declare_parameter('pid_x.kp', 0.5)
        self.declare_parameter('pid_x.ki', 0.0)
        self.declare_parameter('pid_x.kd', 0.1)
        
        self.declare_parameter('pid_y.kp', 0.5)
        self.declare_parameter('pid_y.ki', 0.0)
        self.declare_parameter('pid_y.kd', 0.1)
        
        self.declare_parameter('pid_z.kp', 0.4)
        self.declare_parameter('pid_z.ki', 0.0)
        self.declare_parameter('pid_z.kd', 0.08)
        
        self.declare_parameter('pid_yaw.kp', 0.3)
        self.declare_parameter('pid_yaw.ki', 0.0)
        self.declare_parameter('pid_yaw.kd', 0.05)
        
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('target_class', ['person'])  # Can be string or list
        self.declare_parameter('deadzone_pixels', 50)
        self.declare_parameter('max_tracking_distance', 3.0)
        self.declare_parameter('frame_width', 960)
        self.declare_parameter('frame_height', 720)

        # Get parameters
        pid_x_kp = self.get_parameter('pid_x.kp').value
        pid_x_ki = self.get_parameter('pid_x.ki').value
        pid_x_kd = self.get_parameter('pid_x.kd').value
        
        pid_y_kp = self.get_parameter('pid_y.kp').value
        pid_y_ki = self.get_parameter('pid_y.ki').value
        pid_y_kd = self.get_parameter('pid_y.kd').value
        
        pid_z_kp = self.get_parameter('pid_z.kp').value
        pid_z_ki = self.get_parameter('pid_z.ki').value
        pid_z_kd = self.get_parameter('pid_z.kd').value
        
        pid_yaw_kp = self.get_parameter('pid_yaw.kp').value
        pid_yaw_ki = self.get_parameter('pid_yaw.ki').value
        pid_yaw_kd = self.get_parameter('pid_yaw.kd').value
        
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        
        # Handle target_class - can be string or list of strings
        target_class_param = self.get_parameter('target_class').value
        if isinstance(target_class_param, list):
            self.target_classes = target_class_param
        else:
            self.target_classes = [target_class_param] if target_class_param else []
        
        self.deadzone = self.get_parameter('deadzone_pixels').value
        self.max_distance = self.get_parameter('max_tracking_distance').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value

        # Initialize PID controllers
        self.pid_x = PIDController(pid_x_kp, pid_x_ki, pid_x_kd, 
                                   output_limits=(-self.max_linear_vel, self.max_linear_vel))
        self.pid_y = PIDController(pid_y_kp, pid_y_ki, pid_y_kd,
                                   output_limits=(-self.max_linear_vel, self.max_linear_vel))
        self.pid_z = PIDController(pid_z_kp, pid_z_ki, pid_z_kd,
                                   output_limits=(-self.max_linear_vel, self.max_linear_vel))
        self.pid_yaw = PIDController(pid_yaw_kp, pid_yaw_ki, pid_yaw_kd,
                                     output_limits=(-self.max_angular_vel, self.max_angular_vel))

        # Frame center
        self.frame_center_x = self.frame_width / 2.0
        self.frame_center_y = self.frame_height / 2.0

        # CV Bridge
        self.bridge = CvBridge()

        # State
        self.current_mode = None
        self.tracking_enabled = False
        self.target_detected = False
        self.latest_detections = None
        self.latest_distances = None
        self.latest_image = None

        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self.detection_callback,
            qos_reliable
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_sensor
        )

        self.mode_sub = self.create_subscription(
            ControlMode,
            '/control_mode',
            self.mode_callback,
            qos_reliable
        )

        self.distance_sub = self.create_subscription(
            ObjectDistanceArray,
            '/object_distances',
            self.distance_callback,
            qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/tracking/cmd_vel',
            qos_reliable
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            '/tracking/annotated',
            qos_sensor
        )
        
        # Service for enabling/disabling node processing
        self.active_srv = self.create_service(
            SetNodeActive,
            '/tracking/set_active',
            self.set_active_callback
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Tracking Controller initialized')
        self.get_logger().info(f'Target classes: {self.target_classes if self.target_classes else "closest to center"}')
        self.get_logger().info(f'PID gains - X: {pid_x_kp}/{pid_x_ki}/{pid_x_kd}')
        self.get_logger().info(f'Deadzone: {self.deadzone}px, Max distance: {self.max_distance}m')

    def set_active_callback(self, request, response):
        """Handle node active/pause service requests."""
        if request.active:
            self.tracking_enabled = (self.current_mode == 'tracking')
            status = "enabled" if self.tracking_enabled else "standby (not in tracking mode)"
        else:
            self.tracking_enabled = False
            self.send_stop_command()
            status = "paused"
        response.success = True
        response.message = f"Tracking controller {status}"
        self.get_logger().info(response.message)
        return response

    def mode_callback(self, msg: ControlMode):
        """Update tracking state based on mode."""
        was_enabled = self.tracking_enabled
        self.current_mode = msg.mode
        self.tracking_enabled = (msg.mode == 'tracking' and not msg.transitioning)

        if self.tracking_enabled and not was_enabled:
            self.get_logger().info('✓ Tracking mode activated')
            self.reset_controllers()
        elif not self.tracking_enabled and was_enabled:
            self.get_logger().info('✗ Tracking mode deactivated')
            self.send_stop_command()

    def detection_callback(self, msg: DetectionArray):
        """Store latest detections."""
        self.latest_detections = msg

    def image_callback(self, msg: Image):
        """Store latest image."""
        self.latest_image = msg

    def distance_callback(self, msg: ObjectDistanceArray):
        """Store latest distance measurements."""
        self.latest_distances = msg

    def control_loop(self):
        """Main control loop - runs at 20Hz."""
        if not self.tracking_enabled:
            return

        if self.latest_detections is None or len(self.latest_detections.detections) == 0:
            # No detections - send stop command
            if self.target_detected:
                self.get_logger().warn('Target lost - stopping')
                self.target_detected = False
            self.send_stop_command()
            return

        # Select target
        target = self.select_target(self.latest_detections.detections)

        if target is None:
            if self.target_detected:
                self.get_logger().warn('No valid target - stopping')
                self.target_detected = False
            self.send_stop_command()
            return

        # Check distance if available
        if self.latest_distances is not None:
            target_distance = self.get_target_distance(target)
            if target_distance is not None and target_distance > self.max_distance:
                self.get_logger().warn(f'Target too far ({target_distance:.2f}m) - stopping')
                self.send_stop_command()
                return

        # Calculate errors from frame center
        # Note: error_x is negated because when target is to the right (positive error),
        # drone should move right (negative linear.y in drone frame)
        error_x = -(target.center_x - self.frame_center_x)  # Left-right (inverted for correct tracking)
        error_y = self.frame_center_y - target.center_y  # Up-down (inverted)
        
        # For forward-backward, use bbox size as proxy for distance
        # Larger bbox = closer = move back, smaller = farther = move forward
        target_bbox_height = target.height
        desired_bbox_height = self.frame_height * 0.3  # Target: object fills 30% of frame
        error_depth = desired_bbox_height - target_bbox_height

        # Apply deadzone
        if abs(error_x) < self.deadzone:
            error_x = 0.0
        if abs(error_y) < self.deadzone:
            error_y = 0.0

        # Compute PID outputs
        cmd_lr = self.pid_x.update(error_x)      # Left-right (Twist.linear.y)
        cmd_ud = self.pid_z.update(error_y)      # Up-down (Twist.linear.z)
        cmd_fb = self.pid_y.update(error_depth)  # Forward-backward (Twist.linear.x)
        cmd_yaw = 0.0  # No yaw control for now

        # Create and publish command
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_fb
        cmd_vel.linear.y = cmd_lr
        cmd_vel.linear.z = cmd_ud
        cmd_vel.angular.z = cmd_yaw

        self.cmd_vel_pub.publish(cmd_vel)
        
        # Visualization
        if self.latest_image is not None:
            try:
                annotated_img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                
                # Draw frame center
                cx, cy = int(self.frame_center_x), int(self.frame_center_y)
                cv2.line(annotated_img, (cx-20, cy), (cx+20, cy), (0, 255, 0), 1)
                cv2.line(annotated_img, (cx, cy-20), (cx, cy+20), (0, 255, 0), 1)
                
                # Draw target center
                tx, ty = int(target.center_x), int(target.center_y)
                cv2.circle(annotated_img, (tx, ty), 5, (0, 0, 255), -1)
                
                # Draw error line
                cv2.line(annotated_img, (cx, cy), (tx, ty), (0, 255, 255), 2)
                
                # Draw bounding box
                x, y = int(target.x), int(target.y)
                w, h = int(target.width), int(target.height)
                cv2.rectangle(annotated_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Draw command info
                info_text = f"CMD: x={cmd_fb:.2f} y={cmd_lr:.2f} z={cmd_ud:.2f}"
                cv2.putText(annotated_img, info_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw target info
                target_text = f"Target: {target.class_name} ({target.confidence:.2f})"
                cv2.putText(annotated_img, target_text, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish annotated image
                msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
                msg.header = self.latest_image.header
                self.annotated_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().warn(f'Visualization error: {e}')

        if not self.target_detected:
            self.get_logger().info(f'Tracking: {target.class_name}')
            self.target_detected = True

        self.get_logger().debug(
            f'Target: {target.class_name} @ ({target.center_x:.0f}, {target.center_y:.0f}) '
            f'| Errors: x={error_x:.0f}, y={error_y:.0f}, depth={error_depth:.0f} '
            f'| Cmd: fb={cmd_fb:.2f}, lr={cmd_lr:.2f}, ud={cmd_ud:.2f}'
        )

    def select_target(self, detections):
        """Select target from detections.
        
        Priority:
        1. If target_classes specified, find any of those classes
        2. Otherwise, find detection closest to frame center
        """
        if self.target_classes:
            # Find any of the specified classes
            candidates = [d for d in detections if d.class_name in self.target_classes]
            if not candidates:
                return None
            # If multiple, choose closest to center
            target = min(candidates, 
                        key=lambda d: abs(d.center_x - self.frame_center_x) + 
                                    abs(d.center_y - self.frame_center_y))
        else:
            # Choose closest to center
            target = min(detections,
                        key=lambda d: abs(d.center_x - self.frame_center_x) + 
                                    abs(d.center_y - self.frame_center_y))

        return target

    def get_target_distance(self, target):
        """Get distance to target from distance measurements."""
        if self.latest_distances is None:
            return None

        # Find distance measurement for this target
        # Match by center position (approximate)
        for dist in self.latest_distances.distances:
            if (abs(dist.object_center_x - target.center_x) < 10 and
                abs(dist.object_center_y - target.center_y) < 10):
                return dist.distance_meters

        return None

    def send_stop_command(self):
        """Send stop command (all zeros)."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def reset_controllers(self):
        """Reset all PID controllers."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self.get_logger().debug('PID controllers reset')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrackingControllerNode()
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
