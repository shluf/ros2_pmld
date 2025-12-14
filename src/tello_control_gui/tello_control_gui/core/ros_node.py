"""ROS2 Node for Tello drone control GUI communication."""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseArray
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction
from std_msgs.msg import String, Bool, Int32MultiArray
from tello_interfaces.msg import DetectionArray, ObjectDistanceArray, ControlMode
from tello_interfaces.srv import SetControlMode, SetGestureCamera, SetMirror
from std_srvs.srv import Trigger

from .signal_emitter import SignalEmitter

# These will be set by main.py after importing cv2
cv2 = None
CvBridge = None


def set_cv_modules(cv2_module, cvbridge_class):
    """Set cv2 and CvBridge modules after import."""
    global cv2, CvBridge
    cv2 = cv2_module
    CvBridge = cvbridge_class


class TelloControlNode(Node):
    """ROS2 Node untuk komunikasi dengan drone.
    
    This node handles all ROS2 communication including:
    - Subscribing to drone camera, telemetry, and detection topics
    - Publishing velocity commands and gesture enable/disable
    - Calling services for drone actions and mode switching
    """
    
    def __init__(self, signal_emitter: SignalEmitter):
        """Initialize the Tello control node.
        
        Args:
            signal_emitter: SignalEmitter instance for Qt signal communication
        """
        super().__init__('tello_control_gui')
        
        # Store signal emitter
        self.signals = signal_emitter
        
        # Parameters
        self.declare_parameter('namespace', 'drone1')
        self.namespace = self.get_parameter('namespace').value
        
        # Drone camera mirror mode (controlled via service, not client-side)
        self.drone_camera_mirror = False
        
        # Setup QoS profiles
        self._setup_qos_profiles()
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Setup publishers
        self._setup_publishers()
        
        # Setup service clients
        self._setup_service_clients()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Setup webcam publisher
        self._setup_webcam_publisher()
        
        # State
        self.current_flight_data = {}
        self.gesture_enabled = False
        
        self.get_logger().info('Tello Control GUI Node initialized')
    
    def _setup_qos_profiles(self):
        """Setup QoS profiles for different communication types."""
        # Sensor data uses BEST_EFFORT for low latency
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Commands use RELIABLE for guaranteed delivery
        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    def _setup_subscribers(self):
        """Setup all ROS2 subscribers."""
        # Main image subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            self.sensor_qos
        )
        
        # Flight data subscriber
        self.flight_data_sub = self.create_subscription(
            FlightData,
            '/flight_data',
            self.flight_data_callback,
            self.sensor_qos
        )
        
        # Gesture status subscriber
        self.gesture_status_sub = self.create_subscription(
            String,
            '/gesture_recognition/status',
            self.gesture_callback,
            self.sensor_qos
        )
        
        # Detection subscriber
        self.detection_sub = self.create_subscription(
            DetectionArray, 
            '/detections', 
            self.detection_callback, 
            self.sensor_qos
        )
        
        # ArUco poses subscriber
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            '/aruco_poses', 
            self.aruco_callback, 
            self.sensor_qos
        )
        
        # ArUco marker IDs subscriber
        self.aruco_ids_sub = self.create_subscription(
            Int32MultiArray,
            '/aruco_ids',
            self.aruco_ids_callback,
            self.sensor_qos
        )
        
        # Object distances subscriber
        self.distance_sub = self.create_subscription(
            ObjectDistanceArray, 
            '/object_distances', 
            self.distance_callback, 
            self.sensor_qos
        )
        
        # Tracking command subscriber
        self.tracking_sub = self.create_subscription(
            Twist, 
            '/tracking/cmd_vel', 
            self.tracking_callback, 
            self.sensor_qos
        )
        
        # Control mode subscriber
        self.mode_sub = self.create_subscription(
            ControlMode, 
            '/control_mode', 
            self.mode_callback, 
            self.sensor_qos
        )
        
        # cmd_vel subscriber for telemetry display
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            self.sensor_qos
        )
        
        # Gesture annotated image subscriber
        self.gesture_annotated_sub = self.create_subscription(
            Image,
            '/gesture/annotated',
            self.gesture_annotated_callback,
            self.sensor_qos
        )
    
    def _setup_publishers(self):
        """Setup all ROS2 publishers."""
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.command_qos
        )
        
        self.gesture_enable_pub = self.create_publisher(
            Bool,
            '/gesture_control/enable',
            self.command_qos
        )
    
    def _setup_service_clients(self):
        """Setup all ROS2 service clients."""
        # Mode switch service client
        self.mode_switch_client = self.create_client(
            SetControlMode,
            '/mode_switch'
        )
        if not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode switch service not available yet')
        
        # Gesture camera switch service client
        self.gesture_camera_client = self.create_client(
            SetGestureCamera,
            '/gesture/set_camera'
        )
        if not self.gesture_camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Gesture camera switch service not available yet')
        
        # Tello action service client
        self.tello_action_client = self.create_client(
            TelloAction,
            '/tello_action'
        )
        self.get_logger().info('TelloAction service client created')
        
        # Mirror service client
        self.mirror_client = self.create_client(
            SetMirror,
            '/set_mirror'
        )
        if not self.mirror_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mirror service not available yet')

        # Reconnect service client
        self.reconnect_client = self.create_client(
            Trigger,
            '/reconnect'
        )
        if not self.reconnect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reconnect service not available yet')
    
    def _setup_webcam_publisher(self):
        """Setup webcam publisher for gesture recognition."""
        self.declare_parameter('mirror_webcam_to_image_raw', False)
        self.mirror_webcam = self.get_parameter('mirror_webcam_to_image_raw').value
        
        try:
            self.webcam_pub = self.create_publisher(
                Image, '/webcam/image_raw', self.sensor_qos
            )
        except Exception:
            self.webcam_pub = None
        
        try:
            if self.mirror_webcam:
                self.webcam_mirror_pub = self.create_publisher(
                    Image, '/webcam/image_raw/mirror', self.sensor_qos
                )
            else:
                self.webcam_mirror_pub = None
        except Exception:
            self.webcam_mirror_pub = None
    
    # ==================== Callbacks ====================
    
    def image_callback(self, msg):
        """Callback untuk video stream."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Note: Mirror is now handled server-side in tello_driver
            self.signals.image_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def publish_webcam_frame(self, cv_image):
        """Publish a cv2 BGR image as a ROS Image message.
        
        This allows external gesture recognition nodes to process the laptop
        webcam frames as if they were the drone feed.
        """
        if self.webcam_pub is None:
            return
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.webcam_pub.publish(ros_img)
            
            if getattr(self, 'webcam_mirror_pub', None) is not None:
                self.webcam_mirror_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f'Error publishing webcam frame: {e}')
    
    def flight_data_callback(self, msg):
        """Callback untuk flight data."""
        flight_data = {
            'battery': msg.bat,
            'altitude': msg.h,
            'temperature_low': msg.templ,
            'temperature_high': msg.temph,
            'tof': msg.tof,
            'pitch': msg.pitch,
            'roll': msg.roll,
            'yaw': msg.yaw,
            'velocity_x': msg.vgx,
            'velocity_y': msg.vgy,
            'velocity_z': msg.vgz,
            'barometer': msg.baro,
            'flight_time': msg.time,
            'acceleration_x': msg.agx,
            'acceleration_y': msg.agy,
            'acceleration_z': msg.agz,
        }
        self.current_flight_data = flight_data
        self.signals.flight_data_signal.emit(flight_data)
    
    def gesture_callback(self, msg):
        """Callback untuk gesture status."""
        self.signals.gesture_status_signal.emit(msg.data)

    def detection_callback(self, msg):
        """Callback untuk YOLO detections."""
        self.signals.detections_signal.emit(msg.detections)

    def aruco_callback(self, msg):
        """Callback untuk ArUco poses."""
        self.signals.aruco_signal.emit(msg.poses)
    
    def aruco_ids_callback(self, msg):
        """Callback untuk ArUco marker IDs."""
        self.signals.aruco_ids_signal.emit(list(msg.data))

    def distance_callback(self, msg):
        """Callback untuk object distances."""
        self.signals.distance_signal.emit(msg.distances)

    def tracking_callback(self, msg):
        """Callback untuk tracking command."""
        data = {
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'z': msg.angular.z}
        }
        self.signals.tracking_signal.emit(data)

    def mode_callback(self, msg):
        """Callback untuk control mode changes."""
        self.signals.mode_signal.emit(msg.mode)
    
    def cmd_vel_callback(self, msg):
        """Callback untuk cmd_vel display."""
        self.signals.cmd_vel_signal.emit(msg)

    def gesture_annotated_callback(self, msg):
        """Callback untuk gesture annotated image (skeleton overlay)."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.gesture_annotated_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting gesture annotated image: {e}')

    # ==================== Command Methods ====================

    def set_mode(self, mode_name: str):
        """Request mode switch via service call.
        
        Args:
            mode_name: Name of the mode to switch to
        """
        if not self.mode_switch_client.service_is_ready():
            self.get_logger().warn(f'Mode switch service unavailable, cannot send: {mode_name}')
            return

        request = SetControlMode.Request()
        request.mode = mode_name
        request.camera_source = ''

        self.get_logger().info(f'Requesting mode switch: {mode_name}')
        future = self.mode_switch_client.call_async(request)
        future.add_done_callback(
            lambda f: self._mode_switch_response_callback(f, mode_name)
        )
    
    def send_velocity(self, vx: float, vy: float, vz: float, vw: float):
        """Send velocity command.
        
        Args:
            vx: Linear velocity X (forward/backward)
            vy: Linear velocity Y (left/right)
            vz: Linear velocity Z (up/down)
            vw: Angular velocity Z (yaw)
        """
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(vw)
        self.cmd_vel_pub.publish(msg)
    
    def send_action(self, action: str):
        """Send action command (takeoff, land, etc) using service call.
        
        Args:
            action: Action command string (takeoff, land, emergency, flip_f, etc.)
        """
        if not self.tello_action_client.service_is_ready():
            self.get_logger().warn(f'TelloAction service not available, cannot send: {action}')
            return
        
        request = TelloAction.Request()
        request.cmd = action
        
        self.get_logger().info(f'Sending action: {action}')
        
        future = self.tello_action_client.call_async(request)
        future.add_done_callback(lambda f: self._action_response_callback(f, action))
    
    def enable_gesture_control(self, enable: bool):
        """Enable/disable gesture control.
        
        Args:
            enable: True to enable, False to disable
        """
        msg = Bool()
        msg.data = enable
        self.gesture_enable_pub.publish(msg)
        self.gesture_enabled = enable
        self.get_logger().info(f'Gesture control: {"enabled" if enable else "disabled"}')
    
    def set_drone_camera_mirror(self, mirror: bool):
        """Set drone camera mirror mode via service call.
        
        Args:
            mirror: True to enable horizontal flip, False to disable
        """
        if not self.mirror_client.service_is_ready():
            self.get_logger().warn('Mirror service not available')
            return
        
        request = SetMirror.Request()
        request.mirror = mirror
        
        self.get_logger().info(f'Requesting mirror: {"ON" if mirror else "OFF"}')
        future = self.mirror_client.call_async(request)
        future.add_done_callback(self._mirror_response_callback)

    def reconnect_drone(self):
        """Request the driver to reconnect to the drone."""
        if not getattr(self, 'reconnect_client', None) or not self.reconnect_client.service_is_ready():
            self.get_logger().warn('Reconnect service not available')
            return

        request = Trigger.Request()
        self.get_logger().info('Requesting drone reconnect')
        future = self.reconnect_client.call_async(request)
        future.add_done_callback(self._reconnect_response_callback)

    def _reconnect_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Reconnect succeeded: %s', response.message)
            else:
                self.get_logger().warn('Reconnect failed: %s', response.message)
        except Exception as e:
            self.get_logger().error(f'Reconnect service call failed: {e}')
    
    def _mirror_response_callback(self, future):
        """Handle mirror service response."""
        try:
            response = future.result()
            if response.success:
                self.drone_camera_mirror = response.current_state
                self.get_logger().info(f'Mirror: {response.message}')
            else:
                self.get_logger().warn(f'Mirror failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Mirror service call failed: {e}')
    
    # ==================== Response Callbacks ====================
    
    def _action_response_callback(self, future, action):
        """Handle service response for tello action."""
        try:
            response = future.result()
            if response.rc == TelloAction.Response.OK:
                self.get_logger().info(f'Action "{action}" succeeded')
            elif response.rc == TelloAction.Response.ERROR_NOT_CONNECTED:
                self.get_logger().error(f'Action "{action}" failed: Drone not connected')
            elif response.rc == TelloAction.Response.ERROR_BUSY:
                self.get_logger().warn(f'Action "{action}" failed: Drone busy')
            else:
                self.get_logger().error(f'Action "{action}" failed with code: {response.rc}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for action "{action}": {str(e)}')

    def _mode_switch_response_callback(self, future, mode_name):
        """Log result of SetControlMode service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'Mode switch to "{mode_name}" accepted: {response.message}'
                )
            else:
                self.get_logger().warn(
                    f'Mode switch to "{mode_name}" rejected: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(
                f'Service call failed for mode "{mode_name}": {str(e)}'
            )
