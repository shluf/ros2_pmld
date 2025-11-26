#!/usr/bin/env python3
"""ArUco marker detector node for Tello drone.

Detects ArUco markers and estimates their poses for distance reference.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    """ROS 2 node for ArUco marker detection."""

    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('marker_size', 0.10)  # Marker size in meters (10cm default)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('publish_annotated', True)
        
        # Default camera calibration (Tello approximate values)
        self.declare_parameter('camera_matrix', [921.0, 0.0, 480.0, 0.0, 921.0, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter('distortion_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])

        self.marker_size = self.get_parameter('marker_size').value
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value

        # Camera calibration
        cam_matrix_list = self.get_parameter('camera_matrix').value
        self.camera_matrix = np.array(cam_matrix_list).reshape(3, 3)
        dist_coeffs_list = self.get_parameter('distortion_coeffs').value
        self.dist_coeffs = np.array(dist_coeffs_list)

        # Initialize ArUco detector
        aruco_dict_type = getattr(cv2.aruco, self.aruco_dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info(f'ArUco detector initialized with {self.aruco_dict_name}')
        self.get_logger().info(f'Marker size: {self.marker_size}m')

        # CV Bridge
        self.bridge = CvBridge()

        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_sensor
        )

        # Optional: subscribe to camera_info for calibration
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_sensor
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/aruco_poses',
            qos_reliable
        )

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/aruco/annotated',
                qos_sensor
            )

        # State
        self.camera_info_received = False
        self.last_detection_time = None

        self.get_logger().info(f'ArUco Detector Node initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera calibration from camera_info topic."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera calibration updated from camera_info')

    def image_callback(self, msg: Image):
        """Process incoming image and detect ArUco markers."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            # Debug: Log rejected candidates (potential markers not in dictionary)
            if len(rejected) > 0 and ids is None:
                self.get_logger().debug(f'Found {len(rejected)} rejected candidates - marker may be wrong dictionary type')

            # Create pose array message
            pose_array = PoseArray()
            pose_array.header = msg.header

            if ids is not None and len(ids) > 0:
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                for i, marker_id in enumerate(ids):
                    # Create pose message
                    pose = Pose()
                    
                    # Position (translation vector)
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    # Orientation (convert rotation vector to quaternion)
                    rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                    quat = self.rotation_matrix_to_quaternion(rot_matrix)
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]

                    pose_array.poses.append(pose)

                    self.get_logger().debug(
                        f'Marker {marker_id[0]}: pos=({pose.position.x:.3f}, '
                        f'{pose.position.y:.3f}, {pose.position.z:.3f})'
                    )

            # Publish poses
            self.pose_pub.publish(pose_array)

            # Publish annotated image if enabled
            if self.publish_annotated:
                annotated_frame = cv_image.copy()
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(annotated_frame, corners, ids)
                    
                    # Draw axes for each marker
                    for i in range(len(ids)):
                        cv2.drawFrameAxes(
                            annotated_frame,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvecs[i],
                            tvecs[i],
                            self.marker_size * 0.5
                        )

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)

            # Log detection count
            if ids is not None:
                self.get_logger().debug(f'Detected {len(ids)} ArUco markers: {ids.flatten().tolist()}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            import traceback
            traceback.print_exc()

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert rotation matrix to quaternion."""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArucoDetectorNode()
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
