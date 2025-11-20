#!/usr/bin/env python3
"""Distance estimator node using ArUco markers as reference scale.

Calculates distances between detected objects and ArUco markers using
pixel-to-metric conversion based on known marker size.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from tello_interfaces.msg import DetectionArray, ObjectDistance, ObjectDistanceArray


class DistanceEstimatorNode(Node):
    """ROS 2 node for estimating distances using ArUco marker as scale reference."""

    def __init__(self):
        super().__init__('distance_estimator_node')

        # Parameters
        self.declare_parameter('marker_size', 0.10)  # Marker size in meters
        self.declare_parameter('max_distance_meters', 5.0)  # Maximum valid distance
        self.declare_parameter('min_confidence', 0.3)  # Minimum confidence threshold

        self.marker_size = self.get_parameter('marker_size').value
        self.max_distance = self.get_parameter('max_distance_meters').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera calibration (defaults)
        self.camera_matrix = np.array([[921.0, 0.0, 480.0], [0.0, 921.0, 360.0], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(5)

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

        self.aruco_sub = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_callback,
            qos_reliable
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_sensor
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            qos_sensor
        )

        # Publishers
        self.distance_pub = self.create_publisher(
            ObjectDistanceArray,
            '/object_distances',
            qos_reliable
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            '/distance/annotated',
            qos_sensor
        )

        # State
        self.latest_detections = None
        self.latest_aruco_poses = None
        self.latest_image = None
        self.aruco_marker_corners = []  # Store pixel positions of markers

        self.get_logger().info('Distance Estimator Node initialized')
        self.get_logger().info(f'Marker size: {self.marker_size}m')
        self.get_logger().info(f'Max distance: {self.max_distance}m')

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = msg

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera calibration."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def aruco_callback(self, msg: PoseArray):
        """Store latest ArUco marker poses."""
        self.latest_aruco_poses = msg
        
        # Extract marker pixel positions from poses (z-distance for scale calculation)
        self.aruco_marker_positions = []
        for i, pose in enumerate(msg.poses):
            # Store 3D position for scale calculation
            position = {
                'id': i,  # Marker index
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,  # Distance from camera
            }
            self.aruco_marker_positions.append(position)

        self.get_logger().debug(f'Received {len(msg.poses)} ArUco markers')
        
        # Trigger distance calculation if we have both detections and markers
        if self.latest_detections is not None:
            self.calculate_distances()

    def detection_callback(self, msg: DetectionArray):
        """Store latest detections."""
        self.latest_detections = msg
        
        self.get_logger().debug(f'Received {len(msg.detections)} detections')
        
        # Trigger distance calculation if we have both detections and markers
        if self.latest_aruco_poses is not None:
            self.calculate_distances()

    def calculate_distances(self):
        """Calculate distances between detected objects and ArUco markers."""
        if self.latest_detections is None or self.latest_aruco_poses is None:
            return

        if len(self.latest_detections.detections) == 0:
            # No detections, publish empty array
            distance_array = ObjectDistanceArray()
            distance_array.header = self.latest_detections.header
            self.distance_pub.publish(distance_array)
            return

        if len(self.latest_aruco_poses.poses) == 0:
            self.get_logger().warn('No ArUco markers detected, cannot estimate distances')
            return

        # Create distance array message
        distance_array = ObjectDistanceArray()
        distance_array.header = self.latest_detections.header
        
        # Prepare visualization if image is available
        annotated_img = None
        if self.latest_image is not None:
            try:
                annotated_img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            except Exception as e:
                self.get_logger().warn(f'CV Bridge error: {e}')

        # For each detected object, calculate distance to nearest marker
        for obj_idx, detection in enumerate(self.latest_detections.detections):
            obj_center_x = detection.center_x
            obj_center_y = detection.center_y

            # Find nearest marker and calculate distance
            min_distance_pixels = float('inf')
            nearest_marker_idx = -1
            nearest_marker_pos = None

            for marker_idx, marker_pos in enumerate(self.aruco_marker_positions):
                # We need marker pixel position - use projection from 3D pose
                # For simplicity, we'll use the pose z-distance for scale
                # In practice, you'd project 3D pose back to image plane
                
                # Project 3D point to 2D
                try:
                    point_3d = np.array([[marker_pos['x'], marker_pos['y'], marker_pos['z']]])
                    point_2d, _ = cv2.projectPoints(
                        point_3d, 
                        np.zeros(3), np.zeros(3), # Identity rotation/translation (pose is already in camera frame)
                        self.camera_matrix, 
                        self.dist_coeffs
                    )
                    marker_px_x = point_2d[0][0][0]
                    marker_px_y = point_2d[0][0][1]
                    
                    # Update marker position with projected pixels
                    marker_pos['px_x'] = marker_px_x
                    marker_pos['px_y'] = marker_px_y
                    
                except Exception as e:
                    # Fallback if projection fails
                    marker_pos['px_x'] = 0
                    marker_pos['px_y'] = 0
                
            # Since we need pixel positions of markers, let's use a simplified approach:
            # Calculate distance using the first (closest) marker's scale
            if len(self.aruco_marker_positions) > 0:
                # Use first marker for scale reference
                marker = self.aruco_marker_positions[0]
                marker_z = marker['z']  # Distance in meters
                
                # Estimate pixel-to-meter scale
                # Marker appears smaller when farther away
                # pixels_per_meter depends on focal length and distance
                pixels_per_meter = self.estimate_pixels_per_meter(marker_z)
                
                # Use projected pixel coordinates if available
                marker_px_x = marker.get('px_x', 0.0)
                marker_px_y = marker.get('px_y', 0.0)
                
                # Calculate distance in pixels
                dist_px = self.calculate_pixel_distance(
                    obj_center_x, obj_center_y, 
                    marker_px_x, marker_px_y
                )
                
                # Calculate distance in meters
                dist_m = 0.0
                if pixels_per_meter > 0:
                    dist_m = dist_px / pixels_per_meter
                
                # Create distance measurement
                obj_distance = ObjectDistance()
                obj_distance.object_id = obj_idx
                obj_distance.object_class = detection.class_name
                obj_distance.object_center_x = obj_center_x
                obj_distance.object_center_y = obj_center_y
                
                obj_distance.marker_id = 0  # Using first marker
                obj_distance.marker_center_x = float(marker_px_x)
                obj_distance.marker_center_y = float(marker_px_y)
                
                obj_distance.distance_pixels = dist_px
                obj_distance.distance_meters = dist_m
                obj_distance.confidence = 0.8
                
                distance_array.distances.append(obj_distance)
                
                # Visualization
                if annotated_img is not None:
                    # Draw line
                    pt1 = (int(obj_center_x), int(obj_center_y))
                    pt2 = (int(marker_px_x), int(marker_px_y))
                    cv2.line(annotated_img, pt1, pt2, (0, 255, 255), 2)
                    
                    # Draw distance text
                    mid_x = int((pt1[0] + pt2[0]) / 2)
                    mid_y = int((pt1[1] + pt2[1]) / 2)
                    text = f"{dist_m*100:.1f}cm"
                    cv2.putText(annotated_img, text, (mid_x, mid_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # Draw object center
                    cv2.circle(annotated_img, pt1, 5, (0, 0, 255), -1)
                    # Draw marker center
                    cv2.circle(annotated_img, pt2, 5, (0, 255, 0), -1)

        # Publish distances
        self.distance_pub.publish(distance_array)
        
        # Publish annotated image
        if annotated_img is not None:
            try:
                msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
                msg.header = self.latest_detections.header
                self.annotated_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Error publishing annotated image: {e}')
                
        self.get_logger().debug(f'Published {len(distance_array.distances)} distance measurements')

    def estimate_pixels_per_meter(self, distance_meters):
        """Estimate pixel-to-meter scale at given distance.
        
        Uses pinhole camera model: pixels_per_meter = focal_length / distance
        Assumes focal length ~921 pixels (Tello approximate).
        """
        focal_length = 921.0  # pixels (from camera calibration)
        if distance_meters > 0:
            pixels_per_meter = focal_length * self.marker_size / distance_meters
            return pixels_per_meter
        return 0.0

    def calculate_pixel_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points in pixels."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DistanceEstimatorNode()
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
