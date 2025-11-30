#!/usr/bin/env python3
"""YOLO object detector node for Tello drone.

Subscribes to camera image and publishes detected objects.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from tello_interfaces.msg import Detection, DetectionArray, ControlMode
from tello_interfaces.srv import SetNodeActive


class YOLODetectorNode(Node):
    """ROS 2 node for YOLO-based object detection."""

    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.declare_parameter('model_name', 'yolov8n')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('target_classes', ['all'])
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('publish_annotated', True)

        self.model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        self.target_classes = self.get_parameter('target_classes').value
        self.image_topic = self.get_parameter('image_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value

        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not installed! Install with: pip install ultralytics')
            raise ImportError('ultralytics package required')

        # Initialize YOLO model
        self.get_logger().info(f'Loading {self.model_name} on {self.device}...')
        self.model = YOLO(f'{self.model_name}.pt')
        self.model.to(self.device)
        self.get_logger().info('YOLO model loaded successfully')

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

        # Publishers
        self.detection_pub = self.create_publisher(
            DetectionArray,
            '/detections',
            qos_reliable
        )

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/detections/annotated',
                qos_sensor
            )
        
        # Service for enabling/disabling node processing
        self.active_srv = self.create_service(
            SetNodeActive,
            '/yolo/set_active',
            self.set_active_callback
        )
        
        # Subscribe to control mode to auto-pause when not in tracking mode
        self.mode_sub = self.create_subscription(
            ControlMode,
            '/control_mode',
            self.mode_callback,
            qos_reliable
        )
        
        # Processing state - can be paused to save CPU
        self.processing_active = True
        self.current_mode = 'manual'

        self.get_logger().info(f'YOLO Detector Node initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Target classes: {self.target_classes if self.target_classes else "all"}')

    def mode_callback(self, msg: ControlMode):
        """Auto-pause/resume based on control mode."""
        old_mode = self.current_mode
        self.current_mode = msg.mode
        
        # YOLO is needed for tracking mode
        if msg.mode == 'tracking' and not self.processing_active:
            self.processing_active = True
            self.get_logger().info('Tracking mode active - YOLO processing resumed')
        # Pause when leaving tracking mode (optional - can keep running for visualization)
        elif msg.mode != 'tracking' and old_mode == 'tracking':
            # Don't auto-pause YOLO, it may be useful for visualization
            pass

    def set_active_callback(self, request, response):
        """Handle node active/pause service requests."""
        self.processing_active = request.active
        status = "enabled" if request.active else "paused"
        response.success = True
        response.message = f"YOLO detector processing {status}"
        self.get_logger().info(response.message)
        return response

    def image_callback(self, msg: Image):
        """Process incoming image and detect objects."""
        # Skip processing if paused
        if not self.processing_active:
            return
            
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO detection
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

            # Create detection array message
            detection_array = DetectionArray()
            detection_array.header = msg.header

            # Process results
            if len(results) > 0:
                result = results[0]  # Single image
                boxes = result.boxes

                for box in boxes:
                    # Get box data
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = float(box.conf[0])

                    # Filter by target classes if specified
                    if self.target_classes and class_name not in self.target_classes:
                        continue

                    # Get bounding box coordinates (xyxy format)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    # Convert to xywh format
                    x = int(x1)
                    y = int(y1)
                    width = int(x2 - x1)
                    height = int(y2 - y1)

                    # Calculate center
                    center_x = x + width / 2.0
                    center_y = y + height / 2.0

                    # Create Detection message
                    detection = Detection()
                    detection.x = x
                    detection.y = y
                    detection.width = width
                    detection.height = height
                    detection.class_id = class_id
                    detection.class_name = class_name
                    detection.confidence = confidence
                    detection.center_x = center_x
                    detection.center_y = center_y

                    detection_array.detections.append(detection)

            # Publish detections
            self.detection_pub.publish(detection_array)

            # Publish annotated image if enabled
            if self.publish_annotated and len(results) > 0:
                annotated_frame = results[0].plot()  # Draw boxes on image
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)

            # Log detections
            if len(detection_array.detections) > 0:
                self.get_logger().debug(
                    f'Detected {len(detection_array.detections)} objects: '
                    f'{[d.class_name for d in detection_array.detections]}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YOLODetectorNode()
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
