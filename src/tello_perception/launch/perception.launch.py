#!/usr/bin/env python3
"""Launch perception nodes: YOLO detector, ArUco detector, distance estimator."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    perception_pkg = get_package_share_directory('tello_perception')
    config_file = os.path.join(perception_pkg, 'config', 'perception_config.yaml')

    # Launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolov8n',
        description='YOLO model name (yolov8n, yolov8s, etc.)'
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.10',
        description='ArUco marker size in meters'
    )

    # Nodes
    yolo_detector = Node(
        package='tello_perception',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': 'cpu',
            'target_classes': ['person', 'bottle', 'cup'],
            'publish_annotated': True
        }]
    )

    aruco_detector = Node(
        package='tello_perception',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dict': 'DICT_4X4_50',
            'publish_annotated': True
        }]
    )

    distance_estimator = Node(
        package='tello_perception',
        executable='distance_estimator_node',
        name='distance_estimator',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'max_distance_meters': 5.0,
            'min_confidence': 0.3
        }]
    )

    return LaunchDescription([
        model_name_arg,
        confidence_arg,
        marker_size_arg,
        yolo_detector,
        aruco_detector,
        distance_estimator
    ])
