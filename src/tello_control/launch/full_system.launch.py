#!/usr/bin/env python3
"""Launch complete Tello multi-mode control system."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    perception_pkg = get_package_share_directory('tello_perception')
    control_pkg = get_package_share_directory('tello_control')

    # Launch arguments
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='manual',
        description='Initial control mode (manual, gesture, tracking)'
    )

    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Launch perception nodes (YOLO, ArUco, distance)'
    )

    use_gesture_arg = DeclareLaunchArgument(
        'use_gesture',
        default_value='true',
        description='Launch gesture control node'
    )

    # Include perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        condition=None  # Always launch if use_perception is true
    )

    # Include control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'initial_mode': LaunchConfiguration('initial_mode')
        }.items()
    )

    # Gesture control node
    gesture_control = Node(
        package='gesture_control',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen',
        parameters=[{
            'use_drone_camera': True,
            'debug_mode': False,
            'enable_safety': True,
            'gesture_hold_time': 1.0
        }]
    )

    return LaunchDescription([
        initial_mode_arg,
        use_perception_arg,
        use_gesture_arg,
        perception_launch,
        control_launch,
        gesture_control
    ])
