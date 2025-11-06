#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package directories
    gesture_pkg = get_package_share_directory('gesture_control')
    tello_gazebo_pkg = get_package_share_directory('tello_gazebo')
    
    config_file = os.path.join(gesture_pkg, 'config', 'gesture_mapping.yaml')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for drone topics'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug visualization'
    )
    
    # Launch Gazebo with Tello
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tello_gazebo_pkg, 'launch', 'simple_launch.py')
        )
    )
    
    # Gesture controller for Gazebo
    gesture_controller_node = Node(
        package='gesture_control',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'use_drone_camera': False},  # Use Gazebo camera
            {'debug_mode': LaunchConfiguration('debug_mode')},
            {'enable_safety': False},  # Disable for easier testing
            {'gesture_hold_time': 0.5},  # Shorter for testing
            {'config_file': config_file},
        ],
        remappings=[
            ('cmd_vel', ['/', LaunchConfiguration('namespace'), '/cmd_vel']),
            ('tello_action', ['/', LaunchConfiguration('namespace'), '/tello_action']),
            ('image_raw', ['/', LaunchConfiguration('namespace'), '/image_raw']),
        ]
    )
    
    return LaunchDescription([
        # Arguments
        namespace_arg,
        debug_mode_arg,
        
        # Launch Gazebo first
        gazebo_launch,
        
        # Then gesture controller
        gesture_controller_node,
    ])
