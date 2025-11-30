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
    
    # Gesture Detector (Perception)
    gesture_detector_node = Node(
        package='gesture_control',
        executable='gesture_detector_node',
        name='gesture_detector',
        output='screen',
        parameters=[
            {'use_drone_camera': False},
            {'debug_mode': LaunchConfiguration('debug_mode')},
        ],
        remappings=[
            ('image_raw', ['/', LaunchConfiguration('namespace'), '/image_raw']),
        ]
    )

    # Gesture Control (Action)
    gesture_control_node = Node(
        package='tello_control',
        executable='gesture_control_node',
        name='gesture_control',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'enable_safety': False},
            {'gesture_hold_time': 0.5},
            {'config_file': 'config/gesture_mapping.yaml'},
        ],
        remappings=[
            ('cmd_vel', ['/', LaunchConfiguration('namespace'), '/cmd_vel']),
            ('tello_action', ['/', LaunchConfiguration('namespace'), '/tello_action']),
        ]
    )
    
    return LaunchDescription([
        # Arguments
        namespace_arg,
        debug_mode_arg,
        
        # Launch Gazebo first
        gazebo_launch,
        
        # Then gesture nodes
        gesture_detector_node,
        gesture_control_node,
    ])
