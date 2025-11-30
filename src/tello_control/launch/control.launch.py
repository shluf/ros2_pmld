#!/usr/bin/env python3
"""Launch control nodes: mode manager, tracking controller, control arbitrator."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    control_pkg = get_package_share_directory('tello_control')
    config_file = os.path.join(control_pkg, 'config', 'control_config.yaml')

    # Launch arguments
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='joystick',
        description='Initial control mode (manual, gesture, tracking)'
    )

    hover_duration_arg = DeclareLaunchArgument(
        'hover_duration',
        default_value='1.0',
        description='Hover duration before mode switch (seconds)'
    )

    # Nodes
    mode_manager = Node(
        package='tello_control',
        executable='mode_manager_node',
        name='mode_manager',
        output='screen',
        parameters=[{
            'initial_mode': LaunchConfiguration('initial_mode'),
            'hover_duration': LaunchConfiguration('hover_duration'),
            'camera_source': 'drone'
        }]
    )

    tracking_controller = Node(
        package='tello_control',
        executable='tracking_controller_node',
        name='tracking_controller',
        output='screen',
        parameters=[{
            'pid_x.kp': 0.5,
            'pid_x.ki': 0.0,
            'pid_x.kd': 0.1,
            'pid_y.kp': 0.5,
            'pid_y.ki': 0.0,
            'pid_y.kd': 0.1,
            'pid_z.kp': 0.4,
            'pid_z.ki': 0.0,
            'pid_z.kd': 0.08,
            'pid_yaw.kp': 0.3,
            'pid_yaw.ki': 0.0,
            'pid_yaw.kd': 0.05,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 0.5,
            'target_class': 'person',
            'deadzone_pixels': 50,
            'max_tracking_distance': 3.0,
            'frame_width': 960,
            'frame_height': 720
        }]
    )

    control_arbitrator = Node(
        package='tello_control',
        executable='control_arbitrator_node',
        name='control_arbitrator',
        output='screen',
        parameters=[{
            'command_timeout': 0.5,
            'publish_rate': 20.0
        }]
    )

    return LaunchDescription([
        initial_mode_arg,
        hover_duration_arg,
        
        mode_manager,
        tracking_controller,
        control_arbitrator
    ])
