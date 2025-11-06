#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package directory
    pkg_dir = get_package_share_directory('gesture_control')
    config_file = os.path.join(pkg_dir, 'config', 'gesture_mapping.yaml')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace untuk drone topics'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (deprecated, using drone camera now)'
    )
    
    use_drone_camera_arg = DeclareLaunchArgument(
        'use_drone_camera',
        default_value='true',
        description='Use drone camera feed instead of webcam'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug mode (shows detection window, extra logging)'
    )
    
    webcam_id_arg = DeclareLaunchArgument(
        'webcam_id',
        default_value='0',
        description='Webcam ID for debug mode (when use_drone_camera=false)'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety features (gesture hold time, confirmations)'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (Gazebo)'
    )
    
    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='true',
        description='Show camera feed with gesture detection'
    )
    
    # Gesture controller node
    gesture_controller_node = Node(
        package='gesture_control',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'use_drone_camera': LaunchConfiguration('use_drone_camera')},
            {'debug_mode': LaunchConfiguration('debug_mode')},
            {'webcam_id': LaunchConfiguration('webcam_id')},
            {'enable_safety': LaunchConfiguration('enable_safety')},
            {'config_file': config_file},
        ],
        remappings=[
            ('cmd_vel', ['/cmd_vel']),
            ('tello_action', ['/tello_action']),
            ('image_raw', ['/image_raw']),
        ]
    )
    
    # Optional: Launch Gazebo simulation
    gazebo_launch = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('simulation')),
        cmd=['ros2', 'launch', 'tello_gazebo', 'simple_launch.py'],
        output='screen'
    )
    
    # Optional: Launch RViz for visualization
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('simulation')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'gesture_control.rviz')]
    )
    
    return LaunchDescription([
        # Arguments
        namespace_arg,
        camera_id_arg,
        use_drone_camera_arg,
        debug_mode_arg,
        webcam_id_arg,
        enable_safety_arg,
        simulation_arg,
        show_camera_arg,
        
        # Nodes
        gesture_controller_node,
        gazebo_launch,
        # rviz_node,  # Uncomment jika ingin RViz
    ])
