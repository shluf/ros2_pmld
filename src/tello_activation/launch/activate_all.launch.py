#!/usr/bin/env python3
"""Launch entire Tello system: Driver, Perception, Control, Joystick."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    tello_control_pkg = get_package_share_directory('tello_control')
    tello_perception_pkg = get_package_share_directory('tello_perception')
    
    # Launch arguments  
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='manual',
        description='Initial control mode'
    )

    # 1. Tello Driver
    tello_driver = Node(
        package='tello_driver',
        executable='tello_driver_main',
        name='tello_driver',
        output='screen',
        parameters=[
            {'tello_ip': '192.168.10.1'},
            {'command_port': 8889},
            {'data_port': 8890},
            {'video_port': 11111},
        ]
    )

    # 2. Perception System
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tello_perception_pkg, 'launch', 'perception.launch.py')
        )
    )

    # 3. Control System (Mode Manager, Arbitrator, Tracking)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tello_control_pkg, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'initial_mode': LaunchConfiguration('initial_mode')
        }.items()
    )

    # 4. Joystick Input
    # joy_node (reads hardware)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # joy_controller (converts joy -> cmd_vel)
    joy_controller = Node(
        package='tello_control',
        executable='joy_controller_node',
        name='joy_controller',
        output='screen'
    )

    # Gesture Detector Node (hand tracking & skeleton)
    # Delayed start to avoid conflicts with other MediaPipe/TensorFlow nodes
    gesture_detector = TimerAction(
        period=3.0,  # Wait 3 seconds for other nodes to initialize
        actions=[
            Node(
                package='gesture_control',
                executable='gesture_detector_node',
                name='gesture_detector',
                output='screen',
                parameters=[
                    {'use_drone_camera': True},
                    {'debug_mode': False},
                    {'webcam_id': 0},
                ]
            )
        ]
    )

    tello_control_gui = Node(
        package='tello_control_gui',
        executable='tello_gui',
        name='tello_control_gui',
        output='screen'
    )

    return LaunchDescription([
        initial_mode_arg,

        tello_driver,
        perception_launch,
        control_launch,
        joy_node,
        joy_controller,
        gesture_detector,
        tello_control_gui
    ])