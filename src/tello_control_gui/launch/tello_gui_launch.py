#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace untuk drone topics'
    )
    
    with_driver_arg = DeclareLaunchArgument(
        'with_driver',
        default_value='false',
        description='Launch tello driver juga'
    )
    
    with_gesture_arg = DeclareLaunchArgument(
        'with_gesture',
        default_value='false',
        description='Launch gesture controller juga'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Launch Gazebo simulation'
    )
    
    gui_node = Node(
        package='tello_control_gui',
        executable='tello_gui',
        name='tello_control_gui',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
        ],
        additional_env={'QT_QPA_PLATFORM_PLUGIN_PATH': ''}
    )
    
    tello_driver = Node(
        condition=IfCondition(LaunchConfiguration('with_driver')),
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
    
    gesture_controller = Node(
        condition=IfCondition(LaunchConfiguration('with_gesture')),
        package='gesture_control',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen',
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'enable_safety': True},
        ]
    )
    
    gazebo_sim = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('simulation')),
        cmd=['ros2', 'launch', 'tello_gazebo', 'simple_launch.py'],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        namespace_arg,
        with_driver_arg,
        with_gesture_arg,
        simulation_arg,
        
        # Nodes
        gui_node,
        tello_driver,
        gesture_controller,
        gazebo_sim,
    ])