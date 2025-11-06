#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for the drone'
    )
    
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.5',
        description='Linear speed (0.0 - 1.0)'
    )
    
    yaw_speed_arg = DeclareLaunchArgument(
        'yaw_speed',
        default_value='0.5',
        description='Angular/yaw speed (0.0 - 1.0)'
    )
    
    # Keyboard controller node
    keyboard_node = Node(
        package='tello_keyboard',
        executable='keyboard_controller',
        name='keyboard_controller',
        output='screen',
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
            'speed': LaunchConfiguration('speed'),
            'yaw_speed': LaunchConfiguration('yaw_speed'),
        }],
        prefix='gnome-terminal --'  # Jalankan di terminal terpisah untuk input keyboard
    )
    
    return LaunchDescription([
        namespace_arg,
        speed_arg,
        yaw_speed_arg,
        keyboard_node,
    ])
