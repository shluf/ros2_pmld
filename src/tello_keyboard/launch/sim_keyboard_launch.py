"""Launch Gazebo simulation with keyboard controller"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ns = 'drone1'
    
    # Paths
    tello_gazebo_dir = get_package_share_directory('tello_gazebo')
    world_path = os.path.join(tello_gazebo_dir, 'worlds', 'simple.world')
    
    tello_description_dir = get_package_share_directory('tello_description')
    urdf_path = os.path.join(tello_description_dir, 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ], output='screen'),

        # Spawn tello model
        Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, '0', '0', '1', '0']
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
            namespace=ns
        ),

        # Keyboard controller
        Node(
            package='tello_keyboard',
            executable='keyboard_controller',
            name='keyboard_controller',
            output='screen',
            parameters=[{
                'namespace': ns,
                'speed': 0.5,
                'yaw_speed': 0.5,
            }],
            prefix='gnome-terminal --'
        ),
    ])
