import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for the simulated drone'
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
    
    ns = LaunchConfiguration('namespace')
    
    # Paths
    tello_gazebo_dir = get_package_share_directory('tello_gazebo')
    world_path = os.path.join(tello_gazebo_dir, 'worlds', 'simple.world')
    
    tello_description_dir = get_package_share_directory('tello_description')
    urdf_path = os.path.join(tello_description_dir, 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        namespace_arg,
        speed_arg,
        yaw_speed_arg,
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),

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
                'speed': LaunchConfiguration('speed'),
                'yaw_speed': LaunchConfiguration('yaw_speed'),
            }],
            prefix='gnome-terminal --'
        ),
    ])
