from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the drone (leave empty for single drone)'
    )
    
    drone_ip_arg = DeclareLaunchArgument(
        'drone_ip',
        default_value='192.168.10.1',
        description='IP address of the Tello drone'
    )
    
    # Tello driver node
    tello_driver = Node(
        package='tello_driver',
        executable='tello_driver_main',
        name='tello_driver',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'drone_ip': LaunchConfiguration('drone_ip'),
        }]
    )
    
    return LaunchDescription([
        namespace_arg,
        drone_ip_arg,
        tello_driver,
    ])
