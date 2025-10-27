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
    
    drone_ip_arg = DeclareLaunchArgument(
        'drone_ip',
        default_value='192.168.10.1',
        description='IP address of the Tello drone'
    )
    
    # Tello driver node - connects to real drone
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
        prefix='gnome-terminal --' 
    )
    
    return LaunchDescription([
        namespace_arg,
        speed_arg,
        yaw_speed_arg,
        drone_ip_arg,
        tello_driver,
        keyboard_node,
    ])
