from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('diff_vesc_can_ros2_pkg_cpp'),
        'config',
        'default_robot_params.yaml'
    )
    
    return LaunchDescription([
        # Launch arguments for robot parameters
        DeclareLaunchArgument(
            'wheel_diameter',
            default_value='0.3556',
            description='Wheel diameter in meters'
        ),
        
        DeclareLaunchArgument(
            'wheel_separation',
            default_value='0.370',
            description='Distance between wheel centers in meters'
        ),
        
        DeclareLaunchArgument(
            'tachometer_pulses_per_rev',
            default_value='23',
            description='Tachometer pulses per wheel revolution'
        ),
        
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Odometry publish rate in Hz'
        ),
        
        DeclareLaunchArgument(
            'left_vesc_id',
            default_value='28',
            description='Left wheel VESC controller ID'
        ),
        
        DeclareLaunchArgument(
            'right_vesc_id',
            default_value='46',
            description='Right wheel VESC controller ID'
        ),
        
        # VESC Odometry Node
        Node(
            package='diff_vesc_can_ros2_pkg_cpp',
            executable='vesc_odometry_node',
            name='vesc_odometry_node',
            parameters=[
                config_file,
                {
                    'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                    'wheel_separation': LaunchConfiguration('wheel_separation'),
                    'tachometer_pulses_per_rev': LaunchConfiguration('tachometer_pulses_per_rev'),
                    'can_interface': LaunchConfiguration('can_interface'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'left_vesc_id': LaunchConfiguration('left_vesc_id'),
                    'right_vesc_id': LaunchConfiguration('right_vesc_id')
                }
            ],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2
        )
    ])
