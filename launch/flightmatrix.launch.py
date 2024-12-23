import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('flightmatrix_ros2')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(share_dir, 'config', 'config.yaml'),
            description='Absolute path to the config file'
        ),
        Node(
            package='flightmatrix_ros2',
            executable='flightmatrix_publisher',
            name='flightmatrix_publisher',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='flightmatrix_ros2',
            executable='drone_controller',
            name='drone_controller',
            output='screen'
        )
    ])
