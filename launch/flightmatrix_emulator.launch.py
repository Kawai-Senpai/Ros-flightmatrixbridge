
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_share_directory = get_package_share_directory('flightmatrix_ros2')
    config_file_path = os.path.join(package_share_directory, 'config', 'config_emulator.yaml')

    config_file_path = LaunchConfiguration('config_file', default=config_file_path)
    config_file = LaunchConfiguration('config_file', default=config_file_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Absolute path to the config file'
        ),
        Node(
            package='flightmatrix_ros2',
            executable='flightmatrix_publisher_emulator',
            name='flightmatrix_publisher_node',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    ])