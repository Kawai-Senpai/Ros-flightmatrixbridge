
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = "/path/to/config_emulator.yaml"
    data_directory_path = "/path/to/record_folder"

    config_file = LaunchConfiguration('config_file', default=config_file_path)
    data_directory = LaunchConfiguration('data_directory', default=data_directory_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Absolute path to the config file'
        ),
        DeclareLaunchArgument(
            'data_directory',
            default_value=data_directory_path,
            description='Absolute path to the data directory'
        ),
        Node(
            package='flightmatrix_ros2',
            executable='flightmatrix_publisher_emulator',
            name='flightmatrix_publisher_node',
            output='screen',
            parameters=[{'config_file': config_file, 'data_directory': data_directory}]
        )
    ])