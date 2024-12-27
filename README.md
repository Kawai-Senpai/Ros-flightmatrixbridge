# ROS Flight Matrix Bridge

This repository contains the ROS2 nodes for interfacing with the Flight Matrix system. It includes publishers for various sensor data and a drone controller for handling joystick inputs.

## Learn More

For more information, visit the [GitHub repository](https://github.com/Kawai-Senpai/Py-FlightMatrix-Bridge).

## What is Flight Matrix?

This is actually a ROS-based API for Flight Matrix Simulation Software.

![FlightMatrix Bridge Cover](assets/SplashScreenBridge-ROS.png)

**FlightMatrix Bridge** is a Python-based API designed for controlling and fetching information, frames, and other data from Flight Matrix. This library enables efficient and real-time communication between various processes in a system, primarily designed for interfacing flight simulators, UAV systems, or other robotics platforms. It utilizes the `multiprocessing.shared_memory` module to share data such as frames, sensor data, and movement commands across multiple processes.

Download the software from [Flight Matrix](https://gamejolt.com/games/flightmatrix/933049).

## Emulator Mode

The FlightMatrix Bridge includes an emulator mode that allows you to replay recorded data without requiring the actual Flight Matrix software. This is useful for testing and development.

### Data Format Requirements

The emulator expects data in the following structure:

```
data_directory/
├── sensor_data.csv
├── left_frames/
│   ├── 0.png
│   ├── 1.png
│   └── ...
├── right_frames/
├── left_zdepth/
├── right_zdepth/
├── left_segmentation/
└── right_segmentation/
```

#### sensor_data.csv Format
The CSV file must contain the following columns:
- timestamp (milliseconds)
- accelerometer_x, accelerometer_y, accelerometer_z (cm/s²)
- gyroscope_x, gyroscope_y, gyroscope_z (degrees/s)
- magnetometer_x, magnetometer_y, magnetometer_z
- location_x, location_y, location_z (cm)
- orientation_roll, orientation_pitch, orientation_yaw (degrees)
- lidar_forward, lidar_backward, lidar_left, lidar_right, lidar_bottom
- collision_status (boolean)
- collision_location_x, collision_location_y, collision_location_z (cm)

#### Image Data
- All image files should be numbered sequentially (0.png, 1.png, etc.)
- Images must match the resolution specified in the config file
- RGB images should be in BGR8 format
- Depth images should be in grayscale

#### Sample Data
- Download the sample data from [Google Drive](https://drive.google.com/drive/folders/1dr1cKaWzNfepxaDq-yNX0AATtniOPe1L)

### Launching the Emulator

1. Place your recorded data in a directory following the structure above
2. Configure the emulator:

```yaml

# config_emulator.yaml
flightmatrix_publisher:
  ros__parameters:

    resolution:
      width: 1226
      height: 370

    publishers:
      left_frame: true
      right_frame: false
      left_zdepth: false
      right_zdepth: false
      left_seg: false
      right_seg: false
      sensor_data: true

      queue_size: 10
      timer_delay: 0.0

```

3. Launch the emulator:

```sh
ros2 launch flightmatrix_ros2 flightmatrix_emulator.launch.py config_file:=/path/to/config_emulator.yaml data_directory:=/path/to/data_directory
```

### Debug Mode

To enable debug logging for synchronization monitoring:

```sh
ros2 param set /flightmatrix_publisher_node debug_sync true
```

## Launch Files

### flightmatrix.launch.py

This launch file starts the necessary nodes for the Flight Matrix system.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file_path = "/path/to/config.yaml"
    config_file = LaunchConfiguration('config_file', default=config_file_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Absolute path to the config file'
        ),
        Node(
            package='flightmatrix_ros2',
            executable='flightmatrix_publisher',
            name='flightmatrix_publisher',
            output='screen',
            parameters=[{'config_file': config_file}]
        ),
        Node(
            package='flightmatrix_ros2',
            executable='drone_controller',
            name='drone_controller',
            output='screen',
            parameters=[{'config_file': config_file}]
        )
    ])
```

### flightmatrix_emulator.launch.py

This launch file starts the FlightMatrix emulator node for replaying recorded data.

```python
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
```

## Nodes

### flightmatrix_publisher.py

This node publishes sensor data from shared memory to ROS2 topics.

- **Published Topics:**
  - `/left_frame` (sensor_msgs/Image)
  - `/right_frame` (sensor_msgs/Image)
  - `/left_zdepth` (sensor_msgs/Image)
  - `/right_zdepth` (sensor_msgs/Image)
  - `/left_seg` (sensor_msgs/Image)
  - `/right_seg` (sensor_msgs/Image)
  - `/odometry` (nav_msgs/Odometry)
  - `/imu/data` (sensor_msgs/Imu)
  - `/imu/mag` (sensor_msgs/MagneticField)
  - `/lidar_data` (std_msgs/Float32MultiArray)
  - `/collision` (geometry_msgs/PoseStamped)

### drone_controller.py

This node handles joystick inputs and writes movement commands to shared memory.

- **Subscribed Topics:**
  - `/joy` (sensor_msgs/Joy)

## Configuration

The configuration file `config.yaml` should be placed in the `config` directory of the `flightmatrix_ros2` package. It contains parameters for publishers and shared memory settings.

### Example Configuration (`config.yaml`)

```yaml
  flightmatrix_publisher:
    ros__parameters:

      resolution:
        width: 1226
        height: 370

      publishers:
        left_frame: true
        right_frame: false
        left_zdepth: false
        right_zdepth: false
        left_seg: false
        right_seg: false
        sensor_data: true

        queue_size: 10
        timer_delay: 0.0

  drone_controller:
    ros__parameters:

      placeholder: 0
```

### Using the Configuration

1. Create a `config` directory inside the `flightmatrix_ros2` package if it doesn't exist.
2. Place your `config.yaml` file inside the `config` directory.
3. Modify the configuration file to enable or disable specific publishers and set the desired parameters.

### Supplying the Configuration File

To supply the configuration file when launching the system, use the `config_file` argument:

```sh
ros2 launch flightmatrix_ros2 flightmatrix.launch.py config_file:=/path/to/your/config.yaml
```

## Usage

1. Ensure ROS2 is installed and sourced.
2. Place the configuration file in the appropriate directory.
3. Launch the system using the launch file:
   ```sh
   ros2 launch flightmatrix_ros2 flightmatrix.launch.py
   ```

## License

This project is licensed under the MIT License.