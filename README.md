# ROS Flight Matrix Bridge

A comprehensive ROS2 bridge for interfacing with the Flight Matrix simulation system, providing robust sensor data handling and drone control capabilities.

![FlightMatrix Bridge Cover](https://github.com/Kawai-Senpai/Ros-flightmatrixbridge/blob/25f5746eb33855ee0bda88b32eba03786cbfa19e/Assets/SplashScreenBridge-ROS.png)

## Overview

FlightMatrix Bridge is a sophisticated Python-based API that establishes a seamless connection between ROS2 and the Flight Matrix simulation environment. It provides:

- Real-time sensor data streaming
- High-performance frame capture and processing
- Flexible data replay capabilities
- Multi-format data support (FLIGHTMATRIX, KITTI, VIDEO)
- Advanced synchronization mechanisms
- Configurable publishing options

## Architecture

The bridge implements a modular architecture with the following key components:

1. **Data Providers**:
   - FlightMatrixDataProvider: Direct interface with Flight Matrix
   - KittiDataProvider: KITTI dataset support
   - VideoDataProvider: Video file playback

2. **Publishers**:
   - Image streams (RGB, depth, segmentation)
   - Sensor data (IMU, GPS, magnetometer)
   - Odometry
   - LiDAR data
   - Collision information

3. **Controllers**:
   - Drone control interface
   - Joystick input handling

## Data Types and Formats

### 1. FLIGHTMATRIX Mode
Direct integration with Flight Matrix simulator:

### 2. KITTI Mode
Support for KITTI dataset format:
- Images: sequential PNG files (000000.png, 000001.png, etc.)
- OXTS data: text files with sensor data in KITTI format
- Directory structure must follow KITTI format:
  ```
  dataset/
  ├── image_02/data/      # Left camera images
  ├── image_03/data/      # Right camera images
  └── oxts/data/          # GPS/IMU data
  ```

### 3. VIDEO Mode
Simple video file playback:
- Supports common video formats (mp4, avi, etc.)
- Only publishes left frame data
- No sensor data available

## Configuration

The configuration file (`config_emulator.yaml`) should be adjusted based on your data type:

```yaml
flightmatrix_publisher:
  ros__parameters:
    # Data settings
    data:
      data_directory: "/path/to/your/data"
      data_type: "FLIGHTMATRIX"  # or "KITTI" or "VIDEO"
      loop: true  # whether to loop the data playback

    # Image resolution
    resolution:
      width: 1226
      height: 370

    # Publisher settings
    publishers:
      left_frame: true
      right_frame: false
      left_zdepth: false
      right_zdepth: false
      left_seg: false
      right_seg: false
      sensor_data: false

      queue_size: 10
      timer_delay: 0.0

    # Projection settings (only for KITTI data)
    projection:
      wgs84: "epsg:4326"
      local_proj: "utm"
      local_zone: 33
      local_datum: "WGS84"
```

### Data Type Specific Settings

1. For FLIGHTMATRIX:
```yaml
data:
  data_directory: "/path/to/flightmatrix/data"
  data_type: "FLIGHTMATRIX"
```

2. For KITTI:
```yaml
data:
  data_directory: "/path/to/kitti/dataset"
  data_type: "KITTI"
```

3. For VIDEO:
```yaml
data:
  data_directory: "/path/to/video/file.mp4"
  data_type: "VIDEO"
publishers:
  left_frame: true
  # Other publishers should be false for VIDEO type
```

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
