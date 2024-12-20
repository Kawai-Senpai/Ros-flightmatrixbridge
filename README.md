# ROS Flight Matrix Bridge

This repository contains the ROS2 nodes for interfacing with the Flight Matrix system. It includes publishers for various sensor data and a drone controller for handling joystick inputs.

## Learn More

For more information, visit the [GitHub repository](https://github.com/Kawai-Senpai/Py-FlightMatrix-Bridge).

## What is Flight Matrix?

This is actually a ROS-based API for Flight Matrix Simulation Software.

![FlightMatrix Bridge Cover](https://github.com/Kawai-Senpai/Py-FlightMatrix-Bridge/blob/0d434625dc45a12744eacaa4df21de29f0072612/Assets/FlightMatrix%20Cover.png)

**FlightMatrix Bridge** is a Python-based API designed for controlling and fetching information, frames, and other data from Flight Matrix. This library enables efficient and real-time communication between various processes in a system, primarily designed for interfacing flight simulators, UAV systems, or other robotics platforms. It utilizes the `multiprocessing.shared_memory` module to share data such as frames, sensor data, and movement commands across multiple processes.

Download the software from [Flight Matrix](https://gamejolt.com/games/flightmatrix/933049).

## Launch Files

### flightmatrix.launch.py

This launch file starts the necessary nodes for the Flight Matrix system.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flightmatrix_ros2',
            executable='flightmatrix_publisher',
            name='flightmatrix_publisher',
            output='screen'
        ),
        Node(
            package='flightmatrix_ros2',
            executable='drone_controller',
            name='drone_controller',
            output='screen'
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
  sensor_data: false

  queue_size: 10
  timer_delay: 0.0
```

### Using the Configuration

1. Create a `config` directory inside the `flightmatrix_ros2` package if it doesn't exist.
2. Place your `config.yaml` file inside the `config` directory.
3. Modify the configuration file to enable or disable specific publishers and set the desired parameters.

## Usage

1. Ensure ROS2 is installed and sourced.
2. Place the configuration file in the appropriate directory.
3. Launch the system using the launch file:
   ```sh
   ros2 launch flightmatrix_ros2 flightmatrix.launch.py
   ```

## License

This project is licensed under the MIT License.