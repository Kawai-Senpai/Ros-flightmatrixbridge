import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
import transforms3d
import math
import numpy as np
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from multiprocessing import shared_memory
import struct

class FlightMatrixPublisher(Node):
    
    def __init__(self):
        super().__init__('flightmatrix_publisher')
        
        config_file = os.path.join(
            get_package_share_directory('flightmatrix_ros2'),
            'config',
            'config.yaml'
        )
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_file}")
            return
        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error parsing config file: {exc}")
            return

        self.bridge = CvBridge()

        # Shared memory names
        self.memory_names = {
            'right_frame': 'RightFrame',
            'left_frame': 'LeftFrame',
            'right_zdepth': 'RightZFrame',
            'left_zdepth': 'LeftZFrame',
            'right_seg': 'RightSegFrame',
            'left_seg': 'LeftSegFrame',
            'sensor_data': 'SensorData'
        }
        self.shm = {} # To store shared memory blocks
        self.shm_timestamps = {}  # To store shared memory blocks for timestamps

        # Load the resolution parameters from the config file
        self.width = config['resolution']['width']
        self.height = config['resolution']['height']
        self.channel3 = ((self.height, self.width, 3), self.width * self.height * 3)
        self.channel1 = ((self.height, self.width), self.width * self.height)
        
        if config['publishers']['left_frame']:
            self._create_sharedmemory_object('left_frame', self.memory_names['left_frame'], create=False)
            self.left_frame_pub = self.create_publisher(Image, 'left_frame', config['publishers']['queue_size'])
            self.left_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_frame_cb)

        if config['publishers']['right_frame']:
            self._create_sharedmemory_object('right_frame', self.memory_names['right_frame'], create=False)
            self.right_frame_pub = self.create_publisher(Image, 'right_frame', config['publishers']['queue_size'])
            self.right_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_frame_cb)

        if config['publishers']['left_zdepth']:
            self._create_sharedmemory_object('left_zdepth', self.memory_names['left_zdepth'], create=False)
            self.left_zdepth_pub = self.create_publisher(Image, 'left_zdepth', config['publishers']['queue_size'])
            self.left_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_zdepth_cb)

        if config['publishers']['right_zdepth']:
            self._create_sharedmemory_object('right_zdepth', self.memory_names['right_zdepth'], create=False)
            self.right_zdepth_pub = self.create_publisher(Image, 'right_zdepth', config['publishers']['queue_size'])
            self.right_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_zdepth_cb)

        if config['publishers']['left_seg']:
            self._create_sharedmemory_object('left_seg', self.memory_names['left_seg'], create=False)
            self.left_seg_pub = self.create_publisher(Image, 'left_seg', config['publishers']['queue_size'])
            self.left_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_seg_cb)

        if config['publishers']['right_seg']:
            self._create_sharedmemory_object('right_seg', self.memory_names['right_seg'], create=False)
            self.right_seg_pub = self.create_publisher(Image, 'right_seg', config['publishers']['queue_size'])
            self.right_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_seg_cb)

        if config['publishers']['sensor_data']:
            self._create_sharedmemory_object('sensor_data', self.memory_names['sensor_data'], create=False)
            self.odom_pub = self.create_publisher(Odometry, 'odometry', config['publishers']['queue_size'])
            self.imu_pub = self.create_publisher(Imu, 'imu/data', config['publishers']['queue_size'])
            self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', config['publishers']['queue_size'])
            self.lidar_pub = self.create_publisher(Float32MultiArray, 'lidar_data', config['publishers']['queue_size'])
            self.collision_pub = self.create_publisher(PoseStamped, 'collision', config['publishers']['queue_size'])
            self.sensor_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_sensor_data)
    
    def _create_sharedmemory_object(self, key, name, create=False):
        try:
            if create:
                self.shm[key] = shared_memory.SharedMemory(name=name, create=True)
                self.shm_timestamps[key] = shared_memory.SharedMemory(name=name+'_time', create=True)
            else:
                self.shm[key] = shared_memory.SharedMemory(name=name, create=False)
                self.shm_timestamps[key] = shared_memory.SharedMemory(name=name+'_time', create=False)
        except FileNotFoundError:
            self.get_logger().error(f"Shared memory block not found: {name}")
        except Exception as e:
            self.get_logger().error(f"Error accessing shared memory: {e}")

    def _get_frame(self, key, rgb=True):
        try:
            frame = np.ndarray(self.channel3[0] if rgb else self.channel1[0], dtype=np.uint8, buffer=self.shm[key].buf[:self.channel3[1]] if rgb else self.shm[key].buf[:self.channel1[1]])
            timestamp = self._get_timestamp(key)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8' if rgb else 'mono8')
            msg.header.stamp = timestamp
            return msg
        except KeyError:
            self.get_logger().error(f"Shared memory key not found: {key}")
            return None

    def _get_sensor_data(self):
        try:
            sensor_array = np.ndarray((24,), dtype=np.float32, buffer=self.shm['sensor_data'].buf[:96])
            timestamp = self._get_timestamp('sensor_data')
            return sensor_array, timestamp
        except KeyError:
            self.get_logger().error("Shared memory key 'sensor_data' not found")
            return None, None

    def _get_timestamp(self, key):
        try:
            timestamp_bytes = self.shm_timestamps[key].buf[:8].tobytes()
            timestamp = self.convert_timestamp(struct.unpack('q', timestamp_bytes)[0])  # 'q' is the format for long long (8 bytes)
            return timestamp
        except KeyError:
            self.get_logger().error(f"Timestamp shared memory key not found: {key}")
            return Time()

    def convert_timestamp(self, timestamp_ms):
        """Convert millisecond timestamp to ROS Time message"""
        seconds = timestamp_ms // 1000
        nanoseconds = (timestamp_ms % 1000) * 1000000
        time_msg = Time()
        time_msg.sec = int(seconds)
        time_msg.nanosec = int(nanoseconds)
        return time_msg

    def publish_left_frame_cb(self):
        msg = self._get_frame('left_frame', rgb=True)
        self.left_frame_pub.publish(msg)

    def publish_right_frame_cb(self):
        msg = self._get_frame('right_frame', rgb=True)
        self.right_frame_pub.publish(msg)

    def publish_left_zdepth_cb(self):
        msg = self._get_frame('left_zdepth', rgb=False)
        self.left_zdepth_pub.publish(msg)

    def publish_right_zdepth_cb(self):
        msg = self._get_frame('right_zdepth', rgb=False)
        self.right_zdepth_pub.publish(msg)

    def publish_left_seg_cb(self):
        msg = self._get_frame('left_seg', rgb=True)
        self.left_seg_pub.publish(msg)

    def publish_right_seg_cb(self):
        msg = self._get_frame('right_seg', rgb=True)
        self.right_seg_pub.publish(msg)

    def publish_sensor_data(self):
        
        sensor_array, timestamp = self._get_sensor_data()
        if sensor_array is None or timestamp is None:
            return

        # Publish IMU data (accelerometer and gyroscope)
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = 'base_link'
        
        # Convert to m/s² (from cm/s²)
        accel = sensor_array[9:12] / 100.0
        imu_msg.linear_acceleration.x = accel[0] 
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        
        # Convert to radians/s (from degrees/s)
        gyro = sensor_array[6:9]
        imu_msg.angular_velocity.x = math.radians(gyro[0])
        imu_msg.angular_velocity.y = math.radians(gyro[1])
        imu_msg.angular_velocity.z = math.radians(gyro[2])
        
        self.imu_pub.publish(imu_msg)
        
        # Publish magnetometer data
        mag_msg = MagneticField()
        mag_msg.header.stamp = timestamp
        mag_msg.header.frame_id = 'base_link'
        mag = sensor_array[12:15]
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]
        self.mag_pub.publish(mag_msg)
        
        # Publish odometry (location and orientation)
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Convert position to meters (from cm)
        loc = sensor_array[:3] / 100.0
        odom_msg.pose.pose.position.x = loc[0] 
        odom_msg.pose.pose.position.y = loc[1]
        odom_msg.pose.pose.position.z = loc[2] 
        
        # Convert orientation to quaternion (from euler degrees)
        orient = sensor_array[3:6]
        q = transforms3d.euler.euler2quat(
            math.radians(orient[0]),  # roll
            math.radians(orient[1]),  # pitch
            math.radians(orient[2])   # yaw
        )
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom_msg)
        
        # Publish LiDAR data
        lidar_msg = Float32MultiArray()
        lidar_msg.data = sensor_array[15:20]  # Keep in cm
        self.lidar_pub.publish(lidar_msg)
        
        # Publish collision data if collision detected
        if sensor_array[20]:  # If collision status is True
            collision_msg = PoseStamped()
            collision_msg.header.stamp = timestamp
            collision_msg.header.frame_id = 'base_link'
            # Convert collision location to meters (from cm)
            collision_msg.pose.position.x = sensor_array[21] / 100.0
            collision_msg.pose.position.y = sensor_array[22] / 100.0
            collision_msg.pose.position.z = sensor_array[23] / 100.0
            self.collision_pub.publish(collision_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightMatrixPublisher()
    if node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()
