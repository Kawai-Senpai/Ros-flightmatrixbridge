import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray
import transforms3d
import math
from cv_bridge import CvBridge
import os
import yaml
import pandas as pd
import cv2

class FlightMatrixPublisher(Node):
    def __init__(self):
        super().__init__('flightmatrix_publisher')
        self._initialize_parameters()
        self._load_config()
        self._initialize_publishers()
        self._initialize_timers()
        self._initialize_data()
        self._check_directories_and_resolution()

    def _initialize_parameters(self):
        self.declare_parameter('config_file', '')
        self.declare_parameter('data_directory', '')
        self.declare_parameter('debug_sync', False)
        self.debug_sync = self.get_parameter('debug_sync').get_parameter_value().bool_value

    def _load_config(self):
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_file:
            self.get_logger().error("Config file path not provided")
            return

        try:
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)['flightmatrix_publisher']['ros__parameters']
                self.get_logger().info(f"Config file loaded: {config_file}")
        except (FileNotFoundError, yaml.YAMLError) as exc:
            self.get_logger().error(f"Error loading config file: {exc}")
            return

    def _initialize_publishers(self):
        self.publish_flags = {
            publisher: False 
            for publisher, enabled in self.config['publishers'].items() 
            if enabled and publisher not in ['queue_size', 'timer_delay']
        }

        self.bridge = CvBridge()
        self.width = self.config['resolution']['width']
        self.height = self.config['resolution']['height']

        self._create_publishers()

    def _create_publishers(self):
        publishers = self.config['publishers']
        queue_size = publishers['queue_size']
        timer_delay = publishers['timer_delay']

        if publishers['left_frame']:
            self.left_frame_pub = self.create_publisher(Image, 'left_frame', queue_size)
            self.left_frame_timer = self.create_timer(timer_delay, self.publish_left_frame_cb)

        if publishers['right_frame']:
            self.right_frame_pub = self.create_publisher(Image, 'right_frame', queue_size)
            self.right_frame_timer = self.create_timer(timer_delay, self.publish_right_frame_cb)

        if publishers['left_zdepth']:
            self.left_zdepth_pub = self.create_publisher(Image, 'left_zdepth', queue_size)
            self.left_zdepth_timer = self.create_timer(timer_delay, self.publish_left_zdepth_cb)

        if publishers['right_zdepth']:
            self.right_zdepth_pub = self.create_publisher(Image, 'right_zdepth', queue_size)
            self.right_zdepth_timer = self.create_timer(timer_delay, self.publish_right_zdepth_cb)

        if publishers['left_seg']:
            self.left_seg_pub = self.create_publisher(Image, 'left_seg', queue_size)
            self.left_seg_timer = self.create_timer(timer_delay, self.publish_left_seg_cb)

        if publishers['right_seg']:
            self.right_seg_pub = self.create_publisher(Image, 'right_seg', queue_size)
            self.right_seg_timer = self.create_timer(timer_delay, self.publish_right_seg_cb)

        if publishers['sensor_data']:
            self._create_sensor_publishers()

    def _create_sensor_publishers(self):
        queue_size = self.config['publishers']['queue_size']
        self.odom_pub = self.create_publisher(Odometry, 'odometry', queue_size)
        self.sensor_timer = self.create_timer(self.config['publishers']['timer_delay'], self.publish_sensor_data)

        if self.data_type == "FLIGHTMATRIX":
            self.imu_pub = self.create_publisher(Imu, 'imu/data', queue_size)
            self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', queue_size)
            self.lidar_pub = self.create_publisher(Float32MultiArray, 'lidar_data', queue_size)
            self.collision_pub = self.create_publisher(PoseStamped, 'collision', queue_size)

    def _initialize_timers(self):
        self.frame_increment_timer = self.create_timer(
            self.config['publishers']['timer_delay'], 
            self._check_and_increment_frame
        )

    def _initialize_data(self):
        self.data_type = self.config['data']['data_type']
        self.data_dir = self.config['data']['data_directory']
        self.frame_index = 0
        self._current_sensor_data = None
        self._current_timestamp = None

        if self.data_type == "FLIGHTMATRIX":
            self._load_flightmatrix_data()
        elif self.data_type == "KITTI":
            self._load_kitti_data()
        else:
            self.get_logger().error(f"Unsupported data type: {self.data_type}")

    def _load_flightmatrix_data(self):
        sensor_data_path = os.path.join(self.data_dir, 'sensor_data.csv')
        if not os.path.exists(sensor_data_path):
            self.get_logger().error(f"Sensor data file not found: {sensor_data_path}")
            return
        self.sensor_data = pd.read_csv(sensor_data_path)
        self.get_logger().info(f"Sensor data loaded: {sensor_data_path}")

    def _load_kitti_data(self):
        oxts_data_path = os.path.join(self.data_dir, 'oxts', 'data')
        if not os.path.exists(oxts_data_path):
            self.get_logger().error(f"OXTS data directory not found: {oxts_data_path}")
            return
        self.sensor_data = sorted([os.path.join(oxts_data_path, f) for f in os.listdir(oxts_data_path) if f.endswith('.txt')])
        self.get_logger().info(f"OXTS data loaded: {oxts_data_path}")

    def _check_directories_and_resolution(self):
        required_dirs = self._get_required_directories()
        for dir_name in required_dirs:
            dir_path = os.path.join(self.data_dir, dir_name)
            if not os.path.exists(dir_path):
                self.get_logger().error(f"Required directory not found: {dir_path}")
                return

            first_image_path = os.path.join(dir_path, '0.png')
            if not os.path.exists(first_image_path):
                self.get_logger().error(f"First image not found in directory: {dir_path}")
                return

            image = cv2.imread(first_image_path)
            if image is None:
                self.get_logger().error(f"Error reading image: {first_image_path}")
                return

            if image.shape[1] != self.width or image.shape[0] != self.height:
                self.get_logger().error(f"Resolution mismatch in {first_image_path}: expected ({self.width}, {self.height}), got ({image.shape[1]}, {image.shape[0]})")
                return

    def _get_required_directories(self):
        required_dirs = []
        publishers = self.config['publishers']
        if publishers['left_frame']:
            required_dirs.append('left_frames' if self.data_type == "FLIGHTMATRIX" else 'image_00')
        if publishers['right_frame']:
            required_dirs.append('right_frames' if self.data_type == "FLIGHTMATRIX" else 'image_01')
        if publishers['left_zdepth']:
            required_dirs.append('left_zdepth')
        if publishers['right_zdepth']:
            required_dirs.append('right_zdepth')
        if publishers['left_seg']:
            required_dirs.append('left_segmentation')
        if publishers['right_seg']:
            required_dirs.append('right_segmentation')
        return required_dirs

    def _check_and_increment_frame(self):
        if all(self.publish_flags.values()):
            self.frame_index = (self.frame_index + 1) % len(self.sensor_data)
            if self.debug_sync:
                self.get_logger().debug(f"Moving to frame {self.frame_index}")
            self.publish_flags = {k: False for k in self.publish_flags}
            self._current_sensor_data = None
            self._current_timestamp = None

    def _get_sensor_data(self):
        if self._current_sensor_data is None:
            try:
                if self.data_type == "KITTI":
                    with open(self.sensor_data[self.frame_index], 'r') as file:
                        data = file.read().strip().split()
                        self._current_sensor_data = [float(x) for x in data]
                else:
                    self._current_sensor_data = self.sensor_data.iloc[self.frame_index]
                self._current_timestamp = self.get_clock().now().to_msg()
            except IndexError:
                self.get_logger().error(f"Frame index {self.frame_index} out of range")
                return None, None
        return self._current_sensor_data, self._current_timestamp

    def _get_frame(self, frame_type, rgb=True):
        try:
            mapped_folder = self.folder_mapping[self.data_type].get(frame_type)
            if not mapped_folder:
                self.get_logger().error(f"Unknown frame type: {frame_type}")
                return None

            frame_path = os.path.join(self.data_dir, mapped_folder, f"{self.frame_index:06d}.png")
            frame = cv2.imread(frame_path, cv2.IMREAD_COLOR if rgb else cv2.IMREAD_GRAYSCALE)
            if frame is None:
                self.get_logger().error(f"Frame not found: {frame_path}")
                return None

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8' if rgb else 'mono8')
            msg.header.stamp = self.get_clock().now().to_msg()
            return msg
        except Exception as e:
            self.get_logger().error(f"Error reading frame: {e}")
            return None

    def publish_left_frame_cb(self):
        self._publish_frame('left_frame', 'left_frames', rgb=True)

    def publish_right_frame_cb(self):
        self._publish_frame('right_frame', 'right_frames', rgb=True)

    def publish_left_zdepth_cb(self):
        self._publish_frame('left_zdepth', 'left_zdepth', rgb=False)

    def publish_right_zdepth_cb(self):
        self._publish_frame('right_zdepth', 'right_zdepth', rgb=False)

    def publish_left_seg_cb(self):
        self._publish_frame('left_seg', 'left_segmentation', rgb=True)

    def publish_right_seg_cb(self):
        self._publish_frame('right_seg', 'right_segmentation', rgb=True)

    def _publish_frame(self, flag_key, frame_type, rgb):
        if not self.publish_flags[flag_key]:
            msg = self._get_frame(frame_type, rgb)
            if msg:
                getattr(self, f"{flag_key}_pub").publish(msg)
                self.publish_flags[flag_key] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published {flag_key} {self.frame_index}")

    def publish_sensor_data(self):
        if not self.publish_flags['sensor_data']:
            sensor_row, timestamp = self._get_sensor_data()
            if sensor_row is None:
                return

            current_time = self.get_clock().now().to_msg()

            if self.data_type == "FLIGHTMATRIX":
                self._publish_flightmatrix_sensor_data(sensor_row, current_time)
            elif self.data_type == "KITTI":
                self._publish_kitti_sensor_data(sensor_row, current_time)

            self.publish_flags['sensor_data'] = True
            if self.debug_sync:
                self.get_logger().debug(f"Published sensor data {self.frame_index}")

    def _publish_flightmatrix_sensor_data(self, sensor_row, current_time):
        # Publish IMU data (accelerometer and gyroscope)
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = float(sensor_row['accelerometer_x'] / 100.0)
        imu_msg.linear_acceleration.y = float(sensor_row['accelerometer_y'] / 100.0)
        imu_msg.linear_acceleration.z = float(sensor_row['accelerometer_z'] / 100.0)
        imu_msg.angular_velocity.x = float(math.radians(sensor_row['gyroscope_x']))
        imu_msg.angular_velocity.y = float(math.radians(sensor_row['gyroscope_y']))
        imu_msg.angular_velocity.z = float(math.radians(sensor_row['gyroscope_z']))
        self.imu_pub.publish(imu_msg)

        # Publish magnetometer data
        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time
        mag_msg.header.frame_id = 'base_link'
        mag_msg.magnetic_field.x = float(sensor_row['magnetometer_x'])
        mag_msg.magnetic_field.y = float(sensor_row['magnetometer_y'])
        mag_msg.magnetic_field.z = float(sensor_row['magnetometer_z'])
        self.mag_pub.publish(mag_msg)

        # Publish odometry (location and orientation)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(sensor_row['location_x'] / 100.0)
        odom_msg.pose.pose.position.y = float(sensor_row['location_y'] / 100.0)
        odom_msg.pose.pose.position.z = float(sensor_row['location_z'] / 100.0)
        q = transforms3d.euler.euler2quat(
            math.radians(sensor_row['orientation_roll']),
            math.radians(sensor_row['orientation_pitch']),
            math.radians(sensor_row['orientation_yaw'])
        )
        odom_msg.pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
        self.odom_pub.publish(odom_msg)

        # Publish LiDAR data
        lidar_msg = Float32MultiArray()
        lidar_msg.data = [
            float(sensor_row['lidar_forward']),
            float(sensor_row['lidar_backward']),
            float(sensor_row['lidar_left']),
            float(sensor_row['lidar_right']),
            float(sensor_row['lidar_bottom'])
        ]
        self.lidar_pub.publish(lidar_msg)

        # Publish collision data if collision detected
        if sensor_row['collision_status']:
            collision_msg = PoseStamped()
            collision_msg.header.stamp = current_time
            collision_msg.header.frame_id = 'base_link'
            collision_msg.pose.position.x = float(sensor_row['collision_location_x'] / 100.0)
            collision_msg.pose.position.y = float(sensor_row['collision_location_y'] / 100.0)
            collision_msg.pose.position.z = float(sensor_row['collision_location_z'] / 100.0)
            self.collision_pub.publish(collision_msg)

    def _publish_kitti_sensor_data(self, sensor_row, current_time):
        # Publish odometry (location and orientation)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = sensor_row[0]
        odom_msg.pose.pose.position.y = sensor_row[1]
        odom_msg.pose.pose.position.z = sensor_row[2]
        q = transforms3d.euler.euler2quat(sensor_row[3], sensor_row[4], sensor_row[5])
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        self.odom_pub.publish(odom_msg)

        # Publish IMU data (accelerometer and gyroscope)
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = sensor_row[10]
        imu_msg.linear_acceleration.y = sensor_row[11]
        imu_msg.linear_acceleration.z = sensor_row[12]
        imu_msg.angular_velocity.x = sensor_row[20]
        imu_msg.angular_velocity.y = sensor_row[21]
        imu_msg.angular_velocity.z = sensor_row[22]
        self.imu_pub.publish(imu_msg)

        # Publish magnetometer data
        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time
        mag_msg.header.frame_id = 'base_link'
        mag_msg.magnetic_field.x = sensor_row[23]
        mag_msg.magnetic_field.y = sensor_row[24]
        mag_msg.magnetic_field.z = sensor_row[25]
        self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightMatrixPublisher()
    if node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()
