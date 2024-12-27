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
        
        self.declare_parameter('config_file', '')
        self.declare_parameter('data_directory', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.data_dir = self.get_parameter('data_directory').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error("Config file path not provided")
            return
        
        if not self.data_dir:
            self.get_logger().error("Data directory path not provided")
            return
        
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)['flightmatrix_publisher']['ros__parameters']
                self.get_logger().info(f"Config file loaded: {config_file}")
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_file}")
            return
        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error parsing config file: {exc}")
            return

        self.bridge = CvBridge()

        # Load the sensor data from the data directory
        sensor_data_path = os.path.join(self.data_dir, 'sensor_data.csv')
        if not os.path.exists(sensor_data_path):
            self.get_logger().error(f"Sensor data file not found: {sensor_data_path}")
            return
        self.sensor_data = pd.read_csv(sensor_data_path)
        self.get_logger().info(f"Sensor data loaded: {sensor_data_path}")
        self.frame_index = 0
        
        # Initialize flags only for enabled publishers
        self.publish_flags = {
            publisher: False 
            for publisher, enabled in config['publishers'].items() 
            if enabled and publisher != 'queue_size' and publisher != 'timer_delay'
        }

        # Add debug parameter
        self.declare_parameter('debug_sync', False)
        self.debug_sync = self.get_parameter('debug_sync').get_parameter_value().bool_value

        # Cache for current frame data
        self._current_sensor_data = None
        self._current_timestamp = None
        
        # Add frame increment timer
        self.frame_increment_timer = self.create_timer(
            config['publishers']['timer_delay'], 
            self._check_and_increment_frame
        )

        # Load the resolution parameters from the config file
        self.width = config['resolution']['width']
        self.height = config['resolution']['height']
        
        # Check for required directories and resolution mismatches
        self._check_directories_and_resolution(config)

        if config['publishers']['left_frame']:
            self.left_frame_pub = self.create_publisher(Image, 'left_frame', config['publishers']['queue_size'])
            self.left_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_frame_cb)

        if config['publishers']['right_frame']:
            self.right_frame_pub = self.create_publisher(Image, 'right_frame', config['publishers']['queue_size'])
            self.right_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_frame_cb)

        if config['publishers']['left_zdepth']:
            self.left_zdepth_pub = self.create_publisher(Image, 'left_zdepth', config['publishers']['queue_size'])
            self.left_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_zdepth_cb)

        if config['publishers']['right_zdepth']:
            self.right_zdepth_pub = self.create_publisher(Image, 'right_zdepth', config['publishers']['queue_size'])
            self.right_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_zdepth_cb)

        if config['publishers']['left_seg']:
            self.left_seg_pub = self.create_publisher(Image, 'left_seg', config['publishers']['queue_size'])
            self.left_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_seg_cb)

        if config['publishers']['right_seg']:
            self.right_seg_pub = self.create_publisher(Image, 'right_seg', config['publishers']['queue_size'])
            self.right_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_seg_cb)

        if config['publishers']['sensor_data']:
            self.odom_pub = self.create_publisher(Odometry, 'odometry', config['publishers']['queue_size'])
            self.imu_pub = self.create_publisher(Imu, 'imu/data', config['publishers']['queue_size'])
            self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', config['publishers']['queue_size'])
            self.lidar_pub = self.create_publisher(Float32MultiArray, 'lidar_data', config['publishers']['queue_size'])
            self.collision_pub = self.create_publisher(PoseStamped, 'collision', config['publishers']['queue_size'])
            self.sensor_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_sensor_data)
            self.publish_flags['sensor_data'] = False  # Ensure sensor_data flag is initialized
    
    def _check_directories_and_resolution(self, config):
        
        required_dirs = []
        if config['publishers']['left_frame']:
            required_dirs.append('left_frames')

        if config['publishers']['right_frame']:
            required_dirs.append('right_frames')

        if config['publishers']['left_zdepth']:
            required_dirs.append('left_zdepth')

        if config['publishers']['right_zdepth']:
            required_dirs.append('right_zdepth')

        if config['publishers']['left_seg']:
            required_dirs.append('left_segmentation')

        if config['publishers']['right_seg']:
            required_dirs.append('right_segmentation')

        for dir_name in required_dirs:
            dir_path = os.path.join(self.data_dir, dir_name)
            if not os.path.exists(dir_path):
                self.get_logger().error(f"Required directory not found: {dir_path}")
                return

            # Check resolution of the first image in the directory
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

    def _check_and_increment_frame(self):
        """Check if all publishers are done and increment frame index if true"""
        if all(self.publish_flags.values()):
            self.frame_index = (self.frame_index + 1) % len(self.sensor_data)
            if self.debug_sync:
                self.get_logger().debug(f"Moving to frame {self.frame_index}")
            # Reset flags and cached data
            self.publish_flags = {k: False for k in self.publish_flags}
            self._current_sensor_data = None
            self._current_timestamp = None

    def _get_sensor_data(self):
        """Cache and return sensor data for current frame"""
        if self._current_sensor_data is None:
            try:
                self._current_sensor_data = self.sensor_data.iloc[self.frame_index]
                self._current_timestamp = self.get_clock().now().to_msg()
            except IndexError:
                self.get_logger().error(f"Frame index {self.frame_index} out of range")
                return None, None
        return self._current_sensor_data, self._current_timestamp

    def _get_frame(self, frame_type, rgb=True):
        """Get frame with cached timestamp"""
        try:
            frame_path = os.path.join(self.data_dir, frame_type, f"{self.frame_index}.png")
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
        if not self.publish_flags['left_frame']:  # Changed condition
            msg = self._get_frame('left_frames', rgb=True)
            if msg:
                self.left_frame_pub.publish(msg)
                self.publish_flags['left_frame'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left frame {self.frame_index}")

    def publish_right_frame_cb(self):
        if not self.publish_flags['right_frame']:  # Changed condition
            msg = self._get_frame('right_frames', rgb=True)
            if msg:
                self.right_frame_pub.publish(msg)
                self.publish_flags['right_frame'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right frame {self.frame_index}")

    def publish_left_zdepth_cb(self):
        if not self.publish_flags['left_zdepth']:  # Changed condition
            msg = self._get_frame('left_zdepth', rgb=False)
            if msg:
                self.left_zdepth_pub.publish(msg)
                self.publish_flags['left_zdepth'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left zdepth {self.frame_index}")

    def publish_right_zdepth_cb(self):
        if not self.publish_flags['right_zdepth']:  # Changed condition
            msg = self._get_frame('right_zdepth', rgb=False)
            if msg:
                self.right_zdepth_pub.publish(msg)
                self.publish_flags['right_zdepth'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right zdepth {self.frame_index}")

    def publish_left_seg_cb(self):
        if not self.publish_flags['left_seg']:  # Changed condition
            msg = self._get_frame('left_segmentation', rgb=True)
            if msg:
                self.left_seg_pub.publish(msg)
                self.publish_flags['left_seg'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left segmentation {self.frame_index}")

    def publish_right_seg_cb(self):
        if not self.publish_flags['right_seg']:  # Changed condition
            msg = self._get_frame('right_segmentation', rgb=True)
            if msg:
                self.right_seg_pub.publish(msg)
                self.publish_flags['right_seg'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right segmentation {self.frame_index}")

    def publish_sensor_data(self):
        if not self.publish_flags['sensor_data']:
            sensor_row, timestamp = self._get_sensor_data()
            if sensor_row is None:
                return

            current_time = self.get_clock().now().to_msg()

            # Publish IMU data (accelerometer and gyroscope)
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = 'base_link'
            
            # Convert to m/s² (from cm/s²)
            imu_msg.linear_acceleration.x = float(sensor_row['accelerometer_x'] / 100.0)
            imu_msg.linear_acceleration.y = float(sensor_row['accelerometer_y'] / 100.0)
            imu_msg.linear_acceleration.z = float(sensor_row['accelerometer_z'] / 100.0)
            
            # Convert to radians/s (from degrees/s)
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
            
            # Convert position to meters (from cm)
            odom_msg.pose.pose.position.x = float(sensor_row['location_x'] / 100.0)
            odom_msg.pose.pose.position.y = float(sensor_row['location_y'] / 100.0)
            odom_msg.pose.pose.position.z = float(sensor_row['location_z'] / 100.0)
            
            # Convert orientation to quaternion (from euler degrees)
            q = transforms3d.euler.euler2quat(
                math.radians(sensor_row['orientation_roll']),  # roll
                math.radians(sensor_row['orientation_pitch']),  # pitch
                math.radians(sensor_row['orientation_yaw'])   # yaw
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
            if sensor_row['collision_status']:  # If collision status is True
                collision_msg = PoseStamped()
                collision_msg.header.stamp = current_time
                collision_msg.header.frame_id = 'base_link'
                # Convert collision location to meters (from cm)
                collision_msg.pose.position.x = float(sensor_row['collision_location_x'] / 100.0)
                collision_msg.pose.position.y = float(sensor_row['collision_location_y'] / 100.0)
                collision_msg.pose.position.z = float(sensor_row['collision_location_z'] / 100.0)
                self.collision_pub.publish(collision_msg)

            self.publish_flags['sensor_data'] = True
            if self.debug_sync:
                self.get_logger().debug(f"Published sensor data {self.frame_index}")

def main(args=None):
    rclpy.init(args=args)
    node = FlightMatrixPublisher()
    if node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()
