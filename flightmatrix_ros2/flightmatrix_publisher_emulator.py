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
from pyproj import Proj, transform

class BaseDataProvider:
    def __init__(self, data_dir, config, data_settings, logger):
        self.data_dir = data_dir
        self.config = config
        self.data_settings = data_settings
        self.logger = logger
        self.frame_index = 0
        self._current_sensor_data = None
        self._current_timestamp = None

        #? Initialize frame map
        self.frame_map = {
            'left_frame': data_settings['left_frame'] if data_settings['left_frame'] else None,
            'right_frame': data_settings['right_frame'] if data_settings['right_frame'] else None,
            'left_zdepth': data_settings['left_zdepth'] if data_settings['left_zdepth'] else None,
            'right_zdepth': data_settings['right_zdepth'] if data_settings['right_zdepth'] else None,
            'left_seg': data_settings['left_seg'] if data_settings['left_seg'] else None,
            'right_seg': data_settings['right_seg'] if data_settings['right_seg'] else None
        }
    
    def get_frame_dir(self, frame_type):
        return self.frame_map[frame_type]

    def load_data(self):
        raise NotImplementedError

    def get_sensor_data(self):
        raise NotImplementedError

    def get_frame(self, frame_type, rgb=True):
        raise NotImplementedError

    def increment_frame(self, loop=True):
        if loop:
            self.frame_index = (self.frame_index + 1) % self.get_total_frames()
        else:
            if self.frame_index < self.get_total_frames() - 1:
                self.frame_index += 1
        self._current_sensor_data = None
        self._current_timestamp = None

    def get_total_frames(self):
        raise NotImplementedError

    def get_imu_data(self):
        raise NotImplementedError

    def get_magnetometer_data(self):
        raise NotImplementedError

    def get_odometry_data(self):
        raise NotImplementedError

    def get_lidar_data(self):
        raise NotImplementedError

    def get_collision_data(self):
        raise NotImplementedError

class FlightMatrixDataProvider(BaseDataProvider):
    
    def load_data(self):

        if not self.data_settings['sensor_data']:
            self.logger.error("Skipping loading sensor data as it is not provided in data settings")
            return False

        sensor_data_path = os.path.join(self.data_dir, self.data_settings['sensor_data'])
        if not os.path.exists(sensor_data_path):
            self.logger.error(f"Sensor data file not found: {sensor_data_path}")
            return False
        self.sensor_data = pd.read_csv(sensor_data_path)
        self.logger.info(f"Sensor data loaded: {sensor_data_path}")
        return True

    def get_sensor_data(self):
        if self._current_sensor_data is None:
            try:
                self._current_sensor_data = self.sensor_data.iloc[self.frame_index]
                self._current_timestamp = self.get_clock().now().to_msg()
            except IndexError:
                self.logger.error(f"Frame index {self.frame_index} out of range")
                return None, None
        return self._current_sensor_data, self._current_timestamp

    def get_frame(self, frame_type, rgb=True):
        try:
            frame_dir = self.get_frame_dir(frame_type)
            frame_path = os.path.join(self.data_dir, frame_dir, f"{self.frame_index}.png")
            frame = cv2.imread(frame_path, cv2.IMREAD_COLOR if rgb else cv2.IMREAD_GRAYSCALE)
            if frame is None:
                self.logger.error(f"Frame not found: {frame_path}")
                return None
            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8' if rgb else 'mono8')
            msg.header.stamp = self.get_clock().now().to_msg()
            return msg
        except Exception as e:
            self.logger.error(f"Error reading frame: {e}")
            return None

    def get_total_frames(self):
        return len(self.sensor_data)

    def get_imu_data(self, sensor_row):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = float(sensor_row['accelerometer_x'] / 100.0)
        imu_msg.linear_acceleration.y = float(sensor_row['accelerometer_y'] / 100.0)
        imu_msg.linear_acceleration.z = float(sensor_row['accelerometer_z'] / 100.0)
        imu_msg.angular_velocity.x = float(math.radians(sensor_row['gyroscope_x']))
        imu_msg.angular_velocity.y = float(math.radians(sensor_row['gyroscope_y']))
        imu_msg.angular_velocity.z = float(math.radians(sensor_row['gyroscope_z']))
        return imu_msg

    def get_magnetometer_data(self, sensor_row):
        mag_msg = MagneticField()
        mag_msg.magnetic_field.x = float(sensor_row['magnetometer_x'])
        mag_msg.magnetic_field.y = float(sensor_row['magnetometer_y'])
        mag_msg.magnetic_field.z = float(sensor_row['magnetometer_z'])
        return mag_msg

    def get_odometry_data(self, sensor_row):
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = float(sensor_row['location_x'] / 100.0)
        odom_msg.pose.pose.position.y = float(sensor_row['location_y'] / 100.0)
        odom_msg.pose.pose.position.z = float(sensor_row['location_z'] / 100.0)
        q = transforms3d.euler.euler2quat(
            math.radians(sensor_row['orientation_roll']),
            math.radians(sensor_row['orientation_pitch']),
            math.radians(sensor_row['orientation_yaw'])
        )
        odom_msg.pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
        return odom_msg

    def get_lidar_data(self, sensor_row):
        lidar_msg = Float32MultiArray()
        lidar_msg.data = [
            float(sensor_row['lidar_forward']),
            float(sensor_row['lidar_backward']),
            float(sensor_row['lidar_left']),
            float(sensor_row['lidar_right']),
            float(sensor_row['lidar_bottom'])
        ]
        return lidar_msg

    def get_collision_data(self, sensor_row):
        if sensor_row['collision_status']:
            collision_msg = PoseStamped()
            collision_msg.pose.position.x = float(sensor_row['collision_location_x'] / 100.0)
            collision_msg.pose.position.y = float(sensor_row['collision_location_y'] / 100.0)
            collision_msg.pose.position.z = float(sensor_row['collision_location_z'] / 100.0)
            return collision_msg
        return None

class KittiDataProvider(BaseDataProvider):
    
    def __init__(self, data_dir, config, data_settings, logger):
        super().__init__(data_dir, config, data_settings, logger)
        self.proj_wgs84 = Proj(init=config['projection']['wgs84'])
        self.proj_local = Proj(proj=config['projection']['local_proj'], zone=config['projection']['local_zone'], datum=config['projection']['local_datum'])

    def load_data(self):

        if not self.data_settings['sensor_data']:
            self.logger.error("Skipping loading sensor data as it is not provided in data settings")
            return False

        oxts_data_path = os.path.join(self.data_dir, self.data_settings['sensor_data'])
        if not os.path.exists(oxts_data_path):
            self.logger.error(f"OXTS data directory not found: {oxts_data_path}")
            return False
        self.oxts_files = sorted([f for f in os.listdir(oxts_data_path) if f.endswith('.txt')])
        if not self.oxts_files:
            self.logger.error(f"No OXTS data files found in: {oxts_data_path}")
            return False
        
        # Load all OXTS data into a DataFrame
        data_list = []
        for file in self.oxts_files:
            file_path = os.path.join(oxts_data_path, file)
            with open(file_path, 'r') as f:
                data = f.readline().strip().split()
                data_list.append([float(x) for x in data])
        self.sensor_data = pd.DataFrame(data_list)
        
        self.logger.info(f"OXTS data loaded: {oxts_data_path}")
        return True

    def get_sensor_data(self):
        if self._current_sensor_data is None:
            try:
                self._current_sensor_data = self.sensor_data.iloc[self.frame_index]
                self._current_timestamp = self.get_clock().now().to_msg()
            except IndexError:
                self.logger.error(f"Frame index {self.frame_index} out of range")
                return None, None
        return self._current_sensor_data, self._current_timestamp

    def get_frame(self, frame_type, rgb=True):
        try:
            frame_dir = self.get_frame_dir(frame_type)
            frame_path = os.path.join(self.data_dir, frame_dir, f"{self.frame_index:010d}.png")
            frame = cv2.imread(frame_path, cv2.IMREAD_COLOR if rgb else cv2.IMREAD_GRAYSCALE)
            if frame is None:
                self.logger.error(f"Frame not found: {frame_path}")
                return None
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8' if rgb else 'mono8')
            msg.header.stamp = self.get_clock().now().to_msg()
            return msg
        except Exception as e:
            self.logger.error(f"Error reading frame: {e}")
            return None

    def get_total_frames(self):
        return len(self.sensor_data)

    def get_imu_data(self, sensor_row):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = sensor_row[11]  # ax
        imu_msg.linear_acceleration.y = sensor_row[12]  # ay
        imu_msg.linear_acceleration.z = sensor_row[13]  # az
        imu_msg.angular_velocity.x = sensor_row[18]  # wx
        imu_msg.angular_velocity.y = sensor_row[19]  # wy
        imu_msg.angular_velocity.z = sensor_row[20]  # wz
        return imu_msg

    def get_magnetometer_data(self, sensor_row):
        return None  # KITTI dataset does not provide magnetometer data

    def get_odometry_data(self, sensor_row):
        odom_msg = Odometry()
        
        # Convert latitude, longitude, altitude to Cartesian coordinates
        x, y = transform(self.proj_wgs84, self.proj_local, sensor_row[1], sensor_row[0])
        z = sensor_row[2]
        
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        q = transforms3d.euler.euler2quat(sensor_row[3], sensor_row[4], sensor_row[5])  # roll, pitch, yaw
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        return odom_msg

    def get_lidar_data(self, sensor_row):
        return None  # KITTI dataset does not provide LiDAR data

    def get_collision_data(self, sensor_row):
        return None  # KITTI dataset does not provide collision data

class FlightMatrixPublisher(Node):
    
    def __init__(self):
        super().__init__('flightmatrix_publisher')
        
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error("Config file path not provided")
            return
        
        try:
            with open(config_file, 'r') as file:
                
                config = yaml.safe_load(file)['flightmatrix_publisher']['ros__parameters']

                #? Initialize data directory, data type, and loop parameters
                if config['data']['data_directory']:
                    self.data_dir = config['data']['data_directory']
                else:
                    self.get_logger().error("Data directory not provided in config file")
                    return
                
                #? Initialize data type and loop parameters
                if config['data']['data_type']:
                    self.data_type = config['data']['data_type']
                else:
                    self.get_logger().error("Data type not provided in config file")
                    return
                
                #? Initialize loop parameter
                if config['data']['loop']:
                    self.loop = config['data']['loop']
                else:
                    self.get_logger().warning("Loop parameter not provided in config file. Defaulting to False")
                    self.loop = False

                #? Get data settings
                data_settings = yaml.safe_load(file)['data_settings'][self.data_type]
                #check if data settings is not empty
                if not data_settings:
                    self.get_logger().error("Data settings not provided in config file")
                    return            

                self.get_logger().info(f"Config file loaded: {config_file}")
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_file}")
            return
        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error parsing config file: {exc}")
            return
        
        self.bridge = CvBridge()

        #? Initialize data provider based on data type
        if self.data_type == 'FLIGHTMATRIX':
            self.data_provider = FlightMatrixDataProvider(self.data_dir, config, data_settings, self.get_logger())
        elif self.data_type == 'KITTI':
            self.data_provider = KittiDataProvider(self.data_dir, config, data_settings, self.get_logger())
        else:
            self.get_logger().error(f"Unsupported data type: {self.data_type}")
            return

        if not self.data_provider.load_data():
            return

        #? Initialize flags only for enabled publishers
        self.publish_flags = {
            publisher: False 
            for publisher, enabled in config['publishers'].items() 
            if enabled and publisher != 'queue_size' and publisher != 'timer_delay'
        }

        #? Add debug parameter
        # If True, debug messages will be printed for each frame published
        self.declare_parameter('debug_sync', False)
        self.debug_sync = self.get_parameter('debug_sync').get_parameter_value().bool_value

        #! Add frame increment timer
        self.frame_increment_timer = self.create_timer(
            config['publishers']['timer_delay'], 
            self._check_and_increment_frame
        )

        # Load the resolution parameters from the config file
        self.width = config['resolution']['width']
        self.height = config['resolution']['height']

        #! Initialize publishers and timers
        if config['publishers']['left_frame'] and data_settings['left_frame']:
            self.left_frame_pub = self.create_publisher(Image, 'left_frame', config['publishers']['queue_size'])
            self.left_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_frame_cb)

        if config['publishers']['right_frame'] and data_settings['right_frame']:
            self.right_frame_pub = self.create_publisher(Image, 'right_frame', config['publishers']['queue_size'])
            self.right_frame_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_frame_cb)

        if config['publishers']['left_zdepth'] and data_settings['left_zdepth']:
            self.left_zdepth_pub = self.create_publisher(Image, 'left_zdepth', config['publishers']['queue_size'])
            self.left_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_zdepth_cb)

        if config['publishers']['right_zdepth'] and data_settings['right_zdepth']:
            self.right_zdepth_pub = self.create_publisher(Image, 'right_zdepth', config['publishers']['queue_size'])
            self.right_zdepth_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_zdepth_cb)

        if config['publishers']['left_seg'] and data_settings['left_seg']:
            self.left_seg_pub = self.create_publisher(Image, 'left_seg', config['publishers']['queue_size'])
            self.left_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_left_seg_cb)

        if config['publishers']['right_seg'] and data_settings['right_seg']:
            self.right_seg_pub = self.create_publisher(Image, 'right_seg', config['publishers']['queue_size'])
            self.right_seg_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_right_seg_cb)

        if config['publishers']['sensor_data'] and data_settings['sensor_data']:
            self.odom_pub = self.create_publisher(Odometry, 'odometry', config['publishers']['queue_size'])
            self.imu_pub = self.create_publisher(Imu, 'imu/data', config['publishers']['queue_size'])
            self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', config['publishers']['queue_size'])
            self.lidar_pub = self.create_publisher(Float32MultiArray, 'lidar_data', config['publishers']['queue_size'])
            self.collision_pub = self.create_publisher(PoseStamped, 'collision', config['publishers']['queue_size'])
            self.sensor_timer = self.create_timer(config['publishers']['timer_delay'], self.publish_sensor_data)
            self.publish_flags['sensor_data'] = False  # Ensure sensor_data flag is initialized

    def _check_and_increment_frame(self):
        """Check if all publishers are done and increment frame index if true"""
        if all(self.publish_flags.values()):
            if self.loop:
                self.data_provider.increment_frame(loop=self.loop)
            else:
                if self.data_provider.frame_index < self.data_provider.get_total_frames() - 1:
                    self.data_provider.increment_frame(loop=self.loop)
                else:
                    self.get_logger().info("Reached the end of data. Shutting down.")
                    rclpy.shutdown()
                    return
            if self.debug_sync:
                self.get_logger().debug(f"Moving to frame {self.data_provider.frame_index}")
            # Reset flags and cached data
            self.publish_flags = {k: False for k in self.publish_flags}

    def publish_left_frame_cb(self):
        if not self.publish_flags['left_frame']:
            msg = self.data_provider.get_frame('left_frame', rgb=True)
            if msg:
                self.left_frame_pub.publish(msg)
                self.publish_flags['left_frame'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left frame {self.data_provider.frame_index}")

    def publish_right_frame_cb(self):
        if not self.publish_flags['right_frame']:
            msg = self.data_provider.get_frame('right_frame', rgb=True)
            if msg:
                self.right_frame_pub.publish(msg)
                self.publish_flags['right_frame'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right frame {self.data_provider.frame_index}")

    def publish_left_zdepth_cb(self):
        if not self.publish_flags['left_zdepth']:
            msg = self.data_provider.get_frame('left_zdepth', rgb=False)
            if msg:
                self.left_zdepth_pub.publish(msg)
                self.publish_flags['left_zdepth'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left zdepth {self.data_provider.frame_index}")

    def publish_right_zdepth_cb(self):
        if not self.publish_flags['right_zdepth']:
            msg = self.data_provider.get_frame('right_zdepth', rgb=False)
            if msg:
                self.right_zdepth_pub.publish(msg)
                self.publish_flags['right_zdepth'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right zdepth {self.data_provider.frame_index}")

    def publish_left_seg_cb(self):
        if not self.publish_flags['left_seg']:
            msg = self.data_provider.get_frame('left_segmentation', rgb=True)
            if msg:
                self.left_seg_pub.publish(msg)
                self.publish_flags['left_seg'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published left segmentation {self.data_provider.frame_index}")

    def publish_right_seg_cb(self):
        if not self.publish_flags['right_seg']:
            msg = self.data_provider.get_frame('right_segmentation', rgb=True)
            if msg:
                self.right_seg_pub.publish(msg)
                self.publish_flags['right_seg'] = True
                if self.debug_sync:
                    self.get_logger().debug(f"Published right segmentation {self.data_provider.frame_index}")

    def publish_sensor_data(self):
        if not self.publish_flags['sensor_data']:
            sensor_row, timestamp = self.data_provider.get_sensor_data()
            if sensor_row is None:
                return

            current_time = self.get_clock().now().to_msg()

            # Publish IMU data
            imu_msg = self.data_provider.get_imu_data(sensor_row)
            if imu_msg:
                imu_msg.header.stamp = current_time
                imu_msg.header.frame_id = 'base_link'
                self.imu_pub.publish(imu_msg)

            # Publish magnetometer data
            mag_msg = self.data_provider.get_magnetometer_data(sensor_row)
            if mag_msg:
                mag_msg.header.stamp = current_time
                mag_msg.header.frame_id = 'base_link'
                self.mag_pub.publish(mag_msg)

            # Publish odometry data
            odom_msg = self.data_provider.get_odometry_data(sensor_row)
            if odom_msg:
                odom_msg.header.stamp = current_time
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'
                self.odom_pub.publish(odom_msg)

            # Publish LiDAR data
            lidar_msg = self.data_provider.get_lidar_data(sensor_row)
            if lidar_msg:
                self.lidar_pub.publish(lidar_msg)

            # Publish collision data
            collision_msg = self.data_provider.get_collision_data(sensor_row)
            if collision_msg:
                collision_msg.header.stamp = current_time
                collision_msg.header.frame_id = 'base_link'
                self.collision_pub.publish(collision_msg)

            self.publish_flags['sensor_data'] = True
            if self.debug_sync:
                self.get_logger().debug(f"Published sensor data {self.data_provider.frame_index}")

def main(args=None):
    rclpy.init(args=args)
    node = FlightMatrixPublisher()
    if node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()