import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray, Bool
import transforms3d
import math
import cv2
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
from flightmatrix.bridge import FlightMatrixBridge

class FlightMatrixPublisher(Node):
    """
    A ROS2 node that publishes various types of data from the FlightMatrix simulation environment.
    This node handles the following functionalities:
    - Declares and manages configuration parameters for frame dimensions, noise level, and noise application.
    - Initializes the FlightMatrixBridge with the specified parameters.
    - Sets up publishers for different types of frames (left, right, z-depth, segmentation) and sensor data (odometry, IMU, magnetometer, LiDAR, collision).
    - Creates timers to periodically publish the frames and sensor data.
    - Provides callbacks for parameter updates and frame publishing.
    - Converts timestamps from milliseconds to ROS Time messages.
    - Handles the publishing of sensor data including IMU, magnetometer, odometry, LiDAR, and collision information.
    Attributes:
        bridge (CvBridge): Bridge for converting OpenCV images to ROS Image messages.
        fm_bridge (FlightMatrixBridge): Interface to the FlightMatrix simulation environment.
        publish_left_frame (bool): Flag to publish left camera frames.
        publish_right_frame (bool): Flag to publish right camera frames.
        publish_left_zdepth (bool): Flag to publish left z-depth frames.
        publish_right_zdepth (bool): Flag to publish right z-depth frames.
        publish_left_seg (bool): Flag to publish left segmentation frames.
        publish_right_seg (bool): Flag to publish right segmentation frames.
        left_frame_pub (Publisher): Publisher for left camera frames.
        right_frame_pub (Publisher): Publisher for right camera frames.
        left_zdepth_pub (Publisher): Publisher for left z-depth frames.
        right_zdepth_pub (Publisher): Publisher for right z-depth frames.
        left_seg_pub (Publisher): Publisher for left segmentation frames.
        right_seg_pub (Publisher): Publisher for right segmentation frames.
        odom_pub (Publisher): Publisher for odometry data.
        imu_pub (Publisher): Publisher for IMU data.
        mag_pub (Publisher): Publisher for magnetometer data.
        lidar_pub (Publisher): Publisher for LiDAR data.
        collision_pub (Publisher): Publisher for collision data.
        sensor_timer (Timer): Timer for periodically publishing sensor data.
    Methods:
        __init__(): Initializes the node, declares parameters, sets up publishers and timers.
        convert_timestamp(timestamp_ms): Converts a millisecond timestamp to a ROS Time message.
        publish_left_frame_cb(): Callback to publish left camera frames.
        publish_right_frame_cb(): Callback to publish right camera frames.
        publish_left_zdepth_cb(): Callback to publish left z-depth frames.
        publish_right_zdepth_cb(): Callback to publish right z-depth frames.
        publish_left_seg_cb(): Callback to publish left segmentation frames.
        publish_right_seg_cb(): Callback to publish right segmentation frames.
        create_black_image(single_channel): Creates a black image of the appropriate size.
        parameters_callback(params): Handles parameter updates.
        publish_sensor_data(): Publishes sensor data including IMU, magnetometer, odometry, LiDAR, and collision information.
    """
    def __init__(self):
        super().__init__('flightmatrix_publisher')
        
        # Add configuration parameters with descriptors
        self.declare_parameter('frame_width', 1226,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                description='Width of the camera frames'))
        self.declare_parameter('frame_height', 370,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                description='Height of the camera frames'))
        self.declare_parameter('noise_level', 0.01,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='Level of noise to apply'))
        self.declare_parameter('apply_noise', False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                description='Whether to apply noise to frames'))

        # Get initial parameter values
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        noise_level = self.get_parameter('noise_level').value
        apply_noise = self.get_parameter('apply_noise').value

        # Initialize bridge with parameters
        self.bridge = CvBridge()
        # TODO check if we can simultaneously create two objects for FlightMatrixBridge from two different processes
        self.fm_bridge = FlightMatrixBridge(
            resolution=(width, height),
            noise_level=noise_level,
            apply_noise=apply_noise
        )

        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Parameters to select which frames to publish
        self.declare_parameter('publish_left_frame', True)
        self.declare_parameter('publish_right_frame', True)
        self.declare_parameter('publish_left_zdepth', False)
        self.declare_parameter('publish_right_zdepth', False)
        self.declare_parameter('publish_left_seg', False)
        self.declare_parameter('publish_right_seg', False)
        
        # Fetch parameters
        self.publish_left_frame = self.get_parameter('publish_left_frame').get_parameter_value().bool_value
        self.publish_right_frame = self.get_parameter('publish_right_frame').get_parameter_value().bool_value
        self.publish_left_zdepth = self.get_parameter('publish_left_zdepth').get_parameter_value().bool_value
        self.publish_right_zdepth = self.get_parameter('publish_right_zdepth').get_parameter_value().bool_value
        self.publish_left_seg = self.get_parameter('publish_left_seg').get_parameter_value().bool_value
        self.publish_right_seg = self.get_parameter('publish_right_seg').get_parameter_value().bool_value
        
        # Publishers
        # TODO only create a publisher if the aforementioned parameters are true
        self.left_frame_pub = self.create_publisher(Image, 'left_frame', 10)
        self.right_frame_pub = self.create_publisher(Image, 'right_frame', 10)
        self.left_zdepth_pub = self.create_publisher(Image, 'left_zdepth', 10)
        self.right_zdepth_pub = self.create_publisher(Image, 'right_zdepth', 10)
        self.left_seg_pub = self.create_publisher(Image, 'left_seg', 10)
        self.right_seg_pub = self.create_publisher(Image, 'right_seg', 10)
        
        # Replace single timer with separate timers for each frame type
        if self.publish_left_frame:
            self.left_frame_timer = self.create_timer(0.0, self.publish_left_frame_cb)
        if self.publish_right_frame:
            self.right_frame_timer = self.create_timer(0.0, self.publish_right_frame_cb)
        if self.publish_left_zdepth:
            self.left_zdepth_timer = self.create_timer(0.0, self.publish_left_zdepth_cb)
        if self.publish_right_zdepth:
            self.right_zdepth_timer = self.create_timer(0.0, self.publish_right_zdepth_cb)
        if self.publish_left_seg:
            self.left_seg_timer = self.create_timer(0.0, self.publish_left_seg_cb)
        if self.publish_right_seg:
            self.right_seg_timer = self.create_timer(0.0, self.publish_right_seg_cb)
        
        # Additional publishers for sensor data
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.lidar_pub = self.create_publisher(Float32MultiArray, 'lidar_data', 10)
        self.collision_pub = self.create_publisher(PoseStamped, 'collision', 10)
        
        # Create timer for sensor data publishing
        self.sensor_timer = self.create_timer(0.0, self.publish_sensor_data)  # 100Hz
    
    def convert_timestamp(self, timestamp_ms):
        """Convert millisecond timestamp to ROS Time message"""
        seconds = timestamp_ms // 1000
        nanoseconds = (timestamp_ms % 1000) * 1000000
        time_msg = Time()
        time_msg.sec = int(seconds)
        time_msg.nanosec = int(nanoseconds)
        return time_msg

    def publish_left_frame_cb(self):
        left_frame_data = self.fm_bridge.get_left_frame()
        if left_frame_data.get('error') is None:
            left_frame = left_frame_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(left_frame, encoding='bgr8')
            msg.header.stamp = self.convert_timestamp(left_frame_data.get('timestamp'))
            self.left_frame_pub.publish(msg)

    def publish_right_frame_cb(self):
        right_frame_data = self.fm_bridge.get_right_frame()
        if right_frame_data.get('error') is None:
            right_frame = right_frame_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(right_frame, encoding='bgr8')
            msg.header.stamp = self.convert_timestamp(right_frame_data.get('timestamp'))
            self.right_frame_pub.publish(msg)

    def publish_left_zdepth_cb(self):
        left_zdepth_data = self.fm_bridge.get_left_zdepth()
        if left_zdepth_data.get('error') is None:
            left_zdepth = left_zdepth_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(left_zdepth, encoding='mono8')
            msg.header.stamp = self.convert_timestamp(left_zdepth_data.get('timestamp'))
            self.left_zdepth_pub.publish(msg)

    def publish_right_zdepth_cb(self):
        right_zdepth_data = self.fm_bridge.get_right_zdepth()
        if right_zdepth_data.get('error') is None:
            right_zdepth = right_zdepth_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(right_zdepth, encoding='mono8')
            msg.header.stamp = self.convert_timestamp(right_zdepth_data.get('timestamp'))
            self.right_zdepth_pub.publish(msg)

    def publish_left_seg_cb(self):
        left_seg_data = self.fm_bridge.get_left_seg()
        if left_seg_data.get('error') is None:
            left_seg = left_seg_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(left_seg, encoding='bgr8')  # Changed to bgr8 for RGB segmentation
            msg.header.stamp = self.convert_timestamp(left_seg_data.get('timestamp'))
            self.left_seg_pub.publish(msg)

    def publish_right_seg_cb(self):
        right_seg_data = self.fm_bridge.get_right_seg()
        if right_seg_data.get('error') is None:
            right_seg = right_seg_data.get('frame')
            msg = self.bridge.cv2_to_imgmsg(right_seg, encoding='bgr8')  # Changed to bgr8 for RGB segmentation
            msg.header.stamp = self.convert_timestamp(right_seg_data.get('timestamp'))
            self.right_seg_pub.publish(msg)
    
    def create_black_image(self, single_channel=False):
        # Create a black image of the appropriate size
        height, width = self.fm_bridge.height, self.fm_bridge.width
        if single_channel:
            return np.zeros((height, width), dtype=np.uint8)
        else:
            return np.zeros((height, width, 3), dtype=np.uint8)

    def parameters_callback(self, params):
        """Handle parameter updates"""
        result = SetParametersResult(successful=True)

        try:
            for param in params:
                if param.name == 'frame_width' or param.name == 'frame_height':
                    width = self.get_parameter('frame_width').value
                    height = self.get_parameter('frame_height').value
                    self.fm_bridge.set_resolution(width, height)
                    self.get_logger().info(f'Updated resolution to {width}x{height}')
                
                elif param.name == 'noise_level':
                    self.fm_bridge.set_noise_level(param.value)
                    self.get_logger().info(f'Updated noise level to {param.value}')
                
                elif param.name == 'apply_noise':
                    self.fm_bridge.set_apply_noise(param.value)
                    self.get_logger().info(f'Updated apply_noise to {param.value}')

        except Exception as e:
            result.successful = False
            result.reason = str(e)

        return result

    def publish_sensor_data(self):
        sensor_data = self.fm_bridge.get_sensor_data()
        if sensor_data.get('error') is None:
            timestamp = self.convert_timestamp(sensor_data.get('timestamp'))
            
            # Publish IMU data (accelerometer and gyroscope)
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = 'base_link'
            
            # Convert to m/s² (from cm/s²)
            accel = sensor_data['accelerometer']
            imu_msg.linear_acceleration.x = accel[0] / 100.0
            imu_msg.linear_acceleration.y = accel[1] / 100.0
            imu_msg.linear_acceleration.z = accel[2] / 100.0
            
            # Convert to radians/s (from degrees/s)
            gyro = sensor_data['gyroscope']
            imu_msg.angular_velocity.x = math.radians(gyro[0])
            imu_msg.angular_velocity.y = math.radians(gyro[1])
            imu_msg.angular_velocity.z = math.radians(gyro[2])
            
            self.imu_pub.publish(imu_msg)
            
            # Publish magnetometer data
            mag_msg = MagneticField()
            mag_msg.header.stamp = timestamp
            mag_msg.header.frame_id = 'base_link'
            mag = sensor_data['magnetometer']
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
            loc = sensor_data['location']
            odom_msg.pose.pose.position.x = loc[0] / 100.0
            odom_msg.pose.pose.position.y = loc[1] / 100.0
            odom_msg.pose.pose.position.z = loc[2] / 100.0
            
            # Convert orientation to quaternion (from euler degrees)
            orient = sensor_data['orientation']
            q = transforms3d.euler.euler2quat(
                math.radians(orient[0]),  # roll
                math.radians(orient[1]),  # pitch
                math.radians(orient[2])   # yaw
            )
            odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.odom_pub.publish(odom_msg)
            
            # Publish LiDAR data
            lidar_msg = Float32MultiArray()
            lidar_msg.data = sensor_data['lidar']  # Keep in cm
            self.lidar_pub.publish(lidar_msg)
            
            # Publish collision data if collision detected
            if sensor_data['collision'][0]:  # If collision status is True
                collision_msg = PoseStamped()
                collision_msg.header.stamp = timestamp
                collision_msg.header.frame_id = 'base_link'
                # Convert collision location to meters (from cm)
                collision_msg.pose.position.x = sensor_data['collision'][1] / 100.0
                collision_msg.pose.position.y = sensor_data['collision'][2] / 100.0
                collision_msg.pose.position.z = sensor_data['collision'][3] / 100.0
                self.collision_pub.publish(collision_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlightMatrixPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
