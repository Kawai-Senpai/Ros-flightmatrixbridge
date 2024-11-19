import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from flightmatrix.bridge import FlightMatrixBridge
from flightmatrix.utilities import DataRecorder
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult

class DataRecorderNode(Node):
    """
    A ROS2 node for recording data using the FlightMatrixBridge.
    This node initializes with several configurable parameters for frame dimensions,
    noise level, and data directory. It also subscribes to a topic to start and stop
    recording based on incoming messages.
    Attributes:
        bridge (FlightMatrixBridge): The bridge object for handling frame data.
        recorder (DataRecorder): The recorder object for saving frame data.
        recording (bool): A flag indicating whether recording is currently active.
        subscription (Subscription): The subscription to the 'start_recording' topic.
    Methods:
        parameters_callback(params):
            Callback function to handle parameter changes dynamically.
        recording_callback(msg):
            Callback function to handle start/stop recording based on incoming messages.
    """
    def __init__(self):
        super().__init__('data_recorder')
        
        # Add configuration parameters
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
        self.declare_parameter('data_directory', '/var/log/flightmatrix_data')

        # Get initial parameter values
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        noise_level = self.get_parameter('noise_level').value
        apply_noise = self.get_parameter('apply_noise').value
        data_dir = self.get_parameter('data_directory').value
        
        # Initialize bridge with parameters
        self.bridge = FlightMatrixBridge(
            resolution=(width, height),
            noise_level=noise_level,
            apply_noise=apply_noise
        )
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.recorder = DataRecorder(self.bridge, base_dir=data_dir)
        self.recording = False
        self.subscription = self.create_subscription(
            Bool,
            'start_recording',
            self.recording_callback,
            10)
        self.subscription

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        try:
            for param in params:
                if param.name == 'frame_width' or param.name == 'frame_height':
                    width = self.get_parameter('frame_width').value
                    height = self.get_parameter('frame_height').value
                    self.bridge.set_resolution(width, height)
                    self.get_logger().info(f'Updated resolution to {width}x{height}')
                elif param.name == 'noise_level':
                    self.bridge.set_noise_level(param.value)
                    self.get_logger().info(f'Updated noise level to {param.value}')
                elif param.name == 'apply_noise':
                    self.bridge.set_apply_noise(param.value)
                    self.get_logger().info(f'Updated apply_noise to {param.value}')
        except Exception as e:
            result.successful = False
            result.reason = str(e)
        return result

    def recording_callback(self, msg):
        if msg.data and not self.recording:
            self.recorder.start_recording()
            self.recording = True
            self.get_logger().info('Recording started.')
        elif not msg.data and self.recording:
            self.recorder.stop_recording()
            self.recording = False
            self.get_logger().info('Recording stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()