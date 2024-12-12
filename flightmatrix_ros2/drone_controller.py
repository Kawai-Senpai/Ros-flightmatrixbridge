import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from flightmatrix.bridge import FlightMatrixBridge
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult

class DroneController(Node):
    """
    A ROS2 node that controls a drone using joystick inputs and manages camera frame parameters.
    Attributes:
        bridge (FlightMatrixBridge): An instance of the FlightMatrixBridge class to interface with the drone.
        subscription (Subscription): A ROS2 subscription to the 'joy' topic for receiving joystick inputs.
    Methods:
        __init__():
            Initializes the DroneController node, declares parameters, and sets up the bridge and subscription.
        parameters_callback(params):
            Callback function that updates the bridge configuration when parameters change.
        joy_callback(msg):
            Callback function that maps joystick inputs to drone movement commands and sends them to the bridge.
    """
    def __init__(self):
        super().__init__('drone_controller')
        
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

        # Get initial parameter values
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        noise_level = self.get_parameter('noise_level').value
        apply_noise = self.get_parameter('apply_noise').value

        # Initialize bridge with parameters
        self.bridge = FlightMatrixBridge(
            resolution=(width, height),
            noise_level=noise_level,
            apply_noise=apply_noise
        )
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

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

    def joy_callback(self, msg):
        # Map joystick axes to movement commands
        x = msg.axes[0]  # Forward/Backward control
        y = msg.axes[1]  # Left/Right control
        z = msg.axes[2]  # Up/Down control
        roll = msg.axes[3]  # Roll control
        pitch = msg.axes[4]  # Pitch control
        yaw = msg.axes[5]  # Yaw control

        # Send movement command to FlightMatrix
        self.bridge.send_movement_command(x, y, z, roll, pitch, yaw)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()