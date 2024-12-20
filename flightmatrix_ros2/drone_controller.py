import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import numpy as np
from multiprocessing import shared_memory

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # Define the size for the movement command shared memory
        self.num_floats = 6  # As expected, there are 6 movement command values (x, y, z, r, p, y) #! Do not change this value
        self.data_size = np.dtype(np.float32).itemsize * self.num_floats  # Each float is 4 bytes
        self.total_size = 1 + self.data_size  # 1 byte for availability flag + space for 6 floats
        
        try:
            # Try to create the shared memory block
            self.shm = shared_memory.SharedMemory(name='MovementCommandMemory', create=True, size=self.total_size)
            self.get_logger().info(f"Created shared memory block 'MovementCommandMemory' with size {self.total_size} bytes")
        except FileExistsError:
            # If the shared memory block already exists, open it
            self.shm = shared_memory.SharedMemory(name='MovementCommandMemory', create=False)
            self.get_logger().info(f"Opened shared memory block 'MovementCommandMemory' with size {self.total_size} bytes")

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.last_timestamp = None  # To store the last processed timestamp

    def _send_command(self, command):
        # Create a NumPy array backed by the shared memory buffer (for the 6 float values)
        buffer = self.shm.buf[1:]  # Skip the first byte (availability flag)
        command_array = np.ndarray((self.num_floats,), dtype=np.float32, buffer=buffer)
        # Write the new command data
        command_array[:] = command
        # Set the availability flag (first byte) to 1 (new data available)
        self.shm.buf[0] = 1

    def joy_callback(self, msg):
        
        # Access the timestamp
        timestamp = msg.header.stamp
        self.get_logger().info(f"Received Joy message with timestamp: {timestamp.sec}.{timestamp.nanosec}")

        # Synchronization: Ensure commands are processed in order
        if self.last_timestamp and (timestamp.sec < self.last_timestamp.sec or 
                                    (timestamp.sec == self.last_timestamp.sec and timestamp.nanosec <= self.last_timestamp.nanosec)):
            self.get_logger().warn("Received an out-of-order Joy message. Ignoring.")
            return

        # Update the last processed timestamp
        self.last_timestamp = timestamp

        # Map joystick axes to movement commands
        x = msg.axes[0]  # Forward/Backward control
        y = msg.axes[1]  # Left/Right control
        z = msg.axes[2]  # Up/Down control
        roll = msg.axes[3]  # Roll control
        pitch = msg.axes[4]  # Pitch control
        yaw = msg.axes[5]  # Yaw control

        command = np.array([x, y, z, roll, pitch, yaw], dtype=np.float32)   
        self._send_command(command)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    if node:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()