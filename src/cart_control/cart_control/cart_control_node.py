import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

class CartControlNode(Node):
    def __init__(self):
        super().__init__('cart_control_node')

        # Get serial port parameter
        device_param = self.declare_parameter('device', '/dev/ttyACM0').value
        self.get_logger().info(f"[INIT] Trying to connect to Arduino on port: {device_param}")

        # Try to open serial connection
        try:
            self.ser = serial.Serial(device_param, 9600, timeout=1)
            self.get_logger().info(f"[OK] Connected to Arduino at {device_param}")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Could not open serial port {device_param}: {e}")
            sys.exit(1)

        # Subscribe to incoming cart control messages
        self.subscription = self.create_subscription(
            String,
            'cart_control',
            self.listener_callback,
            10
        )
        self.get_logger().info("[READY] Subscribed to 'cart_control' topic.")

    def listener_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"[RX] Received command from Web UI: '{command}'")

        try:
            serial_command = command + '\n'
            self.ser.write(serial_command.encode())
            self.get_logger().info(f"[TX] Sent to Arduino: '{serial_command.strip()}'")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to send command to Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CartControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[SHUTDOWN] Ctrl+C received, shutting down node.")
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("[CLOSE] Closed serial connection.")
        node.destroy_node()
        rclpy.shutdown()

