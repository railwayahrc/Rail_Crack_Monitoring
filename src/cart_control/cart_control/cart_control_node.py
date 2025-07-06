import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys
import re
import threading

class CartControlNode(Node):
    def __init__(self):
        super().__init__('cart_control_node')

        device_param = self.declare_parameter(
            'device',
            '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_03536383236351902133-if00'
        ).value
        self.get_logger().info(f"[INIT] Trying to connect to Arduino on port: {device_param}")

        try:
            self.ser = serial.Serial(device_param, 9600, timeout=1)
            self.get_logger().info(f"[OK] Connected to Arduino at {device_param}")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Could not open serial port {device_param}: {e}")
            sys.exit(1)

        self.cart_says_pub = self.create_publisher(String, 'cart_says', 10)
        self._serial_thread_running = True
        self.serial_thread = threading.Thread(target=self.read_from_serial)
        self.serial_thread.start()

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

        # Regex to extract all parameters (expecting integer values)
        pattern = r"SPEED:(\d+)\s+DIR:(\d+)\s+L1:(\d+)\s+L2:(\d+)\s+ARM:(\d+)"
        match = re.search(pattern, command)

        if not match:
            # Anomaly detected: missing or malformed parameter
            safe_command = "SPEED:0 DIR:0 L1:0 L2:0 ARM:0\n"
            try:
                self.ser.write(safe_command.encode())
                self.get_logger().warn(
                    "[ANOMALY] Malformed or incomplete command received! "
                    "Sent safe command to Arduino: 'SPEED:0 DIR:0 L1:0 L2:0 ARM:0'"
                )
            except Exception as e:
                self.get_logger().error(f"[ERROR] Failed to send safe command to Arduino: {e}")
            return

        # All parameters present and valid
        try:
            serial_command = command + '\n'
            self.ser.write(serial_command.encode())
            self.get_logger().info(f"[TX] Sent to Arduino: '{serial_command.strip()}'")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to send command to Arduino: {e}")

    def read_from_serial(self):
        while self._serial_thread_running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        msg = String()
                        msg.data = line
                        self.cart_says_pub.publish(msg)
                        self.get_logger().info(f"[ARDUINO] {line}")
            except Exception as e:
                self.get_logger().error(f"[ERROR] Reading from Arduino: {e}")

    def destroy_node(self):
        self._serial_thread_running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        super().destroy_node()

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

