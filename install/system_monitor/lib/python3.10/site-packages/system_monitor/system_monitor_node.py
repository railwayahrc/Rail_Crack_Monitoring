import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
from datetime import datetime

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')

        self.battery_pub = self.create_publisher(String, '/battery_percent', 10)
        self.disk_pub = self.create_publisher(String, '/disk_usage', 10)
        self.time_pub = self.create_publisher(String, '/system_time', 10)

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        try:
            # Battery info
            battery = psutil.sensors_battery()
            battery_str = f"{battery.percent:.1f}%" if battery else "N/A"
            self.battery_pub.publish(String(data=battery_str))

            # Disk usage
            disk = psutil.disk_usage('/')
            disk_str = f"{disk.percent:.1f}%"
            self.disk_pub.publish(String(data=disk_str))

            # System time
            now = datetime.now().strftime('%H:%M:%S')
            self.time_pub.publish(String(data=now))

        except Exception as e:
            self.get_logger().error(f"System monitor error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

