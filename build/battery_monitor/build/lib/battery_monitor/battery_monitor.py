import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(String, 'battery_status', 10)
        self.timer = self.create_timer(5.0, self.publish_battery_status)  # Publish every 5 seconds

    def publish_battery_status(self):
        battery = psutil.sensors_battery()
        if battery:
            status = "Charging" if battery.power_plugged else "Discharging"
            battery_percentage = battery.percent
            battery_info = f"Battery: {battery_percentage}% - {status}"
        else:
            battery_info = "Battery status not available."

        self.publisher_.publish(String(data=battery_info))
        self.get_logger().info(f"Published: {battery_info}")

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)

    # Cleanup when shutting down
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
