import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import defaultdict, deque
import time

TOPICS_TO_MONITOR = [
    '/camera/image_raw',
    '/gps/fix',
    '/lidar/scan'
]

class FrequencyMonitor(Node):
    def __init__(self):
        super().__init__('frequency_monitor_node')
        topic_param = self.declare_parameter('topics', '').value
        self.topics = topic_param.split(';') if topic_param else []
        self.get_logger().info(f"Monitoring topics: {self.topics}")
        self.timestamps = defaultdict(lambda: deque(maxlen=100))
        self.subscribers = []
        self.publisher = self.create_publisher(String, 'topic_frequencies', 10)
        self.timer = self.create_timer(2.0, self.publish_frequencies)

        for topic in self.topics:
            sub = self.create_subscription(String, topic, self._make_callback(topic), 10)
            self.subscribers.append(sub)

    def _make_callback(self, topic):
        def cb(msg):
            self.timestamps[topic].append(time.time())
        return cb

    def publish_frequencies(self):
        lines = []
        for topic, times in self.timestamps.items():
            if len(times) > 1:
                deltas = [t2 - t1 for t1, t2 in zip(times, list(times)[1:])]
                freq = 1.0 / (sum(deltas) / len(deltas)) if deltas else 0.0
                lines.append(f"{topic}: {freq:.1f} Hz")
            else:
                lines.append(f"{topic}: No data")
        status_msg = "\n".join(lines)
        self.publisher.publish(String(data=status_msg))

def main(args=None):
    rclpy.init(args=args)
    node = FrequencyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

