import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from collections import defaultdict, deque
import time
import importlib

EXCLUDE_TOPICS = ['/topic_frequencies']

class FrequencyMonitor(Node):
    def __init__(self):
        super().__init__('frequency_monitor_node')
        self.monitoring_enabled = True
        self.timestamps = defaultdict(lambda: deque(maxlen=100))
        self.subscribers = []
        self.publisher = self.create_publisher(String, '/topic_frequencies', 10)
        self.srv = self.create_service(SetBool, '/toggle_frequency_monitor', self.handle_toggle_monitor)
        self.timer = self.create_timer(2.0, self.publish_frequencies)
        self.topic_types = {}  # topic_name -> type_string
        self._discover_and_subscribe()

    def _discover_and_subscribe(self):
        # Get all topic names and types
        topic_info = self.get_topic_names_and_types()
        # Exclude output and system topics
        self.topic_types = {name: types[0] for name, types in topic_info if name not in EXCLUDE_TOPICS and types}
        self.get_logger().info(f"Monitoring topics: {list(self.topic_types.keys())}")
        self._create_subscribers()

    def _create_subscribers(self):
        # Remove any existing subscribers first
        for sub in self.subscribers:
            self.destroy_subscription(sub)
        self.subscribers = []

        if self.monitoring_enabled:
            for topic, type_str in self.topic_types.items():
                msg_class = self._import_message_class(type_str)
                if msg_class is None:
                    self.get_logger().warn(f"Could not import message type {type_str} for topic {topic}")
                    continue
                # Use partial to bind topic name for callback (avoid lambda late binding issue[1])
                from functools import partial
                sub = self.create_subscription(
                    msg_class,
                    topic,
                    partial(self._generic_callback, topic=topic),
                    10
                )
                self.subscribers.append(sub)

    def _import_message_class(self, type_str):
        # type_str example: 'std_msgs/msg/String'
        try:
            pkg, _, msg = type_str.partition('/msg/')
            mod = importlib.import_module(f"{pkg}.msg")
            return getattr(mod, msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to import {type_str}: {e}")
            return None

    def _generic_callback(self, msg, topic):
        if self.monitoring_enabled:
            self.timestamps[topic].append(time.time())

    def publish_frequencies(self):
        if not self.monitoring_enabled:
            return
        lines = []
        for topic in self.topic_types:
            times = self.timestamps[topic]
            if len(times) > 1:
                deltas = [t2 - t1 for t1, t2 in zip(times, list(times)[1:])]
                freq = 1.0 / (sum(deltas) / len(deltas)) if deltas else 0.0
                lines.append(f"{topic}: {freq:.1f} Hz")
            else:
                lines.append(f"{topic}: No data")
        status_msg = "\n".join(lines) if lines else "No topics being monitored."
        self.publisher.publish(String(data=status_msg))

    def handle_toggle_monitor(self, request, response):
        self.monitoring_enabled = request.data
        if self.monitoring_enabled:
            self.get_logger().info("Frequency monitoring ENABLED")
            self._create_subscribers()
            if self.timer is None:
                self.timer = self.create_timer(2.0, self.publish_frequencies)
        else:
            self.get_logger().info("Frequency monitoring DISABLED")
            for sub in self.subscribers:
                self.destroy_subscription(sub)
            self.subscribers = []
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
        response.success = True
        response.message = "Monitoring " + ("enabled" if self.monitoring_enabled else "disabled")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FrequencyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

