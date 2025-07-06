import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import json
from datetime import datetime

class RosbagControlNode(Node):
    def __init__(self):
        super().__init__('rosbag_control_node')
        self.subscription = self.create_subscription(
            String, '/bag_command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, '/bag_status', 10)
        self.rosbag_process = None
        self.recording = False
        self.current_topics = []
        self.current_dest = ''
        self.current_filename = ''
        self.recordings_dir = os.path.expanduser('~/Recordings')
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid command: {msg.data} ({e})")
            return

        if cmd.get('action') == 'start' and self.rosbag_process is None:
            topics = cmd.get('topics', ['--all'])
            dest = cmd.get('dest', self.recordings_dir)
            filename = cmd.get('filename', 'bag')
            now = datetime.now()
            date_dir = now.strftime('%Y-%m-%d')
            time_dir = now.strftime('%H-%M-%S')
            output_dir = os.path.join(os.path.expanduser(dest), date_dir, time_dir)
            os.makedirs(output_dir, exist_ok=True)
            bag_path = os.path.join(output_dir, filename)
            self.get_logger().info(f"Starting rosbag recording: {bag_path}, topics: {topics}")
            self.rosbag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path] + topics
            )
            self.recording = True
            self.current_topics = topics
            self.current_dest = dest
            self.current_filename = filename

        elif cmd.get('action') == 'stop' and self.rosbag_process:
            self.get_logger().info('Stopping rosbag recording...')
            self.rosbag_process.terminate()
            self.rosbag_process = None
            self.recording = False
            self.current_topics = []
            self.current_dest = ''
            self.current_filename = ''

    def publish_status(self):
        msg = String()
        msg.data = 'recording' if self.recording and self.rosbag_process else 'idle'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RosbagControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

