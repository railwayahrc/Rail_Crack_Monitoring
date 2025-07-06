import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from datetime import datetime

class RosbagControlNode(Node):
    def __init__(self):
            super().__init__('rosbag_control_node')
            self.subscription = self.create_subscription(String, 'bag_command', self.command_callback, 10)
            self.rosbag_process = None
            topic_param = self.declare_parameter('topics', '').value
            self.topics = topic_param.split(';') if topic_param else ['--all']
            self.get_logger().info(f"Recording topics: {self.topics}")
            self.recordings_dir = os.path.expanduser('~/Recordings')

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == 'start' and self.rosbag_process is None:
            now = datetime.now()
            date_dir = now.strftime('%Y-%m-%d')
            time_dir = now.strftime('%H-%M-%S')
            output_dir = os.path.join(self.recordings_dir, date_dir, time_dir)

            os.makedirs(output_dir, exist_ok=True)

            self.get_logger().info(f'Starting recording in {output_dir}')
            self.rosbag_process = subprocess.Popen(
            	['ros2', 'bag', 'record', '-o', os.path.join(output_dir, 'bag')] + self.topics
   		)

        
        elif cmd == 'stop' and self.rosbag_process:
            self.get_logger().info('Stopping rosbag recording...')
            self.rosbag_process.terminate()
            self.rosbag_process = None

def main(args=None):
    rclpy.init(args=args)
    node = RosbagControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

