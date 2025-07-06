#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import json

class DiskSpacePublisher(Node):
    def __init__(self):
        super().__init__('disk_space_publisher')
        self.publisher_ = self.create_publisher(String, 'disk_space', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        disk_space = subprocess.check_output(['df', '-h']).decode('utf-8')
        disk_space = subprocess.check_output(['df', '/']).decode('utf-8').split('\n')[1].split()
        disk_space_json = json.dumps({
            'Filesystem': disk_space[0],
            '1K-blocks': int(disk_space[1]),
            'Used': int(disk_space[2]),
            'Available': int(disk_space[3]),
            'Use%': disk_space[4],
            'Mounted on': disk_space[5]
        })
        msg = String()
        msg.data = disk_space_json
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    disk_space_publisher = DiskSpacePublisher()
    rclpy.spin(disk_space_publisher)
    disk_space_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
