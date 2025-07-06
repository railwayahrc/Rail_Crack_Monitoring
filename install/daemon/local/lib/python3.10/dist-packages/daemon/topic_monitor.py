#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
import json
# from your_msgs.msg import YourMessageType1, YourMessageType2  # Add more if needed

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        self.publisher_ = self.create_publisher(String, 'topic_status', 10)
        self.my_subscriptions = []  # List to hold all subscriptions
        self.msg_counts = {}  # Dictionary to hold message counts for each topic
        self.timer = self.create_timer(1.0, self.timer_callback)  # create a timer that fires every 1 second

        # List of topics to monitor
        topics = [
            ('/camera1/ir_raw', Image),
            # ('/Jetson1/camera1/zed_node/left_raw/image_raw_color', Image),
            # ('/Jetson1/camera1/zed_node/right_raw/image_raw_color', Image),
            ('/camera1/points2', PointCloud2),
            # ('/Jetson1/z/zed_node/point_cloud/cloud_registered', PointCloud2),
            ('/camera1/depth_raw', Image),
            # ('/Jetson1/camera1/zed_node/imu/data', Imu ),
            ('/fix', NavSatFix),
            ('/fix_velocity', TwistWithCovarianceStamped),
            ('/camera2/ir_raw', Image),
            # ('/Jetson2/camera2/zed_node/left_raw/image_raw_color', Image),
            # ('/Jetson2/camera2/zed_node/right_raw/image_raw_color', Image),
            ('/camera2/points2', PointCloud2),
            # ('/Jetson2/camera2/zed_node/point_cloud/cloud_registered', PointCloud2),
            ('/camera2/depth_raw', Image),
            ('/timestamp', String)
            # ('/Jetson2/camera2/zed_node/imu/data', Imu ),
            
            # Add more topics as needed
        ]

        # Create a subscription for each topic
        for topic, msg_type in topics:
            self.my_subscriptions.append(self.create_subscription(msg_type, topic, self.make_callback(topic), 10))
            self.msg_counts[topic] = 0  # Initialize message count for this topic

    def make_callback(self, topic):
        def callback(msg):
            # Increment the message count for the topic this message was received on
            self.msg_counts[topic] += 1
        return callback

    def timer_callback(self):
        # Publish the frequency of messages for each topic
        status_msg = String()
        status_msg.data = json.dumps(self.msg_counts)  # Convert the dictionary to a JSON string
        self.publisher_.publish(status_msg)
        self.msg_counts = dict.fromkeys(self.msg_counts, 0)  # reset the message count

def main(args=None):
    rclpy.init(args=args)

    topic_monitor = TopicMonitor()

    rclpy.spin(topic_monitor)

    # Destroy the node explicitly
    topic_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
