import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import subprocess
import threading
import os

class CameraWorker(threading.Thread):
    def __init__(self, node: Node, device_id: int, topic_name: str):
        super().__init__(daemon=True)
        self.node = node
        self.device_id = device_id
        self.topic_name = topic_name
        self.device_path = f'/dev/video{device_id}'

        self.cap = cv2.VideoCapture(device_id)
        self.publisher = self.node.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        self.running = self.cap.isOpened()

        # Declare and retrieve camera-specific parameters
        self.ns = f'camera_{device_id}'
        self.node.declare_parameter(f'{self.ns}.exposure_mode', 'auto')  # or 'manual'
        self.node.declare_parameter(f'{self.ns}.exposure_value', 157)    # default Arducam value

        # Initial exposure configuration
        self.configure_exposure(
            self.node.get_parameter(f'{self.ns}.exposure_mode').get_parameter_value().string_value,
            self.node.get_parameter(f'{self.ns}.exposure_value').get_parameter_value().integer_value
        )

        # Register parameter callback for dynamic reconfiguration
        self.node.add_on_set_parameters_callback(self.on_parameter_change)

    def configure_exposure(self, mode, value):
        if not os.path.exists(self.device_path):
            self.node.get_logger().warn(f'{self.device_path} not found.')
            return

        try:
            if mode == 'manual':
                subprocess.run(['v4l2-ctl', '-d', self.device_path, '-c', 'auto_exposure=1'], check=True)
                subprocess.run(['v4l2-ctl', '-d', self.device_path, '-c', f'exposure_time_absolute={value}'], check=True)
                self.node.get_logger().info(f'{self.device_path}: Manual exposure set to {value}')
            else:
                subprocess.run(['v4l2-ctl', '-d', self.device_path, '-c', 'auto_exposure=3'], check=True)
                self.node.get_logger().info(f'{self.device_path}: Auto exposure enabled')
        except subprocess.CalledProcessError as e:
            self.node.get_logger().error(f'Failed to set exposure for {self.device_path}: {e}')

    def on_parameter_change(self, params):
        # Extract new parameter values from the callback arguments
        mode = None
        value = None
        updated = False
        for param in params:
            if param.name == f'{self.ns}.exposure_mode':
                mode = param.value
                updated = True
            elif param.name == f'{self.ns}.exposure_value':
                value = param.value
                updated = True

        # Use the most recent values: if not updated, use current
        if updated:
            if mode is None:
                mode = self.node.get_parameter(f'{self.ns}.exposure_mode').get_parameter_value().string_value
            if value is None:
                value = self.node.get_parameter(f'{self.ns}.exposure_value').get_parameter_value().integer_value
            try:
                self.configure_exposure(mode, value)
                return SetParametersResult(successful=True)
            except Exception as e:
                self.node.get_logger().error(f'Exposure reconfiguration failed: {e}')
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def run(self):
        if not self.running:
            self.node.get_logger().warn(f'{self.device_path} failed to open.')
            return

        self.node.get_logger().info(f'Starting stream: {self.device_path} → {self.topic_name}')
        while rclpy.ok() and self.running:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(msg)

    def stop(self):
        self.running = False
        self.cap.release()

class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_parallel_node')
        self.workers = []
        self.init_cameras()

    def init_cameras(self):
        for i in range(10):  # Scan /dev/video0 to /dev/video9
            dev_path = f'/dev/video{i}'
            if os.path.exists(dev_path):
                topic = f'/camera_{i}/image_raw'
                worker = CameraWorker(self, i, topic)
                self.workers.append(worker)
                worker.start()

    def destroy_node(self):
        for worker in self.workers:
            worker.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down multi-camera node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

