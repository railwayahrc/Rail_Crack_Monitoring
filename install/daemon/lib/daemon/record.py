#!/usr/bin/env python3
import rclpy
import psutil
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import Trigger 
import os
import re
import csv
import signal
import subprocess
import threading
from datetime import datetime
from rosidl_runtime_py.utilities import get_message
from rclpy.executors import MultiThreadedExecutor

base_path = "/home/jyo/CS2303DATA"
log_file_path = os.path.join(base_path, 'log.csv')

# Terminate the parent process and all its child processes
def terminate_process_and_children(parent_pid):
    try:
        parent = psutil.Process(parent_pid)
        for child in parent.children(recursive=True):  # Terminate all child processes
            child.terminate()
        parent.terminate()  # Terminate the parent process
        parent.wait(5)  # Wait for the parent process to terminate
    except psutil.NoSuchProcess:
        pass

def human_readable_size(size, decimal_places=2):
    for unit in ['B', 'KB', 'MB', 'GB', 'TB', 'PB']:
        if size < 1024:
            return f"{size:.{decimal_places}f} {unit}"
        size /= 1024

def get_folder_size(folder_path):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(folder_path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            total_size += os.path.getsize(fp)
    return total_size

def get_next_session_folder(base_path):
    # Get a list of all directories in the base path
    dirs = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    
    # Extract session numbers
    session_numbers = []
    for d in dirs:
        match = re.match(r'CS2303-(\d+)', d)
        if match:
            session_numbers.append(int(match.group(1)))

    # Find the next session number
    if session_numbers:
        next_session_number = max(session_numbers) + 1
    else:
        next_session_number = 1

    # Create the new session folder name
    new_session_folder = f"CS2303-{next_session_number:03d}"
    new_session_path = os.path.join(base_path, new_session_folder)

    return new_session_path

def create_new_session_folder(base_path):
    new_session_path = get_next_session_folder(base_path)
    os.makedirs(new_session_path)
    
    # Create a README file with the current date and time
    readme_path = os.path.join(new_session_path, 'README.md')
    with open(readme_path, 'w') as f:
        start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        f.write(f"Session started at: {start_time}\n")
    
    return new_session_path, start_time

def update_readme_with_stop_info(recording_location, start_time):
    readme_path = os.path.join(recording_location, 'README.md')
    stop_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # Calculate the duration
    start_time_dt = datetime.strptime(start_time, '%Y-%m-%d %H:%M:%S')
    stop_time_dt = datetime.strptime(stop_time, '%Y-%m-%d %H:%M:%S')
    duration = stop_time_dt - start_time_dt
    
    with open(readme_path, 'a') as f:
        f.write(f"Session stopped at: {stop_time}\n")
        f.write(f"Recording duration: {duration}\n")
        f.write("Recording successfully stopped.\n")

    # Update the log.csv file
    folder_size = get_folder_size(recording_location)
    human_readable_folder_size = human_readable_size(folder_size)
    session_name = os.path.basename(recording_location)

    if not os.path.exists(log_file_path):
        with open(log_file_path, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Sesion-id','Date', 'Start Time', 'Stop Time', 'Duration', 'File Size'])
    with open(log_file_path, 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([session_name,start_time.split()[0], start_time.split()[1], stop_time.split()[1], str(duration), human_readable_folder_size])
    return 

class RosbagController(Node):
    def __init__(self):
        super().__init__('rosbag_controller')
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics",rclpy.Parameter.Type.STRING_ARRAY)
            ]
        )
        self.is_recording = False
        self.process_bag = None
        self.process_svo = None
        self.process_rpisr = None
        self.process_rpist = None
        self.recording_location = None
        self.recording_start_time = None
        self.start_service = self.create_service(Trigger, 'start_recording', self.handle_start)
        self.stop_service = self.create_service(Trigger, 'stop_recording', self.handle_stop)
        self.interrupt_pub = self.create_publisher(String, 'recording_interrupted', 10)

        self.topics = self.get_parameter("topics").get_parameter_value().string_array_value
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.get_logger().info("Topic recorder is up")

        self.topic_types = None
        self.last_received = None
        self.wait_to_check = None
        self.check_timer = None

    def start_topic_chacker(self):
        # Get list of all published topics
        all_topics = self.get_topic_names_and_types()
        self.get_logger().info("all_topics:" + str(all_topics))
        # Create a dictionary mapping topic names to types
        self.topic_types_helper = {name: type for name, type in all_topics}
        # Filter the dictionary to only include the topics we are interested in
        self.topic_types = {topic: self.topic_types_helper[topic] for topic in self.topics if topic in self.topic_types_helper}
        self.get_logger().info("topic_types : " + str(self.topic_types))
        self.last_received = {topic: self.get_clock().now() for topic in self.topics}
        self.wait_to_check = 1000
        self.check_timer = self.create_timer(5.0, self.check_topics)


    def sigint_handler(self, signum, frame):
        self.get_logger().info("SIGINT received, stopping recording")
        if self.is_recording:
            self.get_logger().info("SIGINT is stoping recording.")
            request = Empty.Request()
            response = Trigger.Response()
            self.handle_stop(request, response)
        rclpy.shutdown()

    def check_topics(self):
        if self.is_recording and self.wait_to_check == 0:
            now = self.get_clock().now()
            for topic, last_time in self.last_received.items():
                if (now - last_time).nanoseconds * 1e-9 > 5.0:
                    self.interrupt_pub.publish(String(data=topic))
                    self.get_logger().info("Recording Stopped by : %s" % topic)
                    request = Empty.Request()
                    response = Trigger.Response()
                    self.get_logger().info("Check_topic is stoping recording.")
                    self.handle_stop(request,response)
                    break
        elif self.wait_to_check > 0:
            self.wait_to_check = self.wait_to_check - 1

    def handle_start(self, req, res):
        subprocess.Popen(['ros2','service','call','/start_rpi_recording','std_srvs/srv/Trigger'],shell=False)
        self.get_logger().info("start recording handle is called")
        self.recording_location,self.recording_start_time = create_new_session_folder(base_path)
        if not self.is_recording:
            self.is_recording = True
            self.subscribers = []
            self.wait_to_check = 6
            self.bag = self.recording_location + '/GPSandTOF.bag'
            self.process_bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', self.bag] + self.topics,shell=False)
            self.get_logger().info("ROSbag recording started in process id: "+ str(self.process_bag.pid)) # since pid is INT
            self.process_svo = subprocess.Popen(['ros2','run','daemon','svo_recording_multicam','1','15',self.recording_location],shell=False)
            # print("SVO Re>>>>>>>>>>>>>"self.process_svo.pid)
            self.get_logger().info("SVO recording started in process id: "+ str(self.process_svo.pid)) # since pid is INT
            # self.process_rpisr = subprocess.Popen(['ros2','service','call','/start_rcording','std_srvs/srv/Trigger'],shell=False)
           
            self.get_logger().info("RPi recording started") # since pid is INT
            
            self.start_topic_chacker()
            for topic, topic_type in self.topic_types.items():
                self.get_logger().info("I am in the loop")
                msg_class = get_message(topic_type[0])
                sub = self.create_subscription(msg_class, topic,lambda msg: self.handle_message(msg,topic), 10)
                self.subscribers.append(sub)
        res.success = True
        res.message = self.recording_location
        return res

    def handle_message(self, msg, topic):
        if self.is_recording:
            self.last_received[topic] = self.get_clock().now()

    def handle_stop(self, req, res):
        subprocess.Popen(['ros2','service','call','/stop_rpi_recording','std_srvs/srv/Trigger'],shell=False)
        self.get_logger().info("stop recording handle is called")
        if self.is_recording:
            for sub in self.subscribers:
                self.destroy_subscription(sub)
                print("it worked")
            self.subscribers = []
            self.is_recording = False
            if self.process_bag is not None:
                self.get_logger().info("bag: process id: "+str(self.process_bag.pid)+" Send SIGINT")
                # self.process_bag.send_signal(signal.SIGINT)
                terminate_process_and_children(self.process_bag.pid)
                #os.killpg(self.process_bag.pid, signal.SIGINT)
                self.process_bag = None
            if self.process_svo is not None:
                self.get_logger().info("SVO: process id: "+str(self.process_svo.pid)+" Send SIGINT")
                # os.killpg(self.process_svo.pid, signal.SIGINT)
                # self.process_svo.send_signal(signal.SIGINT)
                terminate_process_and_children(self.process_svo.pid)
                self.process_svo = None
            # 
            # if self.process_rpisr is not None:
            #     self.process_rpist = subprocess.Popen(['ros2','service','call','/stop_rcording','std_srvs/srv/Trigger'],shell=False)
            #     self.get_logger().info("RPi recording Stopped in process id: "+ str(self.process_rpisr.pid)) # since pid is INT
            #     # self.get_logger().info("RpiSt: process id: "+str(self.process_rpist.pid)+" Send SIGINT")
            #     # # os.killpg(self.process_rpist.pid, signal.SIGINT)
            #     # # self.process_rpist.send_signal(signal.SIGINT)
            #     # terminate_process_and_children(self.process_rpist.pid)
            #     terminate_process_and_children(self.process_rpisr.pid)
            #     terminate_process_and_children(self.process_rpist.pid)
            #     self.process_rpisr = None
            #     self.process_rpist = None
            update_readme_with_stop_info(self.recording_location,self.recording_start_time)
        
        res.success = True
        res.message = "recording stoped"
        return res

def main(args=None):
    rclpy.init(args=args)
    controller = RosbagController()
    executor = MultiThreadedExecutor()

    signal.signal(signal.SIGINT, lambda signum, frame: controller.sigint_handler(signum, frame))

    rclpy.spin(controller, executor=executor)
    controller.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
