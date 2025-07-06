from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Cart Control Node
        Node(
            package='cart_control',
            executable='cart_control_node',
            name='cart_control_node',
            output='screen',
            emulate_tty=True
        ),

        # 2. Rosbag Control Node
        Node(
            package='rosbag_control',
            executable='rosbag_control_node',
            name='rosbag_control_node',
            output='screen',
            emulate_tty=True
        ),

        # 3. System Monitor
        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen',
            emulate_tty=True
        ),

        # 4. Frequency Monitor
        Node(
            package='system_monitor',
            executable='frequency_monitor_node',
            name='frequency_monitor_node',
            output='screen',
            emulate_tty=True
        ),

        # 5. Rosbridge WebSocket
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('rosbridge_server').find('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                )
            ])
        ),
    ])

