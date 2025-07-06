from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arduino_port = LaunchConfiguration('arduino_port')
    recorded_topics = LaunchConfiguration('recorded_topics')
    monitored_topics = LaunchConfiguration('monitored_topics')

    rosbridge_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
            description='Serial port connected to Arduino'
        ),
        DeclareLaunchArgument(
            'recorded_topics',
            default_value='',
            description='Semicolon-separated list of topics to record. Empty = all.'
        ),
        DeclareLaunchArgument(
            'monitored_topics',
            default_value='',
            description='Semicolon-separated list of topics to monitor.'
        ),

        Node(
            package='cart_control',
            executable='cart_control_node',
            name='cart_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'device': arduino_port}]
        ),

        Node(
            package='rosbag_control',
            executable='rosbag_control_node',
            name='rosbag_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'topics': recorded_topics}]
        ),

        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='system_monitor',
            executable='frequency_monitor_node',
            name='frequency_monitor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'topics': monitored_topics}]
        ),

        IncludeLaunchDescription(
    	    XMLLaunchDescriptionSource(rosbridge_launch_file)
	)
    ])

