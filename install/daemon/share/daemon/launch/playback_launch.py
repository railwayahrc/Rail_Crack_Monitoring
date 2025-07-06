from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace



# ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
def generate_launch_description():
    ld = LaunchDescription()

    zed1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'camera1',
            #'serial_number':'32513710',
            'svo_path':'/home/jyo/Recordings/test/SVO_SN32513710.svo2'
        }.items()
    )


    zed2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'camera2',
            #'serial_number':'30281178',
            'svo_path':'/home/jyo/Recordings/test/SVO_SN30281178.svo2'
        }.items()
    )
# ros2 launch synexens_ros2 driver_launch.py
    cs20 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('synexens_ros2'),
                'launch',
                'driver_launch.py'
            ])
        ])
    )

# ros2 launch ublox_gps ublox_gps_node-launch.py

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ublox_gps'),
                'launch',
                'ublox_gps_node-launch.py'
            ])
        ])
    )
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ])
    )


#     ld_group = GroupAction(
#      actions=[
#          PushRosNamespace('Jetson2'),
#          zed,
#          cs20
#       ]
#    )
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', '/home/jyo/Recordings/my_bag', '/topic1', '/topic2'],
        output='screen'
    )

    ld.add_action(zed1)
    ld.add_action(zed2)
    ld.add_action(rosbag_record)
    # ld.add_action(cs20)
    # ld.add_action(gps)
    # ld.add_action(ld_group)
    # ld.add_action(rosbridge)

    return ld