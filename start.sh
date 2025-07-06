#!/bin/bash

SESSION="ros_multi"

# Start new tmux session with the first command
tmux new-session -d -s $SESSION -n multi_camera 'source ~/.bashrc && ros2 run multi_camera_pkg multi_camera_publisher; exec bash -i'

# Add windows for each subsequent command
tmux new-window -t $SESSION:1 -n ublox_gps 'source ~/.bashrc && ros2 launch ublox_gps ublox_gps_node-launch.py param_file:=/home/jyo/ros2_ws/ublox_config/my_f9p.yaml; exec bash -i'
tmux new-window -t $SESSION:2 -n sys_mon 'source ~/.bashrc && ros2 run system_monitor system_monitor_node; exec bash -i'
tmux new-window -t $SESSION:3 -n freq_mon 'source ~/.bashrc && ros2 run system_monitor frequency_monitor_node; exec bash -i'
tmux new-window -t $SESSION:4 -n rosbag_ctrl 'source ~/.bashrc && ros2 run rosbag_control rosbag_control_node; exec bash -i'
tmux new-window -t $SESSION:5 -n cart_ctrl 'source ~/.bashrc && ros2 run cart_control cart_control_node; exec bash -i'
tmux new-window -t $SESSION:6 -n rosbridge 'source ~/.bashrc && ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash -i'
tmux new-window -t $SESSION:7 -n web_video 'ros2 run web_video_server web_video_server; exec bash -i'
tmux new-window -t $SESSION:8 -n http_server 'cd ~/ros2_ws/web_ui && python3 -m http.server 8000; exec bash -i'
tmux new-window -t $SESSION:9 -n uvicorn 'cd ~/ros2_ws/web_ui/ && uvicorn set_param_server:app --host 0.0.0.0 --port 8001; exec bash -i'
tmux new-window -t $SESSION:10 -n browser 'xdg-open http://localhost:8000/control5.html; exec bash -i'

# Attach to the session
tmux attach-session -t $SESSION
