#!/bin/bash

SESSION="cart_system"

# Kill existing session if exists
tmux kill-session -t $SESSION 2>/dev/null

# Start new detached tmux session
tmux new-session -d -s $SESSION -n ROS

# Pane 0: Cart Control
tmux send-keys -t $SESSION:0 'source ~/ros2_ws/install/setup.bash && ros2 run cart_control cart_control_node' C-m

# Pane 1: Rosbag Control
tmux split-window -v -t $SESSION:0
tmux send-keys -t $SESSION:0.1 'source ~/ros2_ws/install/setup.bash && ros2 run rosbag_control rosbag_control_node' C-m

# Pane 2: System Monitor
tmux split-window -h -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 'source ~/ros2_ws/install/setup.bash && ros2 run system_monitor system_monitor_node' C-m

# Pane 3: Frequency Monitor
tmux split-window -v -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 'source ~/ros2_ws/install/setup.bash && ros2 run system_monitor frequency_monitor_node' C-m

# Pane 4: rosbridge_server
tmux split-window -h -t $SESSION:0.3
tmux send-keys -t $SESSION:0.4 'source ~/ros2_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml' C-m

# Resize for better layout
tmux select-layout -t $SESSION:0 tiled

# Attach to session
tmux attach -t $SESSION
