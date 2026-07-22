#!/bin/bash

SESSION=GRASPING_PIPELINE
FILE_PATH=${BASH_SOURCE[0]}

SCRIPT_DIR=$(dirname "$0")
echo "Script directory is: $SCRIPT_DIR"

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -h


tmux select-pane -t 0
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_statemachine.launch.py"

tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "source $SCRIPT_DIR/../../../devel/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_server.launch.py"
# tmux split-window -v
# tmux select-pane -t 2
# tmux send-keys "hsrb_mode" C-m
# tmux send-keys "source $SCRIPT_DIR/../../../devel/setup.bash" C-m
# tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_params.launch"

tmux select-pane -t 3
tmux send-keys "hsrb_mode" C-m
tmux send-keys rv
tmux split-window -v
tmux select-pane -t 4
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "startup && table"

tmux select-window -t $SESSION:1

tmux select-pane -t 0
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "source ~/demos/devel/setup.bash"
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys enter
tmux send-keys "ros2 launch hsrb_moveit_config hsrb_demo.launch.py"

tmux select-window -t $SESSION:0
tmux select-pane -t 2

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION
