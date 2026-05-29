#!/bin/bash
SESSION=GRASPING_PIPELINE

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_statemachine.launch.py"

tmux select-pane -t 1
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 run grasping_pipeline userinput_publisher"

tmux select-pane -t 2
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_server.launch.py"

tmux select-window -t $SESSION:1
tmux split-window -h
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch hsrb_moveit_config move_group.launch.py"

tmux select-pane -t 1
tmux send-keys "source /root/ros2_ws/install/setup.bash" C-m
tmux send-keys "source /root/hsr_ros2_ws/install/setup.bash" C-m
tmux send-keys "export ROS_DOMAIN_ID=0" C-m
tmux send-keys "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" C-m
tmux send-keys "export CYCLONEDDS_URI=/root/config/cyclonedds_profile.xml" C-m
tmux send-keys "ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py"

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION
