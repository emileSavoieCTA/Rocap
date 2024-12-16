#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/install/local_setup.bash

# Start a new tmux session named "ros_nodes"
tmux new-session -d -s ros_nodes

# Create a pane for the Bridge node
tmux rename-window -t ros_nodes "ROS Nodes"
tmux send-keys -t ros_nodes "ros2 launch rocap_ros rocap_demo.launch.py" C-m
sleep 5

# Create a new pane for the SLAM node
tmux split-window -h -t ros_nodes
tmux send-keys -t ros_nodes "ros2 launch rocap_ros slam_lidar_rgbd.launch.py use_sim_time:=true" C-m

# Create another pane for the Navigation node
tmux split-window -v -t ros_nodes:0.1
tmux send-keys -t ros_nodes "ros2 launch rocap_ros navigation.launch.py rtabmap_viz:=false use_sim_time:=true" C-m

# Arrange panes if desired and attach to session
tmux select-layout -t ros_nodes tiled
tmux attach -t ros_nodes
