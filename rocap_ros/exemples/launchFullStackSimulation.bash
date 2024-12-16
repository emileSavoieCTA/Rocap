#!/bin/bash

# Start Bridge node
xterm -hold -title "Bridge" -e "ros2 launch rocap_ros rocap_bridge.launch.py" &
sleep 5

# Start SLAM node
xterm -hold -title "slam" -e "ros2 launch rocap_ros slam_lidar_rgbd.launch.py rtabmap_viz:=false use_sim_time:=true" &

# Start Navigation node
xterm -hold -title "navigation" -e "ros2 launch rocap_ros navigation.launch.py use_sim_time:=true" &
