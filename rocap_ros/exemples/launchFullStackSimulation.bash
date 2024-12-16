#!/bin/bash

xterm -hold -T "simulationAndBridge" -e "ros2 launch rocap_ros rocap_demo.launch.py" &
sleep 10
xterm -hold -T "slam" -e "ros2 launch rocap_ros slam_lidar_rgbd.launch.py" &
xterm -hold -T "navigation" -e "ros2 launch rocap_ros navigation.launch.py" &
