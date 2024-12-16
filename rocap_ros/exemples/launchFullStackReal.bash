gnome-terminal --tab --title Bridge -- ros2 launch rocap_ros rocap_bridge.launch.py
sleep 5
gnome-terminal --tab --title slam -- ros2 launch rocap_ros slam_lidar_rgbd.launch.py  use_sim_time:=false
gnome-terminal --tab --title navigation -- ros2 launch rocap_ros navigation.launch.py use_sim_time:=false