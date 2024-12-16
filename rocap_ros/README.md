# rocap_ros Package
Ros package that provide simulation and bridge to use the rocap robot

## Setup package

Install rtabmap or follow the step to build it from source

```bash

sudo apt install ros-$ROS_DISTRO-rtabmap-ros

```


Clone all dependency in your ros2 workspace src folder

```bash

git clone -b $ROS_DISTRO https://github.com/ros-controls/gazebo_ros2_control.git

git clone -b $ROS_DISTRO https://github.com/ros-controls/ros2_control.git

git clone git@github.com:cegepmontpetit/ROS2_CTA.git

git clone git@github.com:cegepmontpetit/22CTA034_ROS2_Package.git

```

Go to your inside your ros2 workspace, install dependency and build packages

```bash

cd <ros2_ws>

rosdep update && rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source <ros2_ws>/install/local_setup.bash

```

If it's not already done, source your workspace in your bashrc


```bash

echo "source <ros2_ws>/install/local_setup.bash" >> ~/.bashrc

```

### Build and install the rocap-rest-api-client

- install [openapi-python-client](https://github.com/openapi-generators/openapi-python-client)

```bash
pip install openapi-python-client
echo "source ~/.bash_completions/openapi-python-client.sh" >> ~/.bashrc
```

- install poetry 

```bash
pip install poetry 
```

- build client openapi-python-client generate --path openapi.json

```bash
cd <pathToThisPackage>/node/api_bridge
openapi-python-client generate --path openapi.json
```

- build python package

```bash
cd <pathToThisPackage>/node/api_bridge/rocap-rest-api-client
poetry build -f wheel
```

- install builded package 

```bash
cd <pathToThisPackage>/node/api_bridge/rocap-rest-api-client/dist
pip install <package>.whl
```

 > **_NOTE:_**  When the Rocap API will be stable a python package globaly available should be made and add as depend in the package.xml to remove this step


## Launch simulation


### Launch simulation demo without the rope

```bash
ros2 launch rocap_ros rocap_demo.launch.py
```

### Launch simulation demo with rope

```bash
ros2 launch rocap_ros rocap_demo.launch.py with_rope:=True
```

 > **_NOTE:_**  this demo is more heavy and require better hardware to run

## Starting SLAM

You can start SLAM with the following command : 

```bash
ros2 launch rocap_ros slam_lidar_rgbd.launch.py use_sim_time:=<true if using simulation false if not>'
```
Note :
- The topic name should match the topic published by the sensor.
- You may need to set the parameters `use_camera` and `wait_for_imu` to false if you are not using those sensors.

## Starting Navigation

You can start Navigation with the following command : 

```bash
ros2 launch rocap_ros navigation.launch.py use_sim_time:=<true if using simulation false if not>'
```
You can now move the Rocap by setting a navigation goal in `rviz2`

## Using real Rocap

Before using the real Rocap, ensure you have completed the following steps:

1. Build and install the `rocap-rest-api-client` Python package.
2. Request a `signed_permissions.json` file from the Rocap administrator.
3. Place the `signed_permissions.json` file in the `permissions` folder.
4. Determine the IP address and communication port of the Rocap you want to control.

Once you have completed the above steps, you can start the API bridge to control the Rocap through ROS using the following command:

```bash
ros2 launch rocap_ros rocap_bridge base_url:='http://<rocapIP>:<rocapPort>'
```

## Code scripts exemples

This package provides two bash scripts to demonstrate how to launch the full stack in both real and simulation environments. See the `examples` folder for more details.


## Know package issues

- You must use a numpy version less than 1.24 to run the bridge, as the tf_transformations package contains some deprecated numpy code