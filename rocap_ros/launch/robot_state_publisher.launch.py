# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    # Extract script parameter
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get all file path
    rocap_demo_path = os.path.join(
        get_package_share_directory('rocap_ros'))
    
    xacro_robot_file = os.path.join(rocap_demo_path,
                              'urdf',
                              'rocap','rocap.urdf.xacro')
    
    robot_urdf_file = os.path.join(rocap_demo_path,
                              'urdf',
                              'rocap','rocap.urdf')

    # generate urdf
    doc = xacro.parse(open(xacro_robot_file))
    xacro.process_doc(doc)

    f = open(robot_urdf_file, "w")
    f.write(doc.toprettyxml())
    f.close()

    # Create launch description
    params = {'robot_description': doc.toxml(),
              'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_description",
        output='screen',
        parameters=[params]    
        
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use simulation time'),
        node_robot_state_publisher
    ])
