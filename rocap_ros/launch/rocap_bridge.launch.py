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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():

    # Get all file path
    rocap_pkg = os.path.join(
        get_package_share_directory('rocap_ros'))

    permission_file = os.path.join(rocap_pkg,
                              'permissions',
                              'signed_permissions.json')

   
    bridge = Node(
        package='rocap_ros',
        executable='ros2_api_bridge.py',
        output='screen',
        parameters=[{'base_url': LaunchConfiguration('base_url'),
                     'use_mock':LaunchConfiguration('use_mock'),
                     'use_sim_time':LaunchConfiguration('use_sim_time'),
                     'permissions_file_path': permission_file}],
        emulate_tty=True
    )

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
               os.path.join(rocap_pkg, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use simulation time'),
        DeclareLaunchArgument(
            'base_url',
            default_value='http://192.168.100.203:5577', #TODO verify if robot change
            description='base URL of Rocap API'),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='use mock API (for simulation)'),
        robot_state_publisher,
        bridge,
    ])
