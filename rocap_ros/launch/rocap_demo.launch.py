#!/usr/bin/env python3
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def launch_with_context(context, *args, **kwargs):

    # Extract script parameter
    with_rope = LaunchConfiguration('with_rope').perform(context).lower()

    # Get all file path
    rocap_pkg = os.path.join(
        get_package_share_directory('rocap_ros'))

    world_file = os.path.join(rocap_pkg,
                              'world',
                              'basicHouse.world')

    xacro_sim_file = os.path.join(rocap_pkg,
                              'urdf',
                              'rocap_flying_demo','rocap_flying_demo.urdf.xacro')
    
    xacro_sim_urdf_file = os.path.join(rocap_pkg,
                              'urdf',
                              'rocap_flying_demo','rocap_flying_demo.urdf')
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    
    # Create launch description
    gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose':"true",
                              'pause': 'false'}.items(),
       )

    gz_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
               os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
    
    rocap_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
               os.path.join(rocap_pkg, 'launch', 'rocap_bridge.launch.py')
            ),
            launch_arguments={'use_mock': 'true',
                              'use_sim_time': 'true'}.items(),
        )

    doc_sim = xacro.parse(open(xacro_sim_file))
    xacro.process_doc(doc_sim,mappings={'with_rope':with_rope})

    f = open(xacro_sim_urdf_file, "w")
    f.write(doc_sim.toprettyxml())
    f.close()


    params = {'robot_description': doc_sim.toxml(),
              'use_sim_time': True}

    node_sim_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="sim_description",
        output='screen',
        parameters=[params],
        remappings=[
          ('/tf', '/sim/tf'),
          ('/tf_static', '/sim/tf_static'),
          ('/robot_description', '/sim/robot_description')]
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/sim/robot_description',
                                   '-entity', 'rocap'],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )



    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_velocity_controller],
            )
        ),
        gz_server,
        gz_client,
        node_sim_state_publisher,
        spawn_entity,
        rocap_bridge,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_rope',
            default_value='false',
            description='launch simulation with rope plugin'),

        OpaqueFunction(function = launch_with_context),
    ])
