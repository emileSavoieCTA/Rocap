"""
This launch file launches the driver to interpret and publish data from the VLP16 lidar
Launch Arguments:
 This launch file does not take in any arguments
Usage:
 ros2 launch drone_cta_auxilliary velodyne.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Define parameters to pass to the Velodyne launch file
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('velodyne'), 'launch'),
            '/velodyne-all-nodes-VLP16-launch.py'
        ]),
        launch_arguments={
            'points_topic': '/ray/points'  # Remap the points topic
        }.items()
    )

    # Static transform publisher node
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_static_transform',
        output='screen',
        arguments=[
            '0.0',  # x translation
            '0.0',  # y translation
            '0.0',  # z translation
            '0.0',  # x rotation (radians)
            '0.0',  # y rotation (radians)
            '0.0',  # z rotation (radians)
            'base_link',  # parent frame
            'velodyne'    # child frame
        ]
    )

    return LaunchDescription([
        velodyne_launch,
        static_transform_node  # Include the static transform publisher
    ])