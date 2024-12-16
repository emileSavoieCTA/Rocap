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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

def generate_launch_description():

    
    slamParams={
          'frame_id':"base_link",
          'use_sim_time':LaunchConfiguration('use_sim_time'),
          'subscribe_rgb':LaunchConfiguration('use_camera'),
          'subscribe_depth':LaunchConfiguration('use_camera'),
          'use_action_for_goal':True,
          'Grid/RayTracing':"true",
          'Grid/CellSize':"0.15",
          'Grid/ClusterRadius': "0.30",
          'Grid/MinClusterSize': "5",
          'Grid/RangeMax':"70.0",
          'Grid/RangeMin':"1.0",
          'Grid/MaxGroundAngle':"45",
          'Grid/3D=true':True,
          'grid/fromDepth':False,
          'odom_sensor_sync':True,
          'scan_range_min':1.0,
          'scan_range_max':50.0,
          'map_always_update':True, 
          'qos':1,
          'publish_tf':True,
          'wait_imu_to_init':LaunchConfiguration('wait_for_imu'),
          'wait_for_transform_duration':True,
          'approx_sync_max_interval':0.001,
          'subscribe_scan_cloud': True,      
    }

    ipcOdomParams = {
          'frame_id':"base_link",
          'odom_frame_id ':'odom',
          'use_sim_time':LaunchConfiguration('use_sim_time'),
          'scan_range_min':1.0,
          'scan_range_max':50.0,
          'qos':1,
          'publish_tf':True,
          'wait_imu_to_init':LaunchConfiguration('wait_for_imu'),
          'wait_for_transform_duration':True,
          'approx_sync_max_interval':0.001,
          'subscribe_scan_cloud': True,
          'Icp/VoxelSize' :"0.1",
          'Icp/Iterations': "10",
          'Icp/MaxTranslation': '2',
          'Icp/MaxCorrespondenceDistance': "1",
          'Icp/CorrespondenceRatio':"0.01",
          'Icp/Epsilon':"0.001",
          'Odom/ResetCountdown':"1"
    }

    remappingsOdom=[
          ('rgb/image', LaunchConfiguration('camera_image_topic')),
          ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
          ('depth/image', LaunchConfiguration('depth_image_topic')),
          ('scan_cloud',  LaunchConfiguration('scan_cloud_topic')),
          ('imu', LaunchConfiguration('imu_topic'))]
    
    remappingslam=[
          ('rgb/image', LaunchConfiguration('camera_image_topic')),
          ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
          ('depth/image', LaunchConfiguration('depth_image_topic')),
          ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
          ('imu', LaunchConfiguration('imu_topic'))]
    
    odom = Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[ipcOdomParams],
                remappings=remappingsOdom,
                condition=IfCondition(LaunchConfiguration('lidar_odom')))
            

    launchSlam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slamParams],
            remappings=remappingslam,
            arguments=["-d"])
    
    rtabmap_viz = Node(
            package='rtabmap_viz', executable='rtabmap_viz', name="rtabmap_viz", output='screen',
            parameters=[slamParams],
            remappings=remappingslam,
            arguments=["--delete_db_on_start"],
            condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use simulation time'),
        DeclareLaunchArgument(
            'scan_cloud_topic',
            # default_value='/ray/points',
            default_value='/velodyne_points_modified',
            description='input scan cloud topic'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='input imu topic'),
        DeclareLaunchArgument(
            'wait_for_imu',
            default_value='true',
            description='wait for imu to init to start slam'),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/cam/depth/image_raw',
            description='input depth image topic'),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/cam/image_raw',
            description='input camera image topic'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/cam/camera_info',
            description='input camera info topic'),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='define if a depth camera is used for slam'),
        DeclareLaunchArgument(
            'lidar_odom',
            default_value='false',
            description='using IPC odometry instead of external odom'),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='define if we are using rtabmap viewer'),
        odom,
        launchSlam,
        rtabmap_viz
    ])
