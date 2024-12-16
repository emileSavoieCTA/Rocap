from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node

def generate_launch_description():
    # SLAM Parameters
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
    
    remappingslam = [
        ('rgb/image', LaunchConfiguration('camera_image_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_image_topic')),
        ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
        ('imu', LaunchConfiguration('imu_topic'))
    ]
    
    # Create launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='use simulation time'),
        DeclareLaunchArgument(
            'scan_cloud_topic',
            default_value='/ray/points',
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
            default_value='false',
            description='define if a depth camera is used for slam'),
        DeclareLaunchArgument(
            'lidar_odom',
            default_value='false',
            description='using IPC odometry instead of external odom'),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='define if we are using rtabmap viewer'),
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='If true, use ray/points; otherwise, use assembled_cloud')
    ]
    
    # SLAM Node
    launchSlam = Node(
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        parameters=[slamParams],
        remappings=remappingslam,
        arguments=["-d"]
    )
    
    # Scan cloud topic selector
    scan_cloud_topic_selector = [
        SetLaunchConfiguration(
            'scan_cloud_topic', 'ray/points', condition=IfCondition(LaunchConfiguration('sim'))
        ),
        SetLaunchConfiguration(
            # 'scan_cloud_topic', 'assembled_cloud', condition=UnlessCondition(LaunchConfiguration('sim'))
            'scan_cloud_topic', 'assembled_cloud', condition=UnlessCondition(LaunchConfiguration('sim'))
        )
    ]
    
    # Combine all elements
    launch_description_elements = launch_arguments + scan_cloud_topic_selector + [launchSlam]
    
    return LaunchDescription(launch_description_elements)