"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration, with simulation time enabled."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions

def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[driver_params_file, {'use_sim_time': False}]
    )

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    convert_params['use_sim_time'] = False
    velodyne_transform_node = launch_ros.actions.Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[convert_params]
    )

    # Uncomment and modify the following lines if laserscan node is needed
    # laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    # laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    # velodyne_laserscan_node = launch_ros.actions.Node(
    #     package='velodyne_laserscan',
    #     executable='velodyne_laserscan_node',
    #     output='both',
    #     parameters=[laserscan_params_file, {'use_sim_time': True}]
    # )

    static_transform_publisher_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='both'
    )

    return launch.LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        # static_transform_publisher_node,
        # velodyne_laserscan_node,  # Uncomment if needed

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown()
                )],
            )
        ),
    ])
