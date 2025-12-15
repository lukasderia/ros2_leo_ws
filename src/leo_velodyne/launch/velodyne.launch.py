import os
import yaml
import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Driver node (unchanged from default)
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[driver_params_file]
    )

    # Convert node (using custom config with min_range: 0.5)
    custom_config_dir = ament_index_python.packages.get_package_share_directory('leo_velodyne')
    convert_params_file = os.path.join(custom_config_dir, 'config', 'VLP16-velodyne_convert_node-params.yaml')
    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    
    velodyne_convert_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='both',
        parameters=[convert_params]
    )

    # Laserscan node (unchanged from default)
    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        parameters=[laserscan_params_file]
    )

    return LaunchDescription([
        velodyne_driver_node,
        velodyne_convert_node,
        velodyne_laserscan_node,
    ])