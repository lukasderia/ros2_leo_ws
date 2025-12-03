import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # Include Teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_teleop'), 'launch', 'controller_teleop.launch.py')])
    )

    # Include SLAM Launcher
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_slam'), 'launch', 'slam_launch.py')])
    )

    # Include Odom2TF
    odom2TF_node = Node(
        package='leo_utils',
        executable='odom2TF.py',
        name='odom2TF',
        output='screen'
    )    

    # Inclde scan_filter
    ScanFilter = Node(
        package='leo_utils',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen'
    )

    # # Pointcloud to LaserScan conversion
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     remappings=[
    #         ('cloud_in', '/velodyne_points'),
    #         ('scan', '/scan_filter')
    #     ],
    #     parameters=[{
    #         'min_height': -0.15,
    #         'max_height': 0.25,
    #         'range_min': 0.25,
    #         'range_max': 100.0,
    #     }]
    # )

    # Include the velodyne-all-nodes
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne'), "launch", "velodyne-all-nodes-VLP16-launch.py")])
    )

    return LaunchDescription([
        odom2TF_node,
        #teleop_launch,
        velodyne_launch,
        ScanFilter,
        #pointcloud_to_laserscan_node,
        slam_launch,
    ])
