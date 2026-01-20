import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import TimerAction


# Full launcher without the exploration
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

    # Include Nav2 Launcher
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_nav2'), 'launch', 'nav2_launch.py')])
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

    # Include the custom launcher
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_velodyne'), "launch", "velodyne.launch.py")])
    )

    return LaunchDescription([
        odom2TF_node,
        #teleop_launch,
        velodyne_launch,
        ScanFilter,
        #pointcloud_to_laserscan_node,
        slam_launch,
        nav_launch,
    ])
