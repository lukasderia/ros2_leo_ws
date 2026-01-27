import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Include Robot Description
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_description'), 'launch', 'real.launch.py')])
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

    # Include scan_filter
    ScanFilter = Node(
        package='leo_utils',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen'
    )

    # Include the exploration launcher WITH the argument
    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_exploration'), "launch", "exploration.launch.py")]),
        launch_arguments={'odom_topic': '/odometry_merged'}.items()
    )

    # Include Rviz2 with saved configuration
    rviz_config = os.path.join(
        get_package_share_directory('leo_bringup'),
        'rviz', 'real_launch_rviz.rviz'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        description_launch,
        odom2TF_node,
        ScanFilter,
        slam_launch,
        nav_launch,
        exploration_launch,
    ])