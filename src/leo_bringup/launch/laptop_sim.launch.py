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
    
    # Launch argument
    use_converter_arg = DeclareLaunchArgument(
        'use_pointcloud_converter',
        default_value='true',
        description='Convert pointcloud to laserscan'
    )
    use_converter = LaunchConfiguration('use_pointcloud_converter')
    
    # Include Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_gazebo'), 'launch', 'leo_gazebo_launch.py')])
    )
    
    # Include SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_slam'), 'launch', 'slam_launch.py')])
    )
    
    # Include Nav2 (with delay to wait for robot spawn)
    nav2_launch = TimerAction(
        period=5.0,  # Wait 5 seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('leo_nav2'), 'launch', 'nav2_launch.py')])
            )
        ]
    )
    
    # Include Teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_teleop'), 'launch', 'controller_teleop.launch.py')])
    )
    
    # Pointcloud to laserscan converter (conditional)
    converter_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'min_height': -0.1,
            'max_height': 0.5,
            'range_min': 0.25,
        }],
        remappings=[
            ('cloud_in', '/velodyne_points'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(use_converter)
    )
    
    return LaunchDescription([
        use_converter_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        teleop_launch,
        converter_node,
    ])