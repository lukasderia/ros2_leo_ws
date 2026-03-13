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
    
    # Launch arguments
    use_converter_arg = DeclareLaunchArgument(
        'use_pointcloud_converter',
        default_value='true',
        description='Convert pointcloud to laserscan'
    )
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0', description='Robot spawn x position')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0', description='Robot spawn y position')
    router_x_arg = DeclareLaunchArgument('router_x', default_value='18.0', description='Router x position')
    router_y_arg = DeclareLaunchArgument('router_y', default_value='18.0', description='Router y position')
    stddev_arg = DeclareLaunchArgument('stddev', default_value='1.0', description='Standard deviation on wifi signal')
    mode_arg = DeclareLaunchArgument('mode', default_value='2', 
    description='Exploration mode: 0=yamauchi, 1=gao, 2=rss')

    use_converter = LaunchConfiguration('use_pointcloud_converter')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    router_x = LaunchConfiguration('router_x')
    router_y = LaunchConfiguration('router_y')
    stddev = LaunchConfiguration('stddev')
    mode = LaunchConfiguration('mode')
    
    # Include Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_gazebo'), 'launch', 'leo_gazebo_launch.py')]),
        launch_arguments={
            'robot_x': robot_x,
            'robot_y': robot_y
        }.items()
    )
    
    # Include SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_slam'), 'launch', 'slam_launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Include Nav2 (with delay to wait for robot spawn)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('leo_nav2'), 'launch', 'nav2_launch_sim.py')])
            )])

    # Include the exploration launcher
    exploration_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('leo_exploration'), "launch", "exploration_sim.launch.py")]),
        launch_arguments={
            'router_x': router_x,
            'router_y': router_y,
            'stddev': stddev,
            'mode': mode
        }.items()
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
            ('scan', '/scan_filter'),
        ],
        condition=IfCondition(use_converter)
    )
    
    return LaunchDescription([
        use_converter_arg,
        robot_x_arg,
        robot_y_arg,
        router_x_arg,
        router_y_arg,
        stddev_arg,
        mode_arg,
        #Recorder,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        converter_node,
        #termination_node,
        exploration_sim_launch,
    ])