from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import TextSubstitution

def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic name')
    router_x_arg = DeclareLaunchArgument('router_x', default_value='18.0', description='Router x position')
    router_y_arg = DeclareLaunchArgument('router_y', default_value='18.0', description='Router y position')
    stddev_arg = DeclareLaunchArgument('stddev', default_value='1.0', description='Standard deviation on wifi signal')
    mode_arg = DeclareLaunchArgument('mode', default_value='2', 
    description='Exploration mode: 0=yamauchi, 1=gao, 2=rss')

    odom_topic = LaunchConfiguration('odom_topic')
    router_x = ParameterValue(LaunchConfiguration('router_x'), value_type=float)
    router_y = ParameterValue(LaunchConfiguration('router_y'), value_type=float)
    stddev = ParameterValue(LaunchConfiguration('stddev'), value_type=float)
    mode = ParameterValue(LaunchConfiguration('mode'), value_type=int)
    
    return LaunchDescription([
        odom_topic_arg,
        router_x_arg,
        router_y_arg,
        stddev_arg,
        mode_arg,
        Node(
            package='leo_utils',
            executable='map_filter.py',
            name='map_filter',
            output='screen'
        ),
        Node(
            package='leo_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen'
        ),
        Node(
            package='leo_exploration',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'mode': mode}]
        ),
        Node(
            package='leo_exploration',
            executable='rss_node_sim',
            name='rss_node_sim',
            output='screen',
            parameters=[{
                'router_x': router_x,
                'router_y': router_y,
                'stddev': stddev
            }]
        )
    ])