from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import TextSubstitution

def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='2', description='Exploration mode: 0=yamauchi, 1=gao, 2=rss')
    mode = ParameterValue(LaunchConfiguration('mode'), value_type=int)

    # Declare the argument with default value
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic name'
    )
    
    # Get the configuration value
    odom_topic = LaunchConfiguration('odom_topic')
    
    return LaunchDescription([
        odom_topic_arg,
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
                'mode': mode
            }]
        ),
        Node(
            package='leo_exploration',
            executable='rss_node',
            name='rss_node',
            output='screen',
        ),
    ])