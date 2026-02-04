from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
            parameters=[{'odom_topic': odom_topic}]  # Pass to node
        ),
        Node(
            package='leo_exploration',
            executable='rss_node',
            name='rss_node',
            output='screen',
        ),
    ])