import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare argument for config file
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='xbox.config.yaml',
        description='Name of the teleop config file to use'
    )
    
    # Get the config file path using substitutions
    config_filepath = PathJoinSubstitution([
        FindPackageShare('leo_teleop'),
        'config',
        LaunchConfiguration('config_file')
    ])

    return LaunchDescription([
        config_arg,
        
        # Joy node for controller
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 10.0,
            }]
        ),
        
        # Controller teleop
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath]
        ),
        
        # Mode toggle node
        Node(
            package='leo_teleop',
            executable='leo_mode',
            name='mode_node'
        ),
    ])