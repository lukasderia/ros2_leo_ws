import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to Xbox config file
    config_filepath = os.path.join(
        get_package_share_directory('leo_teleop'),
        'config',
        'xbox.config.yaml'
    )

    return LaunchDescription([
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