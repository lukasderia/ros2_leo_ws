from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Declare the argument with default value false (for real robot)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path to your SLAM params file
    slam_params_path = os.path.join(
        get_package_share_directory('leo_slam'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            {'use_sim_time': use_sim_time}  # This overrides the yaml file
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_node
    ])