import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the package share directory
    package_name = 'goal_rotator_wrapper'
    package_share_dir = get_package_share_directory(package_name)

    # 2. Path to your config YAML file
    config_file_path = os.path.join(package_share_dir, 'config', 'config.yaml')

    # 3. Define the node action
    goal_rotator_node = Node(
        package=package_name,
        executable='goal_rotator_node',
        name='goal_rotator_node',
        output='screen',
        # This loads the parameters from the YAML file directly into the node
        parameters=[config_file_path]
    )

    return LaunchDescription([
        goal_rotator_node
    ])