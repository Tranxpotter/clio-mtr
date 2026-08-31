"""
Launch file for the poses_visualizer node.

Usage:
    # Subscribe to live topic:
    ros2 launch inspection_planner_test pose_visualizer.launch.py

    # Load from a saved file:
    ros2 launch inspection_planner_test pose_visualizer.launch.py load_file:=/path/to/poses_debug_20260101_120000.json
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'inspection_planner_test'

    # Declare launch arguments
    load_file_arg = DeclareLaunchArgument(
        'load_file',
        default_value='',
        description='Path to a saved ViewPosesDebug JSON file. '
                    'If empty, the node will subscribe to /inspection_log/poses_debug.'
    )

    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='',
        description='Directory to save received messages. '
                    'If empty, defaults to saved_poses/ inside the package.'
    )

    # Define the node
    poses_visualizer_node = Node(
        package=package_name,
        executable='poses_visualizer',
        name='poses_visualizer',
        output='screen',
        parameters=[{
            'load_file': LaunchConfiguration('load_file'),
            'save_dir': LaunchConfiguration('save_dir'),
        }],
    )

    return LaunchDescription([
        load_file_arg,
        save_dir_arg,
        poses_visualizer_node,
    ])
