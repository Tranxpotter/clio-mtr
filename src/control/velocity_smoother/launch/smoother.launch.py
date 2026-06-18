import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Fetch package share directory map path
    pkg_share = get_package_share_directory('velocity_smoother')

    # Configurable arguments mapping paths
    config_file_param = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'smoother_params.yaml'),
        description='Full path to the YAML parameter file to use'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock telemetry if true'
    )

    # Node Action Execution
    smoother_node = launch_ros.actions.Node(
        package='velocity_smoother',
        executable='ema_velocity_smoother_node',
        name='ema_velocity_smoother_node',
        output='screen',
        parameters=[config_file_param, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_config_file_cmd,
        declare_use_sim_time_cmd,
        smoother_node
    ])