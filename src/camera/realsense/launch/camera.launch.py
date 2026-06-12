import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    # 1. Evaluate 'use_pointcloud' to a raw Python string ('True' or 'False')
    use_pointcloud_value = LaunchConfiguration('use_pointcloud').perform(context)

    # 2. Build the launch description dynamically inside the context
    return [
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py',
                    ])
                ]),
                launch_arguments={
                    'camera_name': 'head_camera',
                    'camera_namespace':'camera', 
                    'device_type': 'd435',
                    'enable_color': 'true',
                    'enable_depth': 'true',
                    'align_depth.enable': 'true',
                    'enable_sync':'true', 
                    # 3. Pass the evaluated string value, NOT the LaunchConfiguration object
                    'pointcloud.enable': use_pointcloud_value,
                    'rgb_camera.enable_auto_exposure': 'false'
                }.items()
            ),
        ], 
        forwarding=False)
    ]

def generate_launch_description():
    this_pkg_name = 'realsense'
    this_pkg_dir = get_package_share_directory(this_pkg_name)


    use_pointcloud_arg = DeclareLaunchArgument(
        'use_pointcloud',
        default_value='False',
        description='Enable pointcloud generation'
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="False", description="Launch rviz to visualize camera output")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(this_pkg_dir, "rviz", "camera.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        use_pointcloud_arg, 
        use_rviz_arg, 
        # OpaqueFunction executes launch_setup and passes the current context into it
        OpaqueFunction(function=launch_setup), 
        rviz_node

    ])