import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    this_pkg_name = 'realsense'
    this_pkg_dir = get_package_share_directory(this_pkg_name)

    declare_launch_camera = DeclareLaunchArgument("camera", default_value="false")
    launch_camera = LaunchConfiguration("camera")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                this_pkg_dir, 
                "launch", 
                "camera.launch.py"
            ])
        ), 
        condition=IfCondition(launch_camera)
    )

    record = Node(
        package=this_pkg_name, 
        executable="record", 
        name="record", 
    )

    

    return LaunchDescription([
        declare_launch_camera, 
        camera, 
        record
    ])