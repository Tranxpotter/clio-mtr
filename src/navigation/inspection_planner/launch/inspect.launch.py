import os
import math
import json
import launch
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_launch_rviz = DeclareLaunchArgument("rviz", default_value="true")
    launch_rviz = LaunchConfiguration("rviz")

    tsp_solver = Node(
        package="inspection_planner", 
        executable="tsp_solver", 
        name="tsp_solver"
    )

    inspection_planner = Node(
        package="inspection_planner", 
        executable="inspection_planner", 
        name="inspection_planner",
        parameters=[{
            "inspection_poses_topic": "/inspection_poses",
            "nav_action_server_name": "/nav_to_pose",
            "tsp_solver": "/solve_tsp",
            "tsp_waypoints_topic": "/inspection_waypoints",
            "tsp_distance_matrix_topic": "/tsp_distance_matrix",
            "pose_merge_distance_tolerance": 0.3,
            "pose_merge_angular_tolerance": 0.349066, 
            "use_tsp":True, 
        }]
    )

    rviz = Node(
        package="rviz2", 
        executable="rviz2", 
        name="inspection_rviz", 
        output="screen", 
        arguments=['-d', os.path.join(get_package_share_directory("inspection_planner"), "rviz", "default.rviz")], 
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        declare_launch_rviz, 
        tsp_solver, 
        inspection_planner, 
        rviz
    ])