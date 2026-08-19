'''
Docstring for clio_bringup.launch.navigation.launch.py

Launch arguments: (* important)
    driver: Whether to launch the Livox ROS Driver 2 (default: True)
    fastlio: Whether to launch the Fast LIO mapping node (default: True)
    static_odom: Whether to launch static odom node to link map -> camera_init (default: True)
    localizer: Whether to launch the localizer node (default: True)
    remapper: Whether to launch the remapper node (default True)
    *map_path: Path to the saved map (default: "maps/scans.pcd")
    *map_2d_path: Path to 2d map yaml (default: "maps/map.yaml")
    use_sim_time: Use sim time (default: False)
'''



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
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    driver_dir = get_package_share_directory("livox_ros_driver2")
    fastlio_dir = get_package_share_directory("fast_lio")
    localizer_dir = get_package_share_directory("localizer")
    localization_utils_dir = get_package_share_directory("localization_utils")
    velocity_smoother_dir = get_package_share_directory("velocity_smoother")
    goal_rotator_wrapper_dir = get_package_share_directory("goal_rotator_wrapper")

    # Declare launch arguments
    declare_use_bag = DeclareLaunchArgument('use_bag', default_value="False")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="True")
    declare_launch_static_odom = DeclareLaunchArgument('static_odom', default_value="True")
    declare_launch_localizer = DeclareLaunchArgument('localizer', default_value="True")
    declare_launch_remapper = DeclareLaunchArgument('remapper', default_value="True")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/")
    declare_plot = DeclareLaunchArgument('plot', default_value="False")

    use_bag = LaunchConfiguration('use_bag')
    launch_fastlio = LaunchConfiguration('fastlio')
    launch_static_odom = LaunchConfiguration('static_odom')
    launch_localizer = LaunchConfiguration('localizer')
    launch_remapper = LaunchConfiguration('remapper')
    bag_path = LaunchConfiguration('bag_path')
    record_bag = LaunchConfiguration('record_bag')
    plot = LaunchConfiguration('plot')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                this_pkg_dir, 
                "launch", 
                "cmu_navigation.launch.py"
            ])
        ), 
        launch_arguments={
            "use_bag":use_bag, 
            "fastlio":launch_fastlio, 
            "static_odom":launch_static_odom, 
            "localizer":launch_localizer, 
            "remapper":launch_remapper, 
            "bag_path":bag_path, 
            "record_bag":record_bag, 
            "plot":plot, 
            "far_rviz":"False"
        }.items()
    )

    waypoint_publisher = Node(
        package="inspection_planner_test", 
        executable="viewpose_publisher_service", 
        name="viewpose_publisher_service"
    )

    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory("inspection_planner"), 
                "launch", 
                "inspect.launch.py"
            ])
        )
    )

    inspection_poses_loader = Node(
        package="inspection_planner_test", 
        executable="inspection_poses_loader", 
        name="inspection_poses_loader", 
        parameters=[{
            "target_frame": "map",
            "source_frame": "map"
        }]
    )

    # Metacam and robot static tf
    robot_to_metacam_tf = Node(
        package="tf2_ros", 
        executable="static_transform_publisher", 
        name="robot_to_metacam", 
        arguments=[
            '0.03', '0', '0.18',     # x and z axis diff
            '0.0', '0.349066', '0',   # 20 degrees pitch  6.4 degrees yaw for forcefully matching waypoints
            'map', 'metacam'
        ]
    )



    return LaunchDescription([
    declare_use_bag, 
    declare_launch_fastlio, 
    declare_launch_static_odom, 
    declare_launch_localizer, 
    declare_launch_remapper, 
    declare_bag, 
    declare_bag_path, 
    declare_plot, 
    
    nav, 
    waypoint_publisher, 
    inspection_launch, 
    inspection_poses_loader, 
    robot_to_metacam_tf
    ])