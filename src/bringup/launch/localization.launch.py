'''
Docstring for clio_bringup.launch.localization.launch

Launch arguments: (* important)
    driver: Whether to launch the Livox ROS Driver 2 (default: False)
    fastlio: Whether to launch the Fast LIO mapping node (default: False)
    localizer: Whether to launch the localizer node (default: True)
    remapper: Whether to launch the remapper node (default True)
    *map_path: Path to the saved map (default: "maps/scans.pcd")
    use_bag: Whether to play a ros bag (default: False)           #Usually we should play rosbag in another terminal
    bag_path: Path the saved ros bag (default: "rosbags/mapping")
'''



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
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    driver_dir = get_package_share_directory("livox_ros_driver2")
    fastlio_dir = get_package_share_directory("fast_lio")
    localizer_dir = get_package_share_directory("localizer")
    guide_robot_localization_dir = get_package_share_directory("guide_robot_localization")

    # Declare launch arguments
    declare_launch_driver = DeclareLaunchArgument('driver', default_value="False")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="False")
    declare_launch_localizer = DeclareLaunchArgument('localizer', default_value="True")
    declare_launch_remapper = DeclareLaunchArgument('remapper', default_value="True")
    declare_map_path = DeclareLaunchArgument('map_path', default_value="maps/scans.pcd")
    declare_use_bag = DeclareLaunchArgument('use_bag', default_value="False")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/mapping")

    launch_driver = LaunchConfiguration('driver')
    launch_fastlio = LaunchConfiguration('fastlio')
    launch_localizer = LaunchConfiguration('localizer')
    launch_remapper = LaunchConfiguration('remapper')
    map_path = LaunchConfiguration('map_path')
    use_bag = LaunchConfiguration('use_bag')
    bag_path = LaunchConfiguration('bag_path')

    # Livox ros driver 2
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                driver_dir, 
                "launch_ROS2", 
                "msg_MID360_launch.py"
            ])
        ), 
        condition=IfCondition(launch_driver)
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                fastlio_dir, 
                "launch", 
                "mapping.launch.py"
            ])
        ), 
        launch_arguments={
            "config_file":"mid360.yaml"
        }.items(), 
        condition=IfCondition(launch_fastlio)
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                localizer_dir, 
                "launch", 
                "localizer_launch.py"
            ])
        ), 
        condition=IfCondition(launch_localizer)
    )

    remapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                guide_robot_localization_dir, 
                "launch", 
                "goal_pose_remapper.launch.py"
            ])
        ), 
        launch_arguments={
            "map":map_path
        }.items(), 
        condition=IfCondition(launch_remapper)
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen', 
        condition=IfCondition(use_bag), 
        name="rosbag_recorder"
    )

    

    return LaunchDescription([
        declare_launch_driver, 
        declare_launch_fastlio, 
        declare_launch_localizer, 
        declare_launch_remapper, 
        declare_map_path, 
        declare_use_bag, 
        declare_bag_path, 
        driver, 
        fastlio, 
        localizer, 
        remapper, 
        rosbag, 
    ])