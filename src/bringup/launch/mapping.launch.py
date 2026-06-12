'''
Docstring for clio_bringup.launch.mapping.launch

Launch arguments:
    driver: Whether to launch the Livox ROS Driver 2 (default: True)
    fastlio: Whether to launch the Fast LIO mapping node (default: True)
    save_path: Path to save the generated map (default: "maps/scans.pcd") # Currently disabled
    record_bag: Whether to record a rosbag during mapping (default: True)
    bag_path: Path to save the recorded rosbag (default: "rosbags/mapping")
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

# def shutdown_func_with_echo_side_effect(event, context):
#     os.system('echo [os.system()] Shutdown callback function can echo this way.')
#     return [
#         LogInfo(msg='Shutdown callback was called for reason "{}"'.format(event.reason)),
#         LogInfo(msg="Copied scan to desired path."), 
#         ExecuteProcess(cmd=['echo', 'However, this echo will fail.'])]


def generate_launch_description():
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    driver_dir = get_package_share_directory("livox_ros_driver2")
    fastlio_dir = get_package_share_directory("fast_lio")

    # Declare launch arguments
    declare_launch_driver = DeclareLaunchArgument('driver', default_value="True")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="True")
    declare_save_path = DeclareLaunchArgument('save_path', default_value="maps/scans.pcd")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/mapping")

    launch_driver = LaunchConfiguration('driver')
    launch_fastlio = LaunchConfiguration('fastlio')
    save_path = LaunchConfiguration('save_path')
    record_bag = LaunchConfiguration('record_bag')
    bag_path = LaunchConfiguration('bag_path')
    fastlio_map_path = "src/FAST_LIO_ROS2/PCD/scans.pcd"

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

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path, "/livox/lidar", "/livox/imu"],
        output='screen', 
        condition=IfCondition(record_bag), 
        name="rosbag_recorder"
    )

    copy_map_action = ExecuteProcess(
        cmd=["cp", fastlio_map_path, save_path], 
        output="screen", 
        name="copy_map"
    )

    # on_shutdown_event = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=fastlio, 
    #         on_exit=copy_map_action
    #     )
    # )

    # on_shutdown_event = RegisterEventHandler(
    #     OnShutdown()
    # )

    return LaunchDescription([
        declare_launch_driver, 
        declare_launch_fastlio, 
        declare_save_path, 
        declare_bag, 
        declare_bag_path, 
        driver, 
        fastlio, 
        rosbag, 
        # on_shutdown_event
    ])