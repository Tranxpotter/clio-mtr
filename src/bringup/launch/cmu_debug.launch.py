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
    velocity_smoother_dir = get_package_share_directory("velocity_smoother")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="True")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="False")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/")
    declare_plot = DeclareLaunchArgument('plot', default_value="False")

    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')
    record_bag = LaunchConfiguration('record_bag')
    plot = LaunchConfiguration('plot')


    odom_to_tf = Node(
        package="odom_to_tf_ros2", 
        executable="odom_to_tf", 
        name="odom_to_tf", 
        parameters=[{
            "odom_topic":"/robot_odom", 
            "use_original_timestamp":False, 
            "use_sim_time":use_sim_time
        }]
    )

    
    # CMU Local Planner
    cmu_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'), 'launch', 'system_real_robot.launch')
        ), 
        launch_arguments={
            "odom_frame":"robot_init", 
            "robot_frame":"robot_footprint", 
            "use_sim_time":use_sim_time
        }.items()
    )

    # FAR Planner
    far_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('far_planner'), 'launch', 'far_planner.launch')
        ), 
        launch_arguments={
            "use_sim_time":use_sim_time
        }.items()
    )



    realtime_TS_plotter_node = Node(
        package="debug", 
        executable="realtime_twist_stamped_plotter_node", 
        name="realtime_twist_stamped_plotter_node", 
        condition=IfCondition(plot), 
        parameters=[{
            "topic_name":"/cmd_vel"
        }]
    )

    realtime_T_plotter_node = Node(
        package="debug", 
        executable="realtime_twist_stamped_plotter_node", 
        name="realtime_twist_stamped_plotter_node2", 
        condition=IfCondition(plot), 
        parameters=[{
            "topic_name":"/cmd_vel_smoothed"
        }]
    )

    stability_visualizer_node = Node(
        package="debug", 
        executable="path_stability_visualizer_node", 
        name="path_stability_visualizer_node", 
        condition=IfCondition(plot)
    )

    velocity_smoother = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                velocity_smoother_dir, 
                "launch", 
                "smoother.launch.py"
            ]), 
        ), 
        launch_arguments={
            "use_sim_time":use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time, 
        declare_bag, 
        declare_bag_path, 
        declare_plot, 
        odom_to_tf, 
        cmu_group, 
        far_group, 
        realtime_TS_plotter_node, 
        realtime_T_plotter_node, 
        stability_visualizer_node, 
        velocity_smoother
    ])