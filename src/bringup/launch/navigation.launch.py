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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    driver_dir = get_package_share_directory("livox_ros_driver2")
    fastlio_dir = get_package_share_directory("fast_lio")
    localizer_dir = get_package_share_directory("localizer")
    localization_utils_dir = get_package_share_directory("localization_utils")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Declare launch arguments
    declare_launch_driver = DeclareLaunchArgument('driver', default_value="True")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="True")
    declare_launch_static_odom = DeclareLaunchArgument('static_odom', default_value="True")
    declare_launch_localizer = DeclareLaunchArgument('localizer', default_value="True")
    declare_launch_remapper = DeclareLaunchArgument('remapper', default_value="True")
    declare_map_path = DeclareLaunchArgument('map_path', default_value="iw_maps/11-3-IWLG-full-processed2.pcd")
    declare_map_2d_path = DeclareLaunchArgument('map_2d_path', default_value="iw_2d_maps/11-3-IWLG-full.yaml")
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="False")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/mapping")

    launch_driver = LaunchConfiguration('driver')
    launch_fastlio = LaunchConfiguration('fastlio')
    launch_static_odom = LaunchConfiguration('static_odom')
    launch_localizer = LaunchConfiguration('localizer')
    launch_remapper = LaunchConfiguration('remapper')
    map_path = LaunchConfiguration('map_path')
    map_2d_path = LaunchConfiguration('map_2d_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_path = LaunchConfiguration('bag_path')
    record_bag = LaunchConfiguration('record_bag')

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



    # Static odom to link map and camera_init
    static_odom_node = Node(
        package="localization_utils", 
        executable="static_odom_publisher", 
        name="static_odom_publisher", 
        output="screen", 
        parameters=[{
            "input_topic":"/cloud_registered", 
            "output_topic":"/static_odom", 
            "parent_frame":"/camera_init", 
            "child_frame":"/static_odom", 
            "period":0.01, 
            "verbose":False, 
            "use_sim_time":use_sim_time
        }], 
        condition=IfCondition(launch_static_odom)
    )



    fastlio_group = GroupAction(
        [
            SetRemap("/Odometry", "/fastlio2/lio_odom"), 

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        fastlio_dir, 
                        "launch", 
                        "mapping.launch.py"
                    ])
                ), 
                launch_arguments={
                    "config_file":"mid360.yaml", 
                    "use_sim_time":use_sim_time
                }.items(), 
                condition=IfCondition(launch_fastlio)
            )
        ]
    )

    

    # ===================== Localizer Launch ============================

    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )

    localizer_node = Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": localizer_config_path.perform(
                            launch.LaunchContext()
                        ), 
                        "use_sim_time":use_sim_time
                    }
                ],
                condition=IfCondition(launch_localizer)
            )
    localizer_rviz = Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                parameters=[{
                    "use_sim_time":use_sim_time
                }], 
                condition=IfCondition(launch_localizer)
            )


    remapper = Node(
        package="localization_utils", 
        executable="pose_estimate_remapper", 
        name="pose_estimate_remapper", 
        parameters=[{
            "map_path":map_path, 
            "verbose":True, 
            "use_sim_time":use_sim_time
        }], 
        condition=IfCondition(launch_remapper)
    )


    # Currently works pretty well, though can still think about whether there is a more elegant solution
    height_remover = Node(
        package="localization_utils", 
        executable="tf_height_remover", 
        name="tf_height_remover", 
        parameters=[{
            "world_frame":"map", 
            "input_frame":"robot_footprint", 
            "output_frame":"base_footprint", 
            "verbose":False, 
            "z_extra_offset":0.2, 
            "use_sim_time":use_sim_time
        }]
    )

    robot_init_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot_init_pub",
        arguments=["0", "0", "0", "3.14159", "-3.14159", "0", "camera_init", "robot_init"], # x, y, z, yaw, pitch, roll
        parameters=[{
            "use_sim_time":use_sim_time
        }]
    )

    footprint_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="footprint_pub",
        arguments=["0", "0", "0", "3.14159", "-3.14159", "0", "body", "robot_footprint"],
        parameters=[{
            "use_sim_time":use_sim_time
        }]
    )
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                this_pkg_dir, 
                "launch", 
                "nav_bringup.launch.py"
            ])
        ), 
        launch_arguments={
            "params_file":PathJoinSubstitution([this_pkg_dir, "config", "nav2_params.yaml"]), 
            "map":map_2d_path,
            "use_sim_time":use_sim_time
        }.items()
    )

    # Tron control node
    control_node = Node(
        package="tron1_control",
        executable="bridge", 
        name="tron_bridge", 
        parameters=[{
            "robot_ip":'10.192.1.2', 
            'robot_port': 5000, 
            'accid': 'WF_TRON1A_212', 
        }]
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path, "/livox/lidar", "/livox/imu"],
        output='screen', 
        condition=IfCondition(record_bag), 
        name="rosbag_recorder"
    )

    return LaunchDescription([
    declare_launch_driver, 
    declare_launch_fastlio, 
    declare_launch_static_odom, 
    declare_launch_localizer, 
    declare_launch_remapper, 
    declare_map_path, 
    declare_map_2d_path, 
    declare_use_sim_time, 
    declare_bag, 
    declare_bag_path, 
    driver, 
    fastlio_group, 
    static_odom_node, 
    localizer_node, 
    localizer_rviz,
    remapper, 
    height_remover, 
    robot_init_pub,
    footprint_pub,
    nav2_bringup, 
    control_node, 
    rosbag
    ])