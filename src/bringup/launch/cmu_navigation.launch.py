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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    velocity_smoother_dir = get_package_share_directory("velocity_smoother")

    # Declare launch arguments
    declare_use_bag = DeclareLaunchArgument('use_bag', default_value="False")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="True")
    declare_launch_static_odom = DeclareLaunchArgument('static_odom', default_value="True")
    declare_launch_localizer = DeclareLaunchArgument('localizer', default_value="True")
    declare_launch_remapper = DeclareLaunchArgument('remapper', default_value="True")
    declare_map_path = DeclareLaunchArgument('map_path', default_value="iw_maps/11-3-IWLG-full-processed2.pcd")
    declare_map_2d_path = DeclareLaunchArgument('map_2d_path', default_value="iw_2d_maps/11-3-IWLG-full.yaml")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/mapping")
    declare_plot = DeclareLaunchArgument('plot', default_value="True")

    use_bag = LaunchConfiguration('use_bag')
    launch_fastlio = LaunchConfiguration('fastlio')
    launch_static_odom = LaunchConfiguration('static_odom')
    launch_localizer = LaunchConfiguration('localizer')
    launch_remapper = LaunchConfiguration('remapper')
    map_path = LaunchConfiguration('map_path')
    map_2d_path = LaunchConfiguration('map_2d_path')
    bag_path = LaunchConfiguration('bag_path')
    record_bag = LaunchConfiguration('record_bag')
    plot = LaunchConfiguration('plot')

    # Livox ros driver 2
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                driver_dir, 
                "launch_ROS2", 
                "msg_MID360_launch.py"
            ])
        ), 
        condition=UnlessCondition(use_bag)
    )



    # Static odom for localizer to link map and robot_init
    static_odom_node = Node(
        package="localization_utils", 
        executable="static_odom_publisher", 
        name="static_odom_publisher", 
        output="screen", 
        parameters=[{
            "input_topic":"/cloud_registered", 
            "output_topic":"/static_odom", 
            "parent_frame":"/robot_init", 
            "child_frame":"/static_odom", 
            "period":0.01, 
            "verbose":False, 
            "use_sim_time":use_bag
        }], 
        condition=IfCondition(launch_static_odom)
    )



    fastlio_group = GroupAction(
        [
            # SetRemap("/Odometry", "/fastlio/Odometry"), 

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
                    "use_sim_time":use_bag, 
                    "rviz":"false"
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
                        "use_sim_time":use_bag
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
                    "use_sim_time":use_bag
                }], 
                condition=IfCondition(launch_localizer)
            )


    remapper = Node(
        package="localization_utils", 
        executable="pose_estimate_remapper", 
        name="pose_estimate_remapper", 
        parameters=[{
            "map_path":map_path, 
            "verbose":False, 
            "use_sim_time":use_bag
        }], 
        condition=IfCondition(launch_remapper)
    )

    # Static transform from robot_init to camera_init
    camera_init_static_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_init_static_pub",
        arguments=["0", "0", "0", "0.0", "0.0", "3.1415927", "robot_init", "camera_init"], # x, y, z, yaw, pitch, roll
        parameters=[{
            "use_sim_time":use_bag
        }]
    )

    
    sensor_frame_corrector_node = Node(
        package="localization_utils", 
        executable="sensor_frame_corrector", 
        name="sensor_frame_corrector", 
        parameters=[{
            "roll":0.0, 
            "pitch":3.14159265359, 
            "yaw":3.14159265359, 
            "point_cloud_input_topic":"/cloud_registered", 
            "point_cloud_output_topic":"/cloud_registered_corrected", 
            "odometry_input_topic":"/Odometry", 
            'odometry_output_topic': '/robot_odom', 
            'verbose':False, 
            "use_sim_time":use_bag
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
                    "use_sim_time":use_bag
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
        }], 
        condition=UnlessCondition(use_bag)
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
            "use_sim_time":use_bag
        }.items()
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path, "/livox/lidar", "/livox/imu", "/cmd_vel", "/initialpose", "/way_point"],
        output='screen', 
        condition=IfCondition(record_bag), 
        name="rosbag_recorder"
    )

    return LaunchDescription([
    declare_use_bag, 
    declare_launch_fastlio, 
    declare_launch_static_odom, 
    declare_launch_localizer, 
    declare_launch_remapper, 
    declare_map_path, 
    declare_map_2d_path, 
    declare_bag, 
    declare_bag_path, 
    declare_plot, 
    driver, 
    fastlio_group, 
    static_odom_node, 
    localizer_node, 
    localizer_rviz,
    remapper, 
    camera_init_static_pub, 
    sensor_frame_corrector_node, 
    cmu_group, 
    control_node, 
    realtime_TS_plotter_node, 
    realtime_T_plotter_node, 
    stability_visualizer_node, 
    velocity_smoother, 
    rosbag
    ])