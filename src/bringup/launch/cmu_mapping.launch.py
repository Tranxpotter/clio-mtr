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
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    driver_dir = get_package_share_directory("livox_ros_driver2")
    fastlio_dir = get_package_share_directory("fast_lio")
    velocity_smoother_dir = get_package_share_directory("velocity_smoother")

    # Declare launch arguments
    declare_use_bag = DeclareLaunchArgument('use_bag', default_value="False")
    declare_launch_fastlio = DeclareLaunchArgument('fastlio', default_value="True")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/")
    declare_plot = DeclareLaunchArgument('plot', default_value="False")

    use_bag = LaunchConfiguration('use_bag')
    launch_fastlio = LaunchConfiguration('fastlio')
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




    fastlio_group = TimerAction(
        period=2.0, # Delay 2 seconds for driver to start
        actions=[GroupAction(
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
            ])]
    )

    # Use static transform from map to robot_init instead of localizer
    robot_init_static_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot_init_static_pub",
        arguments=["0", "0", "0", "0.0", "0.0", "0.0", "map", "robot_init"], # x, y, z, yaw, pitch, roll
        parameters=[{
            "use_sim_time":use_bag
        }]
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
    cmu_group = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('vehicle_simulator'), 'launch', 'system_real_robot.launch')
                ), 
                launch_arguments={
                    "odom_frame":"robot_init", 
                    "robot_frame":"robot_footprint", 
                    "use_sim_time":use_bag
                }.items()
            )
        ]
    )

    # FAR Planner
    far_group = TimerAction(
        period=4.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('far_planner'), 'launch', 'far_planner.launch')
                ), 
                launch_arguments={
                    "use_sim_time":use_bag
                }.items()
            )
        ]
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

    bag_name = datetime.datetime.now().isoformat(timespec="seconds").replace(":", "_")
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', [bag_path, bag_name], "/livox/lidar", "/livox/imu", "/initialpose", "/way_point"],
        output='screen', 
        condition=IfCondition(record_bag), 
        name="rosbag_recorder"
    )


    return LaunchDescription([
    declare_use_bag, 
    declare_launch_fastlio, 
    declare_bag, 
    declare_bag_path, 
    declare_plot, 
    driver, 
    fastlio_group, 
    robot_init_static_pub, 
    camera_init_static_pub, 
    sensor_frame_corrector_node, 
    cmu_group, 
    far_group, 
    # control_node, 
    realtime_TS_plotter_node, 
    realtime_T_plotter_node, 
    stability_visualizer_node, 
    velocity_smoother, 
    rosbag
    ])