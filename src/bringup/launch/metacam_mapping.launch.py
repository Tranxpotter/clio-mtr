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
    goal_rotator_wrapper_dir = get_package_share_directory("goal_rotator_wrapper")

    # Declare launch arguments
    declare_use_bag = DeclareLaunchArgument('use_bag', default_value="False")
    declare_bag = DeclareLaunchArgument('record_bag', default_value="True")
    declare_bag_path = DeclareLaunchArgument('bag_path', default_value="rosbags/")
    declare_plot = DeclareLaunchArgument('plot', default_value="False")

    use_bag = LaunchConfiguration('use_bag')
    bag_path = LaunchConfiguration('bag_path')
    record_bag = LaunchConfiguration('record_bag')
    plot = LaunchConfiguration('plot')


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
    
    metacam_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("metacam_utils"), 
            "launch", 
            "bridge.launch.py"
        )), 
        launch_arguments={
            "use_sim_time":use_bag
        }.items()
    )

    cloud_transform_to_map = Node(
        package="metacam_utils", 
        executable="pointcloud_process", 
        name="pc_transform_node", 
        parameters=[{
            "input_topic":"/cloud_registered_body", 
            "output_topic":"/cloud_registered", 
            "target_frame":"robot_init", 
            "do_transform":True, 
            "do_downsample":False, 
            "verbal":False, 
            "use_sim_time":use_bag
        }], 
    )

    body_footprint_static_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_footprint_static_pub",
        arguments=["0", "0", "0", "0.0", "-0.349066", "0.0", "body", "robot_footprint"], # x, y, z, yaw, pitch, roll
        parameters=[{
            "use_sim_time":use_bag
        }]
    )


    tf_to_odom = Node(
        package="metacam_utils", 
        executable="tf_to_odom", 
        name="tf_to_odom", 
        parameters=[{
            "parent_frame":"robot_init", 
            "child_frame":"robot_footprint", 
            "odom_topic":"/Odometry", 
            "rate":10.0, 
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
                    "odom_topic":"/Odometry", 
                    "cloud_topic":"/cloud_registered", 
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

    goal_rotator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                goal_rotator_wrapper_dir, 
                "launch", 
                "goal_rotator.launch.py"
            ])
        ), 
        launch_arguments={
            "use_sim_time":use_bag
        }.items()
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


    bag_name = datetime.datetime.now().isoformat(timespec="seconds").replace(":", "_")
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','-s','mcap', '-o', [bag_path, bag_name], 
             "/tower/lidar/points", 
             "/tower/imu/data", 
             "/tower/mapping/odometry", 
             "/tower/camera/left/jpeg", 
             "/tower/camera/left/preview", 
             "/tower/camera/right/jpeg", 
             "/tower/camera/right/preview", 
             "/tower/rtk/nmea",
             "/tower/rtk/gnss_soln",
            ],
        output='screen',  
        name="rosbag_recorder"
    )


    return LaunchDescription([
    declare_use_bag, 
    declare_bag, 
    declare_bag_path, 
    declare_plot, 
    robot_init_static_pub, 
    metacam_bridge, 
    cloud_transform_to_map, 
    body_footprint_static_pub, 
    tf_to_odom, 
    cmu_group, 
    far_group, 
    goal_rotator, 
    velocity_smoother, 
    # control_node, 
    realtime_TS_plotter_node, 
    realtime_T_plotter_node, 
    stability_visualizer_node, 
    rosbag
    ])