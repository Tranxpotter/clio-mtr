'''
Launch file to visualize LiDAR pointcloud and test pointcloud rotation.

Launch arguments:
    rotation_angle_deg: Rotation angle in degrees to compensate for tilt (default: -30.0)
        Negative = rotate up (compensate for downward tilt)
        Positive = rotate down (compensate for upward tilt)
    rotation_axis: Axis to rotate around - '0' for x, '1' for y, '2' for z (default: '1')
        '0' = Roll (x-axis)
        '1' = Pitch (y-axis) - most common for forward tilt
        '2' = Yaw (z-axis)
    enable_rotation: Whether to enable the pointcloud rotator (default: false)
    rviz_config: Path to RViz config file (optional)

Note: We use numbers (0, 1, 2) instead of letters (x, y, z) for rotation_axis
because 'y' is interpreted as a YAML boolean (yes) in ROS2 parameter passing.
To visualize, set the fixed frame in RViz to "livox_frame" and add a PointCloud2 display subscribing to "/livox/lidar_rotated".
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Map axis numbers to letters (for logging only now)
AXIS_MAP = {
    0: 'x',
    1: 'y', 
    2: 'z',
}


def launch_setup(context: LaunchContext, *args, **kwargs):
    """Opaque function to properly resolve launch configurations."""
    
    enable_rotation_str = LaunchConfiguration('enable_rotation').perform(context)
    rotation_angle_str = LaunchConfiguration('rotation_angle_deg').perform(context)
    rotation_axis_raw = LaunchConfiguration('rotation_axis').perform(context)
    rviz_config_str = LaunchConfiguration('rviz_config').perform(context)
    
    # Parse enable_rotation
    enable_rotation_bool = enable_rotation_str.lower() in ('true', '1', 'yes', 'on')
    
    # Parse rotation angle
    try:
        rotation_angle_float = float(rotation_angle_str)
    except ValueError:
        print(f"[WARN] Invalid rotation_angle_deg: '{rotation_angle_str}', using default -30.0")
        rotation_angle_float = -30.0
    
    # Parse rotation axis as integer
    try:
        axis_key = int(rotation_axis_raw)
    except ValueError:
        print(f"[WARN] Invalid rotation_axis: '{rotation_axis_raw}', using default 1 (y/pitch)")
        axis_key = 1
    
    rotation_axis_str = AXIS_MAP.get(axis_key, 'y')  # For logging
    
    if axis_key not in AXIS_MAP:
        print(f"[WARN] Invalid rotation_axis: {axis_key}, using default 1 (y/pitch)")
        axis_key = 1
    
    nodes = []
    
    # RViz2 node
    rviz_args = ['-d', rviz_config_str] if rviz_config_str else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': False}]
    )
    nodes.append(rviz_node)
    
    # Pointcloud rotator node (only if enabled)
    if enable_rotation_bool:
        pointcloud_rotator = Node(
            package='localization_utils',
            executable='pointcloud_rotator',
            name='pointcloud_rotator',
            output='screen',
            parameters=[{
                'rotation_angle_deg': rotation_angle_float,
                'rotation_axis': axis_key,  # Integer: 0=x, 1=y, 2=z
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/lidar_rotated',
                'use_custom_msg': True,
            }]
        )
        nodes.append(pointcloud_rotator)
        
        axis_name = rotation_axis_str  # x, y, or z
        info_rotated = LogInfo(
            msg=f'Visualizing ROTATED pointcloud from /livox/lidar_rotated '
                f'(rotation_angle_deg: {rotation_angle_float}, axis: {axis_key}={axis_name})'
        )
        nodes.insert(0, info_rotated)
    else:
        info_raw = LogInfo(
            msg='Visualizing RAW pointcloud from /livox/lidar (rotation disabled)'
        )
        nodes.insert(0, info_raw)
    
    return nodes


def generate_launch_description():
    # Declare launch arguments
    declare_enable_rotation = DeclareLaunchArgument(
        'enable_rotation',
        default_value='false',
        description='Enable pointcloud rotation to compensate for tilted LiDAR'
    )
    declare_rotation_angle = DeclareLaunchArgument(
        'rotation_angle_deg',
        default_value='-60.0',
        description='Rotation angle in degrees (negative = rotate up for downward tilt)'
    )
    # Use numbers to avoid YAML boolean parsing issues with 'y'
    declare_rotation_axis = DeclareLaunchArgument(
        'rotation_axis',
        default_value='1',
        description="Rotation axis: '0'=x(roll), '1'=y(pitch), '2'=z(yaw). "
                    "Using numbers because 'y' is a YAML boolean."
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Path to RViz config file (optional)'
    )

    return LaunchDescription([
        declare_enable_rotation,
        declare_rotation_angle,
        declare_rotation_axis,
        declare_rviz_config,
        OpaqueFunction(function=launch_setup),
    ])
