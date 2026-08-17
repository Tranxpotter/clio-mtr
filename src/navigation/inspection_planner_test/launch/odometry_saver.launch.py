import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'inspection_planner_test'

    launch_dir = os.path.dirname(os.path.realpath(__file__))
    src_db_dir = os.path.abspath(os.path.join(launch_dir, '..', 'databases'))
    os.makedirs(src_db_dir, exist_ok=True)

    # 1. Declare Launch Arguments
    db_filename_arg = DeclareLaunchArgument(
        'db_filename',
        default_value='inspections.db',
        description='Filename for the SQLite database stored in the package databases directory'
    )

    is_gt_arg = DeclareLaunchArgument(
        'is_gt',
        default_value='false',
        description='Boolean flag indicating whether the dataset is ground truth (true/false)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/robot_odom',
        description='Input Odometry topic name'
    )

    # 2. Append package share path: <package_share_path>/databases/<db_filename>
    db_path = PathJoinSubstitution([
        src_db_dir, 
        LaunchConfiguration('db_filename')
    ])

    # 3. Define Node
    odom_sqlite_node = Node(
        package=package_name,
        executable='odometry_saver',
        name='odometry_saver',
        output='screen',
        parameters=[{
            'db_path': db_path,
            'is_gt': LaunchConfiguration('is_gt'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }]
    )

    return LaunchDescription([
        db_filename_arg,
        is_gt_arg,
        odom_topic_arg,
        odom_sqlite_node
    ])