#!/usr/bin/env python3
"""
latest_pose_to_odom - ROS2 node that converts inspection debug poses to odometry.

Subscribes to /inspection_debug/latest_pose (ViewPose) and publishes
/robot_odom (nav_msgs/msg/Odometry) using the previously received pose.

Usage:
    ros2 run inspection_planner_test latest_pose_to_odom
"""

import rclpy
from rclpy.node import Node
from inspection_planner_interfaces.msg import ViewPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time


class LatestPoseToOdom(Node):
    def __init__(self):
        super().__init__('latest_pose_to_odom')

        # Store the most recently received pose
        self.previous_pose = None

        # Subscriber
        self.subscription = self.create_subscription(
            ViewPose,
            '/inspection_debug/latest_pose',
            self.pose_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/robot_odom',
            10
        )

        self.get_logger().info(
            'latest_pose_to_odom node started. '
            'Subscribing to /inspection_debug/latest_pose, '
            'publishing to /robot_odom'
        )

    def pose_callback(self, msg: ViewPose):
        if self.previous_pose is not None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'robot_init'
            odom.child_frame_id = 'robot_footprint'
            odom.pose.pose = self.previous_pose
            self.odom_pub.publish(odom)

        self.previous_pose = msg.pose


def main(args=None):
    rclpy.init(args=args)
    node = LatestPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
