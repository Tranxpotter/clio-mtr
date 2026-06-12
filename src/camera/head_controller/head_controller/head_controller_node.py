#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from head_controller.msg import HeadPose
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from stservo_ros.msg import ServoCommand
import asyncio
import time
import numpy as np
from scipy.spatial.transform import Rotation

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class HeadControllerNode(Node):
    def __init__(self):
        super().__init__("head_controller_node")

        self.subscription = self.create_subscription(HeadPose, "/head_pose", self.head_pose_callback, 10)

        # Head servo configuration
        self.head_servo_ids = [1, 2]  # Servo IDs for head: pan, roll
        self.max_speed = 2000  # Servo speed parameter
        self.max_acceleration = 200  # Servo acceleration parameter

        # Servo angle limits for safety
        self.servo_limits = {
            0: (-90.0, 90.0),   # Servo 0: Bottom
            1: (-180.0, 180.0),   # Servo 1: Yaw  
            2: (-180.0, 180.0),   # Servo 2: Pitch. TBC
        }

        # Create servo command publisher for direct head servo control
        self.servo_command_pub = self.create_publisher(ServoCommand, 'servo_command', 10)

    def head_pose_callback(self, msg: HeadPose):
        angle1:float = msg.angle1
        angle2:float = msg.angle2
        cmd = ServoCommand()
        cmd.command_type = "position"
        cmd.servo_ids = [1,2]
        cmd.angles = [angle1, angle2]
        cmd.speeds = [self.max_speed, self.max_speed]
        cmd.accelerations = [self.max_acceleration, self.max_acceleration]

        # Apply servo limits for safety
        clamped_angles = []
        for i, (servo_id, angle) in enumerate(zip(self.head_servo_ids, cmd.angles)):
            clamped_angle = self.clamp_servo_angle(servo_id, angle)
            clamped_angles.append(clamped_angle)
        cmd.angles = clamped_angles
        
        # Send the servo command
        self.servo_command_pub.publish(cmd)
        self.get_logger().info(f"[Head] Published look forward command: servo_ids={cmd.servo_ids}, angles={cmd.angles}")
        

    def clamp_servo_angle(self, servo_id: int, angle_deg: float) -> float:
        """Clamp servo angle to safe limits"""
        if servo_id in self.servo_limits:
            min_angle, max_angle = self.servo_limits[servo_id]
            clamped_angle = max(min_angle, min(max_angle, angle_deg))
            if abs(clamped_angle - angle_deg) > 0.1:
                self.get_logger().warn(f"[Head] Servo {servo_id} angle clamped from {angle_deg:.2f}° to {clamped_angle:.2f}°")
            return clamped_angle
        return angle_deg

def main(args=None):
    rclpy.init(args=args)
    node = HeadControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
