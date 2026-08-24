import math
import os
import rclpy
import rclpy.time
from rclpy.node import Node
from inspection_planner_interfaces.msg import ViewPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def quaternion_to_yaw(q) -> float:
    """Convert a geometry_msgs/Quaternion or Transform rotation to yaw in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavAccuracyLogger(Node):
    def __init__(self):
        super().__init__('nav_accuracy_logger')

        # Declare customizable parameters
        self.declare_parameter('pose_topic', '/goal_pose')
        self.declare_parameter('log_file', 'pose_diff.log')

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.log_file_path = self.get_parameter('log_file').get_parameter_value().string_value

        # Pose Subscriber
        self.sub = self.create_subscription(
            ViewPose,
            pose_topic,
            self.pose_callback,
            10
        )

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State storage
        self.prev_pose = None
        self.prev_id = None

        self.get_logger().info(f"Subscribed to '{pose_topic}'. Output logging to terminal and '{os.path.abspath(self.log_file_path)}'")

    def pose_callback(self, msg: ViewPose):
        # 1. First pose incoming: store and wait for the next
        if self.prev_pose is None:
            self.prev_pose = msg.pose
            self.prev_id = msg.id
            self.get_logger().info(f"Received first pose. ID: {msg.id} Stored as prev_pose.")
            return

        # 2. Subsequent pose incoming: look up current TF transform map -> robot_footprint
        try:
            tf_transform = self.tf_buffer.lookup_transform(
                'map',
                'robot_footprint',
                rclpy.time.Time()
            )
        except Exception as ex:
            self.get_logger().warn(f"Could not transform 'map' to 'robot_footprint': {ex}")
            return

        # Extract positions
        robot_pos = tf_transform.transform.translation
        prev_pos = self.prev_pose.position

        # Position Difference (component-wise & Euclidean distance)
        dx = robot_pos.x - prev_pos.x
        dy = robot_pos.y - prev_pos.y
        dz = robot_pos.z - prev_pos.z
        euc_dist = math.sqrt(dx**2 + dy**2)

        # Extract Yaws (in radians and degrees)
        robot_yaw = quaternion_to_yaw(tf_transform.transform.rotation)
        prev_yaw = quaternion_to_yaw(self.prev_pose.orientation)

        # Yaw difference normalized to [-pi, pi]
        raw_yaw_diff = robot_yaw - prev_yaw
        yaw_diff = math.atan2(math.sin(raw_yaw_diff), math.cos(raw_yaw_diff))

        # Format the log output
        log_entry = (
            f"--- Pose Comparison ---\n"
            f"Viewpose ID: {self.prev_id}\n"
            f"Position Diff (dx: {dx:.3f}m, dy: {dy:.3f}m, dz: {dz:.3f}m) | Euclidean: {euc_dist:.3f}m\n"
            f"Robot Current Yaw: {math.degrees(robot_yaw):.2f}° | Prev Pose Yaw: {math.degrees(prev_yaw):.2f}°\n"
            f"Yaw Difference: {math.degrees(yaw_diff):.2f}° ({yaw_diff:.4f} rad)\n"
        )

        # Log to ROS 2 Terminal
        self.get_logger().info(log_entry)

        # Log to local text file
        timestamp = self.get_clock().now().to_msg()
        with open(self.log_file_path, 'a') as f:
            f.write(f"[{timestamp.sec}.{timestamp.nanosec}] {log_entry}\n")

        # 3. Replace prev_pose with the incoming pose
        self.prev_pose = msg.pose
        self.prev_id = msg.id


def main(args=None):
    rclpy.init(args=args)
    node = NavAccuracyLogger()
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