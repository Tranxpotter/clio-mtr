'''
PointcloudRotator Node

This node rotates pointcloud data to compensate for a tilted LiDAR sensor.
Supports both Livox CustomMsg and standard PointCloud2 formats.

Parameters
----------
rotation_angle_deg : float
    Rotation angle in degrees. Positive = counter-clockwise when looking
    along the positive axis direction. Default: 0.0
rotation_axis : str
    Axis to rotate around: '0'='x', '1'='y', '2'='z'. Default: '1' (y/pitch)
    Note: Using numbers because 'y' is a YAML boolean.
input_topic : str
    Input pointcloud topic. Default: '/livox/lidar'
output_topic : str
    Output pointcloud topic. Default: '/livox/lidar_rotated'
frame_id : str
    Frame ID for the output pointcloud. If empty, uses input frame_id. Default: ''
use_custom_msg : bool
    Whether to use Livox CustomMsg format. Auto-detected if not specified. Default: True

Example Usage
-------------
For a LiDAR tilted 60 degrees to the ground (30 degrees from horizontal),
pointing downward, use rotation_angle_deg=-30.0 and rotation_axis='1' (y-axis).
'''

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# Try to import Livox CustomMsg
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    LIVOX_AVAILABLE = True
except ImportError:
    LIVOX_AVAILABLE = False
    CustomMsg = None
    CustomPoint = None


class PointcloudRotator(Node):
    def __init__(self):
        super().__init__('pointcloud_rotator')

        # Declare parameters
        self.declare_parameter('rotation_angle_deg', 0.0)
        self.declare_parameter('rotation_axis', 1)  # Integer: 0=x, 1=y, 2=z (YAML issue with 'y')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_rotated')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('use_custom_msg', True)

        # Get parameters
        self.rotation_angle_deg = self.get_parameter(
            'rotation_angle_deg').get_parameter_value().double_value
        
        # Get rotation_axis as integer and map to letter
        axis_value = self.get_parameter('rotation_axis').get_parameter_value().integer_value
        axis_map = {0: 'x', 1: 'y', 2: 'z'}
        self.rotation_axis = axis_map.get(axis_value, 'y')
        
        self.input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.use_custom_msg = self.get_parameter(
            'use_custom_msg').get_parameter_value().bool_value

        # Validate rotation axis
        if self.rotation_axis not in ['x', 'y', 'z']:
            self.get_logger().error(
                f"Invalid rotation_axis '{self.rotation_axis}'. "
                "Must be 'x', 'y', 'z', '0', '1', or '2'. Defaulting to 'y'.")
            self.rotation_axis = 'y'

        # Check if Livox messages are available
        if self.use_custom_msg and not LIVOX_AVAILABLE:
            self.get_logger().warn(
                'Livox CustomMsg not available. Falling back to PointCloud2.')
            self.use_custom_msg = False

        # Convert angle to radians
        self.rotation_angle_rad = math.radians(self.rotation_angle_deg)

        # Log configuration
        msg_type = 'CustomMsg' if self.use_custom_msg else 'PointCloud2'
        self.get_logger().info(
            f"PointcloudRotator initialized:\n"
            f"  Rotation angle: {self.rotation_angle_deg} degrees "
            f"({self.rotation_angle_rad:.4f} rad)\n"
            f"  Rotation axis: {self.rotation_axis}\n"
            f"  Input topic: {self.input_topic}\n"
            f"  Output topic: {self.output_topic}\n"
            f"  Input message type: {msg_type}\n"
            f"  Livox available: {LIVOX_AVAILABLE}"
        )

        # DEBUG: Print actual topic names
        self.get_logger().info(f"[DEBUG] Creating subscriber on topic: '{self.input_topic}'")
        self.get_logger().info(f"[DEBUG] Creating publisher on topic: '{self.output_topic}'")
        
        # Create subscriber based on message type
        if self.use_custom_msg:
            if LIVOX_AVAILABLE:
                self.custom_sub = self.create_subscription(
                    CustomMsg,
                    self.input_topic,
                    self.custom_msg_callback,
                    10
                )
                self.get_logger().info(f"Subscribed to '{self.input_topic}' (CustomMsg)")
            else:
                self.get_logger().error(
                    'Cannot use CustomMsg - livox_ros_driver2 not available')
                self.use_custom_msg = False

        # Always create PointCloud2 subscriber as fallback
        if not self.use_custom_msg:
            self.pc2_sub = self.create_subscription(
                PointCloud2,
                self.input_topic,
                self.pointcloud2_callback,
                10
            )
            self.get_logger().info(f"Subscribed to '{self.input_topic}' (PointCloud2)")

        # Create publisher (always PointCloud2 for compatibility)
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )
        self.get_logger().info(f"Publishing to '{self.output_topic}' (PointCloud2)")

        # Track message count for debugging
        self.msg_count = 0

    def create_rotation_matrix(self, angle: float, axis: str) -> np.ndarray:
        """Create a 3x3 rotation matrix for rotation around specified axis."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        if axis == 'x':
            return np.array([
                [1, 0, 0],
                [0, cos_a, -sin_a],
                [0, sin_a, cos_a]
            ])
        elif axis == 'y':
            return np.array([
                [cos_a, 0, sin_a],
                [0, 1, 0],
                [-sin_a, 0, cos_a]
            ])
        else:  # axis == 'z'
            return np.array([
                [cos_a, -sin_a, 0],
                [sin_a, cos_a, 0],
                [0, 0, 1]
            ])

    def rotate_points(self, points: np.ndarray) -> np.ndarray:
        """Apply rotation to a set of 3D points."""
        rotation_matrix = self.create_rotation_matrix(
            self.rotation_angle_rad, self.rotation_axis)
        return np.dot(points, rotation_matrix.T)

    def custom_msg_callback(self, msg: CustomMsg):
        """Process Livox CustomMsg, apply rotation, and publish as PointCloud2."""
        if not msg.points:
            self.get_logger().warn('Received empty CustomMsg', throttle_duration_sec=5)
            return

        self.msg_count += 1
        if self.msg_count <= 3 or self.msg_count % 100 == 0:
            self.get_logger().info(
                f'Processing CustomMsg #{self.msg_count} with {len(msg.points)} points')

        # Build list of points for create_cloud
        # Format: [x, y, z, intensity, offset_time, ring]
        points_list = []
        for point in msg.points:
            points_list.append([
                point.x,
                point.y,
                point.z,
                float(point.reflectivity),  # intensity
                float(point.offset_time),    # offset_time
                float(point.line)            # ring
            ])

        # Convert to numpy for rotation
        points_array = np.array(points_list, dtype=np.float32)
        xyz = points_array[:, :3]

        # Apply rotation
        rotated_xyz = self.rotate_points(xyz)
        points_array[:, :3] = rotated_xyz

        # Convert back to list of tuples for create_cloud
        points_list = [tuple(p) for p in points_array]

        # Create header
        header = Header()
        header.stamp = msg.header.stamp  # Use original timestamp
        header.frame_id = self.frame_id if self.frame_id else msg.header.frame_id

        # Define fields matching Livox CustomMsg format
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='offset_time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        # Create PointCloud2 message using the library function
        output_msg = pc2.create_cloud(header, fields, points_list)

        # Publish
        self.publisher.publish(output_msg)

        if self.msg_count <= 3:
            self.get_logger().info(
                f'Published PointCloud2: {output_msg.width} points, '
                f'frame_id: {output_msg.header.frame_id}')

    def pointcloud2_callback(self, msg: PointCloud2):
        """Process PointCloud2, apply rotation, and publish result."""
        self.msg_count += 1
        
        # Read all fields from the message
        field_names = [field.name for field in msg.fields]
        
        try:
            # Read points with all available fields
            points_gen = pc2.read_points(msg, skip_nans=True)
            points_list = list(points_gen)
        except Exception as e:
            self.get_logger().error(f'Failed to read PointCloud2: {e}')
            return

        if not points_list:
            self.get_logger().warn('Received empty pointcloud', throttle_duration_sec=5)
            return

        if self.msg_count <= 3 or self.msg_count % 100 == 0:
            self.get_logger().info(
                f'Processing PointCloud2 #{self.msg_count} with {len(points_list)} points, '
                f'fields: {field_names}')

        # Convert to numpy for rotation
        num_points = len(points_list)
        
        # Extract XYZ coordinates (always first 3 fields)
        xyz = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
        
        # Apply rotation
        rotated_xyz = self.rotate_points(xyz)
        
        # Build new points list with rotated coordinates
        new_points_list = []
        for i, point in enumerate(points_list):
            new_point = list(point)
            new_point[0] = rotated_xyz[i, 0]  # x
            new_point[1] = rotated_xyz[i, 1]  # y
            new_point[2] = rotated_xyz[i, 2]  # z
            new_points_list.append(tuple(new_point))

        # Create header
        header = Header()
        header.stamp = msg.header.stamp  # Preserve original timestamp
        header.frame_id = self.frame_id if self.frame_id else msg.header.frame_id

        # Create PointCloud2 message using the same fields.
        # Some inputs have fields that are out-of-order by offset which makes
        # numpy structured dtypes fail ("overlapping or out-of-order fields").
        # Work around by sorting the fields by offset and reordering the
        # point tuples to match that order before building the cloud.
        fields_sorted = sorted(msg.fields, key=lambda f: f.offset)

        # If ordering differs, compute an index map from sorted order back to
        # the original tuple order returned by read_points (which follows
        # msg.fields order).
        orig_field_names = [f.name for f in msg.fields]
        sorted_field_names = [f.name for f in fields_sorted]
        if orig_field_names != sorted_field_names:
            index_map = [orig_field_names.index(n) for n in sorted_field_names]
            reordered_points = [tuple(p[i] for i in index_map) for p in new_points_list]
        else:
            reordered_points = new_points_list

        output_msg = pc2.create_cloud(header, fields_sorted, reordered_points)

        # Publish
        self.publisher.publish(output_msg)

        if self.msg_count <= 3:
            self.get_logger().info(
                f'Published PointCloud2: {output_msg.width}x{output_msg.height} points, '
                f'frame_id: {output_msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudRotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
