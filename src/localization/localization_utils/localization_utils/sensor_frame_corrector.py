#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import array

# --- MONKEY PATCH FOR NUMPY 1.24+ / CONDA COMPATIBILITY ---
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'maximum_sctype'):
    # Just in case maximum_sctype is also deprecated/removed in your NumPy env
    np.maximum_sctype = lambda x: np.finfo(x).machar.type
# -----------------------------------------------------------

# Message imports
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

# TF imports (this will now load without throwing an error)
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_transformations

class SensorFrameCorrector(Node):
    def __init__(self):
        super().__init__('sensor_frame_corrector')

        # ----------------------------------------------------------------------
        # 1. ROS 2 Parameters (Easy optimization via Launch parameters)
        # ----------------------------------------------------------------------
        self.declare_parameter('verbose', False)
        # By default, a 180 degree flip around X-axis (Roll = pi)
        self.declare_parameter('roll', 3.14159265359)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # Translation offsets if the sensor isn't at the rotational center
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.0)

        # Topics
        self.declare_parameter('point_cloud_input_topic', '/cloud_registered')
        self.declare_parameter('point_cloud_output_topic', '/cloud_registered_corrected')
        self.declare_parameter('odometry_input_topic', '/Odometry')
        self.declare_parameter('odometry_output_topic', '/robot_odom')

        # Frames
        # TODO: Need custom frames here for more flexibility


        # Get values
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        r = self.get_parameter('roll').get_parameter_value().double_value
        p = self.get_parameter('pitch').get_parameter_value().double_value
        y = self.get_parameter('yaw').get_parameter_value().double_value
        tx = self.get_parameter('x_offset').get_parameter_value().double_value
        ty = self.get_parameter('y_offset').get_parameter_value().double_value
        tz = self.get_parameter('z_offset').get_parameter_value().double_value
        pc_input_topic = self.get_parameter('point_cloud_input_topic').get_parameter_value().string_value
        pc_output_topic = self.get_parameter('point_cloud_output_topic').get_parameter_value().string_value
        odom_input_topic = self.get_parameter('odometry_input_topic').get_parameter_value().string_value
        odom_output_topic = self.get_parameter('odometry_output_topic').get_parameter_value().string_value

        # Calculate rotation transformations (4x4 Homogeneous transformation matrices)
        # T_rot converts inverted frames to right-side up structures
        q_rot = tf_transformations.quaternion_from_euler(r, p, y)
        self.T_rot = tf_transformations.quaternion_matrix(q_rot)
        self.T_rot[0:3, 3] = [tx, ty, tz]
        self.T_rot_inv = tf_transformations.inverse_matrix(self.T_rot)

        # ----------------------------------------------------------------------
        # 2. Pubs, Subs, and TF Broadcasters
        # ----------------------------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Point Cloud handling
        self.pc_sub = self.create_subscription(
            PointCloud2, pc_input_topic, self.pointcloud_callback, 10)
        self.pc_pub = self.create_publisher(
            PointCloud2, pc_output_topic, 10)

        # Odometry handling
        self.odom_sub = self.create_subscription(
            Odometry, odom_input_topic, self.odom_callback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, odom_output_topic, 10)

        self.get_logger().info(f"Sensor Frame Corrector node successfully started. {self.verbose=}")

    

    def odom_callback(self, msg: Odometry):
        """
        Processes FAST_LIO odometry (camera_init -> body).
        1. Computes right-side-up Odometry (robot_init -> robot_footprint) for CMU.
        2. Broadcasts a dynamic linear TF frame (body -> robot_footprint).
        """
        # --- 1. COMPUTE THE RIGHT-SIDE-UP ODOMETRY MATRIX ---
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        T_cam_to_body = tf_transformations.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
        T_cam_to_body[0:3, 3] = [pos.x, pos.y, pos.z]

        # Calculate where the robot footprint is relative to robot_init
        T_robot = np.dot(np.dot(self.T_rot, T_cam_to_body), self.T_rot_inv)
        t_output = tf_transformations.translation_from_matrix(T_robot)
        q_output = tf_transformations.quaternion_from_matrix(T_robot)

        # --- 2. CALCULATE THE LINEAR TF LINK (body -> robot_footprint) ---
        # To keep your tree in a straight line, the TF broadcaster frame handles body -> robot_footprint
        # This actively counter-rotates FAST_LIO's pitch/roll inversions in real-time
        T_body_to_footprint = self.T_rot_inv 
        
        t_tf = tf_transformations.translation_from_matrix(T_body_to_footprint)
        q_tf = tf_transformations.quaternion_from_matrix(T_body_to_footprint)

        # --- 3. BROADCAST THE LINEAR TF ENTRY ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp # Match FAST_LIO's update timestamp perfectly
        tf_msg.header.frame_id = 'body'             # Parent frame
        tf_msg.child_frame_id = 'robot_footprint'    # Child frame
        tf_msg.transform.translation.x = t_tf[0]
        tf_msg.transform.translation.y = t_tf[1]
        tf_msg.transform.translation.z = t_tf[2]
        tf_msg.transform.rotation.x = q_tf[0]
        tf_msg.transform.rotation.y = q_tf[1]
        tf_msg.transform.rotation.z = q_tf[2]
        tf_msg.transform.rotation.w = q_tf[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # --- 4. PUBLISH NAV2 ODOMETRY FOR CMU EXPLORATION ---
        corrected_odom = Odometry()
        corrected_odom.header.stamp = msg.header.stamp
        corrected_odom.header.frame_id = 'robot_init'
        corrected_odom.child_frame_id = 'robot_footprint'
        
        corrected_odom.pose.pose.position.x = t_output[0]
        corrected_odom.pose.pose.position.y = t_output[1]
        corrected_odom.pose.pose.position.z = t_output[2]
        corrected_odom.pose.pose.orientation.x = q_output[0]
        corrected_odom.pose.pose.orientation.y = q_output[1]
        corrected_odom.pose.pose.orientation.z = q_output[2]
        corrected_odom.pose.pose.orientation.w = q_output[3]

        # Velocity conversions
        vel_lin = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, 0.0])
        vel_ang = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, 0.0])
        
        vel_lin_corr = np.dot(self.T_rot, vel_lin)
        vel_ang_corr = np.dot(self.T_rot, vel_ang)

        corrected_odom.twist.twist.linear.x = vel_lin_corr[0]
        corrected_odom.twist.twist.linear.y = vel_lin_corr[1]
        corrected_odom.twist.twist.linear.z = vel_lin_corr[2]
        corrected_odom.twist.twist.angular.x = vel_ang_corr[0]
        corrected_odom.twist.twist.angular.y = vel_ang_corr[1]
        corrected_odom.twist.twist.angular.z = vel_ang_corr[2]

        self.odom_pub.publish(corrected_odom)

        if self.verbose:
            self.get_logger().info("Published corrected odom")



    def pointcloud_callback(self, msg: PointCloud2):
        """
        Processes /cloud_registered pointcloud matrices, physically rotating 
        every 3D spatial coordinate from camera_init to right-side up robot_init.
        """
        if len(msg.data) == 0:
            return
        
        # 1. Generate the exact layout rule factoring in the true point stride
        dtype = point_cloud2.dtype_from_fields(msg.fields, point_step=msg.point_step)
        
        # 2. Extract raw bytes into our writable numpy template
        point_array = np.frombuffer(msg.data, dtype=dtype).copy()
        num_points = point_array.shape[0]

        # 3. Pull coordinates into the homogeneous transformation layout
        points = np.ones((num_points, 4), dtype=np.float32)
        points[:, 0] = point_array['x']
        points[:, 1] = point_array['y']
        points[:, 2] = point_array['z']

        # 4. Apply the 4x4 matrix transformation
        transformed_points = np.dot(self.T_rot, points.T).T

        # 5. Shovel the transformed positions straight back into the structured fields
        point_array['x'] = transformed_points[:, 0]
        point_array['y'] = transformed_points[:, 1]
        point_array['z'] = transformed_points[:, 2]

        # 6. MANUALLY BUILD THE POINTCLOUD2 CONTAINER
        corrected_cloud = PointCloud2()
        corrected_cloud.header.stamp = msg.header.stamp
        corrected_cloud.header.frame_id = 'robot_init'
        
        # Replicate structural metadata verbatim
        corrected_cloud.height = msg.height
        corrected_cloud.width = msg.width
        corrected_cloud.fields = msg.fields
        corrected_cloud.is_bigendian = msg.is_bigendian
        corrected_cloud.point_step = msg.point_step
        corrected_cloud.row_step = msg.row_step
        corrected_cloud.is_dense = msg.is_dense

        # 7. BYPASS MEMORYVIEW: Flatten the structured array directly to an array of bytes
        # This is safer, cleaner, and keeps all custom field steps intact.
        raw_bytes = point_array.tobytes()
        corrected_cloud.data = array.array('B', raw_bytes)

        # 8. Ship it out
        self.pc_pub.publish(corrected_cloud)

        if self.verbose:
            self.get_logger().info("Physically rotated point cloud buffer successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = SensorFrameCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()