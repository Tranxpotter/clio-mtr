#!/usr/bin/env python3
"""
inspection_poses_loader - loads inspection poses from database

Provides a service that accepts a database path, loads the poses from database

Usage:
    ros2 run inspection_planner_test inspection_poses_loader

Call the service:
    ros2 service call /load_inspection_poses inspection_planner_interfaces/srv/LoadInspectionPoses "{db_path: '/path/to/file.db'}"
"""

import os
import sqlite3
import rclpy
from rclpy.node import Node
from inspection_planner_interfaces.msg import ViewPoses, ViewPose
from inspection_planner_interfaces.srv import LoadInspectionPoses
from geometry_msgs.msg import Pose, Point, Quaternion
import tf2_ros
import rclpy.time
import tf2_geometry_msgs


class InspectionPosesLoader(Node):
    """Service node that reads database and publishes ViewPoses to /inspection_poses."""

    def __init__(self):
        super().__init__('inspection_poses_loader')

        self.declare_parameter("target_frame", "map")       # Optional frame transformation
        self.declare_parameter("source_frame", "metacam")
        
        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value

        self.tf_buffer = None
        self.tf_listener = None
        if self.target_frame != self.source_frame:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(ViewPoses, '/inspection_poses', 5)

        self.srv = self.create_service(
            LoadInspectionPoses,
            '/load_inspection_poses',
            self._handle_publish_request
        )
        self.get_logger().info('Service /load_inspection_poses is ready.')

    def _handle_publish_request(
        self,
        request: LoadInspectionPoses.Request,
        response: LoadInspectionPoses.Response
    ) -> LoadInspectionPoses.Response:
        db_path = request.db_path

        if not db_path:
            response.success = False
            response.message = 'No db_path provided.'
            response.waypoints_published = 0
            return response

        if not os.path.exists(db_path):
            response.success = False
            response.message = f'File not found: {db_path}'
            response.waypoints_published = 0
            return response

        try:
            conn = sqlite3.connect(db_path)
            cur = conn.cursor()
        except Exception as e:
            response.success = False
            response.message = f'Failed to connect to database: {e}'
            response.waypoints_published = 0
            return response

        # Check if abnormal_detections and images tables exist
        cur.execute(
            "SELECT name FROM sqlite_master WHERE type='table' "
            "AND name IN ('abnormal_detections', 'images')"
        )
        existing_tables = {row[0] for row in cur.fetchall()}
        if not {'abnormal_detections', 'images'}.issubset(existing_tables):
            conn.close()
            response.success = False
            response.message = 'Database missing required "abnormal_detections" or "images" tables.'
            response.waypoints_published = 0
            return response

        # Get inspection poses by joining abnormal_detections and images
        cur.execute(
            "SELECT DISTINCT img.id, "
            "img.tf_translation_x, img.tf_translation_y, img.tf_translation_z, "
            "img.tf_rotation_x, img.tf_rotation_y, img.tf_rotation_z, img.tf_rotation_w "
            "FROM abnormal_detections ad "
            "JOIN images img ON ad.gt_image = img.id " 
            "ORDER BY img.timestamp_ns"
        )
        rows = cur.fetchall()
        conn.close()

        inspection_pose_list = [
            {
                'id': row[0],
                'position': {'x': row[1], 'y': row[2], 'z': row[3]},
                'orientation': {'x': row[4], 'y': row[5], 'z': row[6], 'w': row[7]},
            }
            for row in rows
        ]

        # Build ViewPoses message
        msg = ViewPoses()
        for entry in inspection_pose_list:
            vp = ViewPose()
            vp.id = int(entry.get('id', 0))

            pos_data = entry.get('position', {})
            ori_data = entry.get('orientation', {})

            pose = Pose(
                position=Point(
                    x=float(pos_data.get('x', 0.0)),
                    y=float(pos_data.get('y', 0.0)),
                    z=float(pos_data.get('z', 0.0)),
                ),
                orientation=Quaternion(
                    x=float(ori_data.get('x', 0.0)),
                    y=float(ori_data.get('y', 0.0)),
                    z=float(ori_data.get('z', 0.0)),
                    w=float(ori_data.get('w', 1.0)),
                ),
            )

            # Do pose transform
            pose = self.transform_pose(pose)
            vp.pose = pose

            msg.poses.append(vp)

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published {len(msg.poses)} view poses from {db_path}'
        )

        response.success = True
        response.message = f'Successfully published {len(msg.poses)} view poses.'
        response.waypoints_published = len(msg.poses)
        return response
    
    def transform_pose(self, pose:Pose) -> Pose:
        if self.source_frame == self.target_frame:
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().warn(f"Cannot transform pose from {self.source_frame} to {self.target_frame}.")
            return pose


def main(args=None):
    rclpy.init(args=args)
    node = InspectionPosesLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()