#!/usr/bin/env python3
"""
viewpose_publisher_service - Publish inspection view poses from a YAML file via service.

Provides a service that accepts a YAML file path, reads the recorded view poses,
converts them to inspection_planner_interfaces/ViewPoses messages, and publishes
them to /inspection_poses for visualization and planning.

Usage:
    ros2 run inspection_planner_test viewpose_publisher_service

Call the service:
    ros2 service call /publish_viewposes_from_yaml inspection_planner_interfaces/srv/PublishWaypointsFromYaml "{yaml_file_path: '/path/to/file.yaml'}"

Expected YAML format (as produced by goal_pose_recorder):
    waypoints:
      - id: 1
        position:
          x: 1.234
          y: 2.345
          z: 3.456
        orientation:
          x: 0.0
          y: 0.0
          z: 0.707
          w: 0.707
      - id: 2
        ...
"""

import os
import rclpy
from rclpy.node import Node
from inspection_planner_interfaces.msg import ViewPoses, ViewPose
from inspection_planner_interfaces.srv import PublishWaypointsFromYaml
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def load_yaml(path: str) -> dict:
    """Load a YAML file, falling back to a simple parser if PyYAML is unavailable."""
    try:
        import yaml
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except ImportError:
        pass

    # Minimal YAML parser for the expected format:
    #   waypoints:
    #     - id: 1
    #       position:
    #         x: 1.0
    #         y: 2.0
    #         z: 3.0
    #       orientation:
    #         x: 0.0
    #         y: 0.0
    #         z: 0.707
    #         w: 0.707
    data = {'waypoints': []}
    current = None
    current_sub = None
    with open(path, 'r') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if stripped.startswith('- id:'):
                if current:
                    data['waypoints'].append(current)
                current = {'id': int(stripped.split(':')[1].strip())}
                current_sub = None
            elif current is not None and ':' in stripped:
                key, val = stripped.split(':', 1)
                key = key.strip()
                val = val.strip()

                if key in ('position', 'orientation'):
                    current_sub = {}
                    current[key] = current_sub
                elif current_sub is not None:
                    try:
                        val = float(val)
                    except ValueError:
                        pass
                    current_sub[key] = val
        if current:
            data['waypoints'].append(current)
    return data


class ViewPosePublisherService(Node):
    """Service node that reads YAML and publishes ViewPoses to /inspection_poses."""

    def __init__(self):
        super().__init__('viewpose_publisher_service')
        self.publisher = self.create_publisher(ViewPoses, '/inspection_poses', 5)

        self.srv = self.create_service(
            PublishWaypointsFromYaml,
            '/publish_viewposes_from_yaml',
            self._handle_publish_request
        )
        self.get_logger().info(
            'Service /publish_viewposes_from_yaml is ready.'
        )

    def _handle_publish_request(
        self,
        request: PublishWaypointsFromYaml.Request,
        response: PublishWaypointsFromYaml.Response
    ) -> PublishWaypointsFromYaml.Response:
        yaml_path = request.yaml_file_path

        if not yaml_path:
            response.success = False
            response.message = 'No yaml_file_path provided.'
            response.waypoints_published = 0
            return response

        if not os.path.isfile(yaml_path):
            response.success = False
            response.message = f'File not found: {yaml_path}'
            response.waypoints_published = 0
            return response

        try:
            data = load_yaml(yaml_path)
        except Exception as e:
            response.success = False
            response.message = f'Failed to parse YAML: {e}'
            response.waypoints_published = 0
            return response

        wp_list = data.get('waypoints', [])
        if not wp_list:
            response.success = False
            response.message = 'No waypoints found in YAML file.'
            response.waypoints_published = 0
            return response

        # Build ViewPoses message
        msg = ViewPoses()
        for entry in wp_list:
            vp = ViewPose()
            vp.id = int(entry.get('id', 0))

            pos_data = entry.get('position', {})
            ori_data = entry.get('orientation', {})

            vp.pose = Pose(
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
            msg.poses.append(vp)

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published {len(msg.poses)} view poses from {yaml_path}'
        )

        response.success = True
        response.message = f'Successfully published {len(msg.poses)} view poses.'
        response.waypoints_published = len(msg.poses)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ViewPosePublisherService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()