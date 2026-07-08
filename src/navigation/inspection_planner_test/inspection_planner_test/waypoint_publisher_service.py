#!/usr/bin/env python3
"""
waypoint_publisher_service – Publish inspection waypoints from a YAML file via service.

Provides a service that accepts a YAML file path, reads the recorded waypoints,
converts them to inspection_planner_interfaces/Waypoints messages, and publishes
them to /inspection_waypoints for FAR Planner TSP distance calculations.

Usage:
    ros2 run inspection_planner waypoint_publisher_service

Call the service:
    ros2 service call /publish_waypoints_from_yaml inspection_planner_interfaces/srv/PublishWaypointsFromYaml "{yaml_file_path: '/path/to/file.yaml'}"
"""

import os
import rclpy
from rclpy.node import Node
from inspection_planner_interfaces.msg import Waypoints, Waypoint
from inspection_planner_interfaces.srv import PublishWaypointsFromYaml
from geometry_msgs.msg import Point
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
    #       x: 1.0
    #       y: 2.0
    #       z: 3.0
    data = {'waypoints': []}
    current = None
    with open(path, 'r') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if stripped.startswith('- id:'):
                if current:
                    data['waypoints'].append(current)
                current = {'id': int(stripped.split(':')[1].strip())}
            elif current is not None and ':' in stripped:
                key, val = stripped.split(':', 1)
                key = key.strip()
                val = val.strip()
                try:
                    val = float(val)
                except ValueError:
                    pass
                current[key] = val
        if current:
            data['waypoints'].append(current)
    return data


class WaypointPublisherService(Node):
    """Service node that reads YAML and publishes Waypoints to /inspection_waypoints."""

    def __init__(self):
        super().__init__('waypoint_publisher_service')
        # qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5)
        self.publisher = self.create_publisher(Waypoints, '/inspection_waypoints', 5)

        self.srv = self.create_service(
            PublishWaypointsFromYaml,
            '/publish_waypoints_from_yaml',
            self._handle_publish_request
        )
        self.get_logger().info(
            'Service /publish_waypoints_from_yaml is ready.'
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

        # Build Waypoints message
        msg = Waypoints()
        for entry in wp_list:
            wp = Waypoint()
            wp.id = int(entry.get('id', 0))
            wp.point = Point(
                x=float(entry.get('x', 0.0)),
                y=float(entry.get('y', 0.0)),
                z=float(entry.get('z', 0.0)),
            )
            msg.waypoints.append(wp)

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published {len(msg.waypoints)} waypoints from {yaml_path}'
        )

        response.success = True
        response.message = f'Successfully published {len(msg.waypoints)} waypoints.'
        response.waypoints_published = len(msg.waypoints)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisherService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
