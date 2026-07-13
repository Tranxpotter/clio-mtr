#!/usr/bin/env python3
"""
goal_point_recorder - Record goal points from /goal_point and save to YAML.

Subscribes to /goal_point (geometry_msgs/PointStamped), assigns each a
sequential inspection waypoint ID, and writes all collected points to a
YAML file when the user presses Ctrl+C (SIGINT).

Usage:
    ros2 run inspection_planner_test goal_point_recorder --output-file ~/inspection_points.yaml

Output YAML format:
    waypoints:
      - id: 1
        x: 1.234
        y: 2.345
        z: 3.456
      - id: 2
        ...
"""

import signal
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class GoalPointRecorder(Node):
    """Subscribe to /goal_point and accumulate unique points with IDs."""

    def __init__(self, output_file: str):
        super().__init__('goal_point_recorder')
        self.output_file = output_file
        self.waypoints = []
        self.seen_points = set()  # for deduplication (rounded)

        self.subscription = self.create_subscription(
            PointStamped,
            '/goal_point',
            self._goal_point_callback,
            10
        )
        self.get_logger().info(
            f'Listening on /goal_point. Press Ctrl+C to save to {output_file}'
        )

    def _goal_point_callback(self, msg: PointStamped) -> None:
        p = msg.point
        # Round to 6 decimals for deduplication
        key = (round(p.x, 6), round(p.y, 6), round(p.z, 6))
        if key in self.seen_points:
            return
        self.seen_points.add(key)
        wp_id = len(self.waypoints) + 1
        self.waypoints.append({
            'id': wp_id,
            'x': p.x,
            'y': p.y,
            'z': p.z,
        })
        self.get_logger().info(
            f'Recorded waypoint #{wp_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
        )


def save_yaml(waypoints: list, path: str) -> None:
    """Write waypoints to a YAML file (no external dependency required)."""
    # Try PyYAML first; fall back to manual formatting.
    try:
        import yaml
        data = {'waypoints': waypoints}
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        return
    except ImportError:
        pass

    # Manual YAML writer (produces valid YAML without PyYAML)
    with open(path, 'w') as f:
        f.write('waypoints:\n')
        for wp in waypoints:
            f.write(f'  - id: {wp["id"]}\n')
            f.write(f'    x: {wp["x"]}\n')
            f.write(f'    y: {wp["y"]}\n')
            f.write(f'    z: {wp["z"]}\n')


def parse_args() -> str:
    """Parse --output-file argument from command line."""
    output = 'goal_points.yaml'
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '--output-file' and i + 1 < len(args):
            output = args[i + 1]
            i += 2
        else:
            i += 1
    return output


def main(args=None):
    output_file = parse_args()
    rclpy.init(args=args)
    node = GoalPointRecorder(output_file)

    # Register SIGINT handler to gracefully save and exit
    def handle_sigint(signum, frame):
        node.get_logger().info(f'\nReceived interrupt. Saving {len(node.waypoints)} waypoints...')
        save_yaml(node.waypoints, output_file)
        node.get_logger().info(f'Saved to {output_file}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    # signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        handle_sigint(None, None)


if __name__ == '__main__':
    main()
