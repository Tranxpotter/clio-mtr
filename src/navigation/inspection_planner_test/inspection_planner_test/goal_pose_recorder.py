#!/usr/bin/env python3
"""
goal_pose_recorder – Record goal poses from /goal_pose and save to YAML.

Subscribes to /goal_pose (geometry_msgs/PoseStamped), assigns each a
sequential inspection waypoint ID, and writes all collected poses to a
YAML file when the user presses Ctrl+C (SIGINT).

Usage:
    ros2 run inspection_planner goal_pose_recorder --output-file ~/inspection_poses.yaml

Output YAML format:
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

import signal
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPoseRecorder(Node):
    """Subscribe to /goal_pose and accumulate unique poses with IDs."""

    def __init__(self, output_file: str):
        super().__init__('goal_pose_recorder')
        self.output_file = output_file
        self.waypoints = []
        self.seen_poses = set()  # for deduplication (rounded)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_pose_callback,
            10
        )
        self.get_logger().info(
            f'Listening on /goal_pose. Press Ctrl+C to save to {output_file}'
        )

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        pose = msg.pose
        # Round to 6 decimals for deduplication
        key = (
            round(pose.position.x, 6),
            round(pose.position.y, 6),
            round(pose.position.z, 6),
            round(pose.orientation.x, 6),
            round(pose.orientation.y, 6),
            round(pose.orientation.z, 6),
            round(pose.orientation.w, 6),
        )
        if key in self.seen_poses:
            return
        self.seen_poses.add(key)
        wp_id = len(self.waypoints) + 1
        self.waypoints.append({
            'id': wp_id,
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w,
            },
        })
        self.get_logger().info(
            f'Recorded waypoint #{wp_id}: '
            f'pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), '
            f'orient=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, '
            f'{pose.orientation.z:.3f}, {pose.orientation.w:.3f})'
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
            f.write(f'    position:\n')
            f.write(f'      x: {wp["position"]["x"]}\n')
            f.write(f'      y: {wp["position"]["y"]}\n')
            f.write(f'      z: {wp["position"]["z"]}\n')
            f.write(f'    orientation:\n')
            f.write(f'      x: {wp["orientation"]["x"]}\n')
            f.write(f'      y: {wp["orientation"]["y"]}\n')
            f.write(f'      z: {wp["orientation"]["z"]}\n')
            f.write(f'      w: {wp["orientation"]["w"]}\n')


def parse_args() -> str:
    """Parse --output-file argument from command line."""
    output = 'goal_poses.yaml'
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
    node = GoalPoseRecorder(output_file)

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