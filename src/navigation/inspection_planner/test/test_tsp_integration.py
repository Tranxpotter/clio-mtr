#!/usr/bin/env python3
"""
Phase 2 integration test: publish waypoints and verify the distance matrix.

Runs after FAR Planner is launched with a test map.
Checks:
  1. waypoint_ids contain all published IDs
  2. entries count matches expected pairs
  3. distances are symmetric within tolerance
  4. distances are positive for distinct waypoints
  5. old TSP nodes are cleared when new waypoints arrive
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from inspection_planner_interfaces.msg import Waypoints, Waypoint, TspDistanceMatrix
from geometry_msgs.msg import Point


class TspIntegrationNode(Node):

    def __init__(self):
        super().__init__('tsp_integration_test_node')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5)
        self.pub = self.create_publisher(Waypoints, '/inspection_waypoints', qos)
        self.last_matrix = None
        self.sub = self.create_subscription(
            TspDistanceMatrix, '/tsp_distance_matrix', self._cb, qos
        )

    def _cb(self, msg):
        self.last_matrix = msg

    def publish_waypoints(self, wp_list):
        msg = Waypoints()
        for wp_id, x, y, z in wp_list:
            wp = Waypoint()
            wp.id = wp_id
            wp.point = Point(x=x, y=y, z=z)
            msg.waypoints.append(wp)
        self.pub.publish(msg)

    def wait_for_matrix(self, timeout_sec=10.0):
        self.last_matrix = None
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_matrix is not None:
                return self.last_matrix
        return None


class TestTspIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TspIntegrationNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_waypoint_ids_and_entry_count(self):
        self.node.publish_waypoints([
            (1, 0.0, 0.0, 0.0),
            (2, 3.0, 0.0, 0.0),
            (3, 0.0, 4.0, 0.0),
        ])
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)
        self.assertIsNotNone(matrix, 'No distance matrix received')
        self.assertEqual(sorted(matrix.waypoint_ids), [1, 2, 3])
        self.assertEqual(len(matrix.entries), 6)

    def test_distance_symmetry(self):
        dist = {}
        for e in matrix.entries:
            dist[(e.source_id, e.target_id)] = e.distance
        for src, tgt in [(1, 2), (1, 3), (2, 3)]:
            self.assertAlmostEqual(dist.get((src, tgt)), dist.get((tgt, src)),
                                   places=1, msg=f'{src}<->{tgt} not symmetric')

    def test_distance_positive(self):
        for e in matrix.entries:
            self.assertGreater(e.distance, 0.0,
                               f'Distance {e.source_id}->{e.target_id} should be > 0')

    def test_old_nodes_cleared_on_new_publish(self):
        self.node.publish_waypoints([(10, 0.0, 0.0, 0.0), (11, 5.0, 0.0, 0.0)])
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)
        self.assertIsNotNone(matrix)
        self.assertEqual(sorted(matrix.waypoint_ids), [10, 11])


if __name__ == '__main__':
    unittest.main()
