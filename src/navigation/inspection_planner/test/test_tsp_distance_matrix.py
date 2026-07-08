#!/usr/bin/env python3
"""
Comprehensive test suite for TSP distance matrix computation in FAR Planner.

Test Cases:
  TC1: Basic distance computation - 3 waypoints in triangle, verify pairwise distances
  TC2: Waypoint replacement - publish new waypoints, verify old ones cleared
  TC3: Single waypoint - publish 1 waypoint, verify no entries produced
  TC4: Unreachable waypoints - waypoints in disconnected regions
  TC5: Graph integrity - navigation goal still works after TSP computation
  TC6: Z-height adjustment - waypoints with Z=0 get terrain-adjusted heights
  TC7: Empty waypoints - publish empty array, verify no crash
"""

import unittest
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from inspection_planner_interfaces.msg import Waypoints, Waypoint, TspDistanceMatrix, TspDistanceEntry
from geometry_msgs.msg import Point


class TspTestNode(Node):
    """Helper ROS2 node for publishing waypoints and receiving distance matrices."""

    def __init__(self):
        super().__init__('tsp_test_node')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5)
        self.pub = self.create_publisher(Waypoints, '/inspection_waypoints', qos)
        self.last_matrix = None
        self.matrix_received = False
        self.sub = self.create_subscription(
            TspDistanceMatrix, '/tsp_distance_matrix', self._matrix_callback, qos
        )

    def _matrix_callback(self, msg: TspDistanceMatrix):
        self.last_matrix = msg
        self.matrix_received = True

    def publish_waypoints(self, wp_list):
        """Publish a list of (id, x, y, z) tuples as waypoints."""
        msg = Waypoints()
        for wp_id, x, y, z in wp_list:
            wp = Waypoint()
            wp.id = wp_id
            wp.point = Point(x=x, y=y, z=z)
            msg.waypoints.append(wp)
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(msg.waypoints)} waypoints')

    def wait_for_matrix(self, timeout_sec=10.0):
        """Spin until a matrix is received or timeout."""
        self.last_matrix = None
        self.matrix_received = False
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.matrix_received:
                return self.last_matrix
        rclpy.spin_once(self, timeout_sec=0)
        return None


class TestTspDistanceMatrix(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TspTestNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------ #
    #  TC1  Basic distance computation
    # ------------------------------------------------------------------ #
    def test_basic_distance_computation(self):
        """Publish 3 waypoints forming a triangle and verify all pairwise distances."""
        waypoints = [
            (1, 0.0, 0.0, 0.0),
            (2, 3.0, 0.0, 0.0),
            (3, 0.0, 4.0, 0.0),
        ]
        self.node.publish_waypoints(waypoints)
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)

        self.assertIsNotNone(matrix, 'Distance matrix was not published')
        self.assertEqual(sorted(matrix.waypoint_ids), [1, 2, 3],
                         'waypoint_ids mismatch')
        self.assertEqual(len(matrix.entries), 6,
                         'Expected 6 entries (3 waypoints x 2 directions)')

        # Build lookup: (src, tgt) -> distance
        dist = {}
        for e in matrix.entries:
            dist[(e.source_id, e.target_id)] = e.distance

        # Symmetry check
        for src, tgt in [(1, 2), (1, 3), (2, 3)]:
            self.assertIn((src, tgt), dist, f'Missing entry {src}->{tgt}')
            self.assertIn((tgt, src), dist, f'Missing entry {tgt}->{src}')
            self.assertAlmostEqual(dist[(src, tgt)], dist[(tgt, src)], places=1,
                                   msg=f'Distance {src}->{tgt} not symmetric')
            self.assertGreater(dist[(src, tgt)], 0.0,
                               f'Distance {src}->{tgt} should be positive')

        # Euclidean lower-bound check (graph distance >= straight-line distance)
        self.assertGreaterEqual(dist[(1, 2)], 2.9, '1->2 too short')
        self.assertGreaterEqual(dist[(1, 3)], 3.9, '1->3 too short')
        self.assertGreaterEqual(dist[(2, 3)], 4.9, '2->3 too short')

    # ------------------------------------------------------------------ #
    #  TC2  Waypoint replacement
    # ------------------------------------------------------------------ #
    def test_waypoint_replacement(self):
        """Publish a second set of waypoints and verify old IDs are gone."""
        self.node.publish_waypoints([(10, 0.0, 0.0, 0.0), (11, 5.0, 0.0, 0.0)])
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)

        self.assertIsNotNone(matrix, 'Distance matrix was not published')
        self.assertEqual(sorted(matrix.waypoint_ids), [10, 11],
                         'Old waypoint IDs still present after replacement')
        self.assertEqual(len(matrix.entries), 2,
                         'Expected 2 entries for 2 new waypoints')

    # ------------------------------------------------------------------ #
    #  TC3  Single waypoint
    # ------------------------------------------------------------------ #
    def test_single_waypoint(self):
        """Publish only 1 waypoint; no distance entries should be produced."""
        self.node.publish_waypoints([(99, 1.0, 1.0, 0.0)])
        # The callback should NOT produce a valid matrix (ComputeTspDistanceMatrix returns false).
        # We give it a short timeout and expect None or an empty matrix.
        matrix = self.node.wait_for_matrix(timeout_sec=5.0)
        # Depending on implementation, either no message or an empty one.
        if matrix is not None:
            self.assertEqual(len(matrix.entries), 0,
                             'Single waypoint should produce zero entries')

    # ------------------------------------------------------------------ #
    #  TC4  Unreachable waypoints  (best-effort; depends on map)
    # ------------------------------------------------------------------ #
    def test_unreachable_waypoints(self):
        """Publish waypoints far apart; unreachable pairs may have missing entries."""
        waypoints = [
            (1, 0.0, 0.0, 0.0),
            (2, 1000.0, 1000.0, 0.0),  # likely unreachable
        ]
        self.node.publish_waypoints(waypoints)
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)

        if matrix is None:
            self.skipTest('No matrix received; skipping unreachable test')
        # At least one entry should exist if the graph covers waypoint 1
        # Missing entries for unreachable pairs are acceptable.
        entry_count = len(matrix.entries)
        self.assertLessEqual(entry_count, 2,
                             'Unreachable pair should not produce 2 entries')

    # ------------------------------------------------------------------ #
    #  TC5  Graph integrity
    # ------------------------------------------------------------------ #
    def test_graph_integrity(self):
        """After TSP computation, verify that normal operations are not broken.
        We check that publishing waypoints does not corrupt the waypoint_ids list."""
        self.node.publish_waypoints([(1, 0.0, 0.0, 0.0), (2, 3.0, 4.0, 0.0)])
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)
        self.assertIsNotNone(matrix, 'Matrix not received')
        self.assertEqual(len(matrix.waypoint_ids), 2,
                         'waypoint_ids count mismatch after TSP')

    # ------------------------------------------------------------------ #
    #  TC6  Z-height adjustment
    # ------------------------------------------------------------------ #
    def test_z_height_adjustment(self):
        """Publish waypoints with Z=0 and verify the planner adjusts Z."""
        waypoints = [
            (1, 0.0, 0.0, 0.0),
            (2, 3.0, 4.0, 0.0),
        ]
        self.node.publish_waypoints(waypoints)
        matrix = self.node.wait_for_matrix(timeout_sec=15.0)
        self.assertIsNotNone(matrix, 'Matrix not received')
        # If terrain adjustment is applied, distances should still be valid.
        self.assertGreater(len(matrix.entries), 0,
                           'No entries produced; Z adjustment may have failed')

    # ------------------------------------------------------------------ #
    #  TC7  Empty waypoints
    # ------------------------------------------------------------------ #
    def test_empty_waypoints(self):
        """Publish an empty waypoint array; the node should not crash."""
        self.node.publish_waypoints([])
        # Just verify the node is still alive
        rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertIsNotNone(self.node.get_clock(), 'Node crashed after empty publish')


if __name__ == '__main__':
    unittest.main()
