#!/usr/bin/env python3
"""
Phase 1 test: verify that TSP message definitions are correctly built.

Checks:
  1. TspDistanceEntry has fields: source_id, target_id, distance
  2. TspDistanceMatrix has fields: header, waypoint_ids, entries
"""

import unittest
from inspection_planner_interfaces.msg import TspDistanceEntry, TspDistanceMatrix


class TestMessageDefinitions(unittest.TestCase):

    def test_tsp_distance_entry_fields(self):
        entry = TspDistanceEntry()
        entry.source_id = 1
        entry.target_id = 2
        entry.distance = 5.3
        self.assertEqual(entry.source_id, 1)
        self.assertEqual(entry.target_id, 2)
        self.assertAlmostEqual(entry.distance, 5.3, places=1)

    def test_tsp_distance_matrix_fields(self):
        matrix = TspDistanceMatrix()
        matrix.waypoint_ids = [1, 2, 3]
        e = TspDistanceEntry(source_id=1, target_id=2, distance=3.0)
        matrix.entries.append(e)
        self.assertEqual(len(matrix.waypoint_ids), 3)
        self.assertEqual(len(matrix.entries), 1)
        self.assertEqual(matrix.entries[0].source_id, 1)


if __name__ == '__main__':
    unittest.main()
