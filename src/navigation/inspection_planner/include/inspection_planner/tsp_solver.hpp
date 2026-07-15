#ifndef INSPECTION_PLANNER__TSP_SOLVER_HPP_
#define INSPECTION_PLANNER__TSP_SOLVER_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <inspection_planner_interfaces/msg/tsp_distance_matrix.hpp>
#include <inspection_planner_interfaces/msg/tsp_distance_entry.hpp>
#include <inspection_planner_interfaces/msg/waypoints.hpp>
#include <inspection_planner_interfaces/msg/waypoint.hpp>
#include <inspection_planner_interfaces/srv/solve_tsp.hpp>

class TspSolverNode : public rclcpp::Node
{
public:
  TspSolverNode();

private:
  // ------------------------------------------------------------------ //
  //  Callbacks
  // ------------------------------------------------------------------ //
  void on_service_called(
    const inspection_planner_interfaces::srv::SolveTsp::Request::SharedPtr request, 
    inspection_planner_interfaces::srv::SolveTsp::Response::SharedPtr response);

  // ------------------------------------------------------------------ //
  //  Matrix normalization
  // ------------------------------------------------------------------ //
  /// Populate a symmetric distance lookup from raw entries.
  /// Returns true if the matrix is usable (all pairs reachable or n < 2).
  bool normalize_matrix(
    const std::vector<uint32_t>& waypoint_ids,
    const std::vector<inspection_planner_interfaces::msg::TspDistanceEntry>& entries,
    std::map<std::pair<uint32_t, uint32_t>, double>& dist_map,
    std::vector<uint32_t>& unreachable_pairs);

  /// Find waypoints reachable from the robot (start_id) using distance matrix entries.
  /// Returns the subset of waypoint_ids that have a distance entry from start_id.
  std::vector<uint32_t> find_connected_component(
    uint32_t start_id,
    const std::vector<uint32_t>& waypoint_ids,
    const std::vector<inspection_planner_interfaces::msg::TspDistanceEntry>& entries) const;

  // ------------------------------------------------------------------ //
  //  Solver algorithms
  // ------------------------------------------------------------------ //
  /// Nearest-neighbor greedy heuristic (deterministic start from first ID).
  std::vector<uint32_t> solve_nearest_neighbor(
    const std::vector<uint32_t>& waypoint_ids,
    const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map);

  /// Random-start nearest-neighbor (uses random_seed_).
  std::vector<uint32_t> solve_random_nearest(
    const std::vector<uint32_t>& waypoint_ids,
    const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map);

  /// Brute-force (all permutations). Falls back to nearest_neighbor if n > brute_force_max_nodes_.
  std::vector<uint32_t> solve_brute_force(
    const std::vector<uint32_t>& waypoint_ids,
    const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map);

  /// Compute total tour cost for a given ordering (closed tour — returns to start).
  double tour_cost(const std::vector<uint32_t>& order,
                   const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map) const;

  /// Compute open-path cost: start_id → wp[0] → wp[1] → ... → wp[n-1] (no return leg).
  double open_tour_cost(const std::vector<uint32_t>& order,
                        const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map) const;

  // ------------------------------------------------------------------ //
  //  Attributes
  // ------------------------------------------------------------------ //
  rclcpp::Service<inspection_planner_interfaces::srv::SolveTsp>::SharedPtr srv_;

  uint32_t start_id_;  // Robot's current position ID (reserved value 0)

  std::string service_name_;
  std::string solver_algorithm_;
  int random_seed_;
  int brute_force_max_nodes_;
  double symmetry_tolerance_;
  int qos_depth_;
};

#endif  // INSPECTION_PLANNER__TSP_SOLVER_HPP_
