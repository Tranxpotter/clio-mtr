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
#include <inspection_planner_interfaces/msg/waypoint_ids.hpp>
#include <inspection_planner_interfaces/msg/waypoints.hpp>
#include <inspection_planner_interfaces/msg/waypoint.hpp>

class TspSolverNode : public rclcpp::Node
{
public:
  TspSolverNode();

private:
  // ------------------------------------------------------------------ //
  //  Callbacks
  // ------------------------------------------------------------------ //
  void on_distance_matrix(const inspection_planner_interfaces::msg::TspDistanceMatrix::SharedPtr msg);

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

  /// Compute total tour cost for a given ordering.
  double tour_cost(const std::vector<uint32_t>& order,
                   const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map) const;

  // ------------------------------------------------------------------ //
  //  Result publishing
  // ------------------------------------------------------------------ //
  void publish_result(const std::vector<uint32_t>& order,
                      const std::vector<uint32_t>& waypoint_ids,
                      const std::vector<inspection_planner_interfaces::msg::TspDistanceEntry>& entries);

  // ------------------------------------------------------------------ //
  //  ROS handles
  // ------------------------------------------------------------------ //
  rclcpp::Subscription<inspection_planner_interfaces::msg::TspDistanceMatrix>::SharedPtr sub_;
  rclcpp::Publisher<inspection_planner_interfaces::msg::WaypointIds>::SharedPtr pub_;

  // ------------------------------------------------------------------ //
  //  Parameters
  // ------------------------------------------------------------------ //
  std::string input_topic_;
  std::string output_topic_;
  std::string solver_algorithm_;
  int random_seed_;
  int brute_force_max_nodes_;
  double symmetry_tolerance_;
  int qos_depth_;
};

#endif  // INSPECTION_PLANNER__TSP_SOLVER_HPP_
