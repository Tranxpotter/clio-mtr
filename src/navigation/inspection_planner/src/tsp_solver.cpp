#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "inspection_planner/tsp_solver.hpp"


TspSolverNode::TspSolverNode()
: Node("tsp_solver_node")
{
  // ---- Declare parameters with descriptors ----
  auto service_name_desc = rcl_interfaces::msg::ParameterDescriptor();
  service_name_desc.description = "Input TSP distance matrix topic";
  this->declare_parameter<std::string>("service_name", "/solve_tsp", service_name_desc);

  auto solver_algorithm_desc = rcl_interfaces::msg::ParameterDescriptor();
  solver_algorithm_desc.description =
    "TSP solver algorithm: nearest_neighbor, random_nearest, brute_force";
  this->declare_parameter<std::string>("solver_algorithm", "nearest_neighbor", solver_algorithm_desc);

  auto random_seed_desc = rcl_interfaces::msg::ParameterDescriptor();
  random_seed_desc.description = "Random seed for random_nearest algorithm";
  this->declare_parameter<int>("random_seed", 42, random_seed_desc);

  auto brute_force_max_nodes_desc = rcl_interfaces::msg::ParameterDescriptor();
  brute_force_max_nodes_desc.description =
    "Maximum number of waypoints for brute_force (falls back to nearest_neighbor above this)";
  this->declare_parameter<int>("brute_force_max_nodes", 10, brute_force_max_nodes_desc);

  auto symmetry_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor();
  symmetry_tolerance_desc.description =
    "Relative tolerance for symmetry warning: |d_ij - d_ji| / max(d_ij, d_ji) > tolerance triggers warning";
  this->declare_parameter<double>("symmetry_tolerance", 0.01, symmetry_tolerance_desc);

  auto qos_depth_desc = rcl_interfaces::msg::ParameterDescriptor();
  qos_depth_desc.description = "QoS depth for subscriber and publisher";
  this->declare_parameter<int>("qos_depth", 1, qos_depth_desc);

  // ---- Read parameter values ----
  service_name_        = this->get_parameter("service_name").as_string();
  solver_algorithm_   = this->get_parameter("solver_algorithm").as_string();
  random_seed_        = this->get_parameter("random_seed").as_int();
  brute_force_max_nodes_ = this->get_parameter("brute_force_max_nodes").as_int();
  symmetry_tolerance_ = this->get_parameter("symmetry_tolerance").as_double();
  qos_depth_          = this->get_parameter("qos_depth").as_int();


  srv_ = this->create_service<inspection_planner_interfaces::srv::SolveTsp>(
    "/solve_tsp", std::bind(&TspSolverNode::on_service_called, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
    "TSP Solver started — algorithm=%s, service=%s",
    solver_algorithm_.c_str(), service_name_.c_str());
}



void TspSolverNode::on_service_called(
  const inspection_planner_interfaces::srv::SolveTsp::Request::SharedPtr request, 
  inspection_planner_interfaces::srv::SolveTsp::Response::SharedPtr response
)
{
  start_id_      = request->matrix.start_id;
  auto waypoint_ids = request->matrix.waypoint_ids;
  auto entries      = request->matrix.entries;

  RCLCPP_INFO(this->get_logger(),
    "Received distance matrix: start_id=%u, %lu waypoint IDs, %lu entries",
    start_id_, waypoint_ids.size(), entries.size());

  // ---- Handle trivial cases ----
  if (waypoint_ids.empty()) {
    RCLCPP_WARN(this->get_logger(), "TSP: empty waypoint list, ignoring.");
    return;
  }

  if (waypoint_ids.size() == 1) {
    RCLCPP_INFO(this->get_logger(), "TSP: single waypoint (%u), publishing as-is.", waypoint_ids[0]);
    response->ordered_waypoint_ids.reserve(1);
    response->ordered_waypoint_ids.push_back(waypoint_ids[0]);
    response->success = true;
    return;
  }

  // ---- Filter unreachable waypoints using connected component ----
  std::vector<uint32_t> reachable_waypoints = find_connected_component(start_id_, waypoint_ids, entries);
  std::vector<uint32_t> filtered_ids;
  if (reachable_waypoints.size() < waypoint_ids.size()) {
    std::unordered_set<uint32_t> reachable_set(reachable_waypoints.begin(), reachable_waypoints.end());
    for (uint32_t id : waypoint_ids) {
      if (!reachable_set.count(id)) {
        filtered_ids.push_back(id);
      }
    }
    RCLCPP_WARN(this->get_logger(),
      "TSP: %lu of %lu waypoints reachable from robot (start_id=%u), filtering %lu unreachable",
      reachable_waypoints.size(), waypoint_ids.size(), start_id_, filtered_ids.size());
  }

  // ---- Normalize the distance matrix on the reachable subset ----
  std::map<std::pair<uint32_t, uint32_t>, double> dist_map;
  std::vector<uint32_t> unreachable_pairs;
  std::vector<uint32_t> working_waypoints = reachable_waypoints;

  if (!normalize_matrix(working_waypoints, entries, dist_map, unreachable_pairs)) {
    // Edge case: subgraph not fully connected — iteratively remove most-disconnected waypoint
    RCLCPP_WARN(this->get_logger(),
      "TSP: reachable subgraph has %lu unreachable pairs, attempting iterative repair.",
      unreachable_pairs.size());
    for (int iteration = 0; iteration < (int) working_waypoints.size(); ++iteration) {
      // Count how many missing pairs each waypoint participates in
      std::map<uint32_t, int> miss_count;
      for (uint32_t id : unreachable_pairs) {
        miss_count[id]++;
      }
      // Remove the waypoint with the most missing entries
      uint32_t worst = 0;
      int worst_count = 0;
      for (auto const& [id, count] : miss_count) {
        if (count > worst_count) {
          worst_count = count;
          worst = id;
        }
      }
      auto it = std::find(working_waypoints.begin(), working_waypoints.end(), worst);
      if (it != working_waypoints.end()) {
        working_waypoints.erase(it);
        filtered_ids.push_back(worst);
      }
      if (working_waypoints.size() < 2) break;
      if (normalize_matrix(working_waypoints, entries, dist_map, unreachable_pairs)) {
        RCLCPP_INFO(this->get_logger(), "TSP: iterative repair succeeded after removing %lu waypoints.", filtered_ids.size() - (reachable_waypoints.size() < waypoint_ids.size() ? (waypoint_ids.size() - reachable_waypoints.size()) : 0));
        break;
      }
    }
    if (!unreachable_pairs.empty() && working_waypoints.size() >= 2) {
      RCLCPP_ERROR(this->get_logger(), "TSP: matrix normalization still failed after repair. Cannot solve.");
      return;
    }
  }

  // ---- Handle trivial cases after filtering ----
  if (working_waypoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "TSP: no reachable waypoints after filtering.");
    response->success = false;
    response->message = "No reachable waypoints from robot position.";
    return;
  }

  if (working_waypoints.size() == 1) {
    RCLCPP_INFO(this->get_logger(), "TSP: single reachable waypoint (%u), returning as-is.", working_waypoints[0]);
    response->ordered_waypoint_ids.push_back(working_waypoints[0]);
    response->success = true;
    if (!filtered_ids.empty()) {
      response->message = "Filtered " + std::to_string(filtered_ids.size()) + " unreachable waypoints.";
    }
    return;
  }

  // ---- Solve TSP based on selected algorithm ----
  std::vector<uint32_t> order;
  if (solver_algorithm_ == "brute_force") {
    order = solve_brute_force(working_waypoints, dist_map);
  } else if (solver_algorithm_ == "random_nearest") {
    order = solve_random_nearest(working_waypoints, dist_map);
  } else {
    // Default to nearest_neighbor (also used as fallback)
    order = solve_nearest_neighbor(working_waypoints, dist_map);
  }

  // ---- Publish the result ----
  response->ordered_waypoint_ids.reserve(order.size());

  for (const auto& id : order) {
    response->ordered_waypoint_ids.push_back(id);
  }
  response->success = true;

  if (!filtered_ids.empty()) {
    std::string ids_str;
    for (size_t i = 0; i < filtered_ids.size(); ++i) {
      if (i > 0) ids_str += ", ";
      ids_str += std::to_string(filtered_ids[i]);
    }
    response->message = "Filtered " + std::to_string(filtered_ids.size()) + " unreachable waypoints: [" + ids_str + "]";
  }

  // Log the full path including robot start
  std::string order_str = "TSP path: robot(" + std::to_string(start_id_) + ") -> ";
  for (size_t i = 0; i < order.size(); ++i) {
    order_str += std::to_string(order[i]);
    if (i + 1 < order.size()) order_str += " -> ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", order_str.c_str());

  // Log the open-path cost
  double cost = open_tour_cost(order, dist_map);
  RCLCPP_INFO(this->get_logger(), "TSP open-path cost = %.4f", cost);

  RCLCPP_INFO(this->get_logger(), "Responded with %lu ordered waypoint ids.",
    response->ordered_waypoint_ids.size());
}

std::vector<uint32_t> TspSolverNode::find_connected_component(
  uint32_t start_id,
  const std::vector<uint32_t>& waypoint_ids,
  const std::vector<inspection_planner_interfaces::msg::TspDistanceEntry>& entries) const
{
  // Scan entries for all entries where source_id == start_id
  // The corresponding target_ids are the waypoints reachable from the robot
  std::unordered_set<uint32_t> reachable_targets;
  for (const auto& e : entries) {
    if (e.source_id == start_id) {
      reachable_targets.insert(e.target_id);
    }
  }

  // Filter waypoint_ids to only those in the reachable set
  std::vector<uint32_t> result;
  result.reserve(waypoint_ids.size());
  for (uint32_t id : waypoint_ids) {
    if (reachable_targets.count(id)) {
      result.push_back(id);
    }
  }

  return result;
}

bool TspSolverNode::normalize_matrix(
  const std::vector<uint32_t>& waypoint_ids,
  const std::vector<inspection_planner_interfaces::msg::TspDistanceEntry>& entries,
  std::map<std::pair<uint32_t, uint32_t>, double>& dist_map,
  std::vector<uint32_t>& unreachable_pairs)
{
  dist_map.clear();
  unreachable_pairs.clear();

  // Build a set of valid waypoint IDs for validation
  std::unordered_set<uint32_t> id_set(waypoint_ids.begin(), waypoint_ids.end());

  // Insert all raw entries into the map, preserving message order (first wins on duplicates)
  for (const auto& e : entries) {
    auto key = std::make_pair(e.source_id, e.target_id);
    if (dist_map.find(key) == dist_map.end()) {
      dist_map[key] = e.distance;
    }
    // Duplicates in the message are silently ignored (first entry wins)
  }

  // For every pair (i, j) with i != j, ensure both directions exist
  for (size_t a = 0; a < waypoint_ids.size(); ++a) {
    for (size_t b = 0; b < waypoint_ids.size(); ++b) {
      if (a == b) continue;

      uint32_t i = waypoint_ids[a];
      uint32_t j = waypoint_ids[b];
      auto fwd  = std::make_pair(i, j);
      auto rev  = std::make_pair(j, i);

      bool has_fwd = (dist_map.find(fwd) != dist_map.end());
      bool has_rev = (dist_map.find(rev) != dist_map.end());

      if (has_fwd && has_rev) {
        // Both exist — check symmetry
        double d_fwd = dist_map.at(fwd);
        double d_rev = dist_map.at(rev);
        double max_d = std::max(std::abs(d_fwd), std::abs(d_rev));
        if (max_d > 1e-9) {
          double rel_diff = std::abs(d_fwd - d_rev) / max_d;
          if (rel_diff > symmetry_tolerance_) {
            RCLCPP_WARN(this->get_logger(),
              "TSP: asymmetric distance %u<->%u: %.4f vs %.4f (rel_diff=%.4f > tol=%.4f). Keeping earlier value.",
              i, j, d_fwd, d_rev, rel_diff, symmetry_tolerance_);
          }
        }
        // Keep the earlier value (already in map from first insertion)
      } else if (has_fwd) {
        // Missing reverse — fill from forward
        dist_map[rev] = dist_map.at(fwd);
        RCLCPP_DEBUG(this->get_logger(),
          "TSP: filled missing reverse edge %u->%u = %.4f (from %u->%u)",
          j, i, dist_map.at(fwd), i, j);
      } else if (has_rev) {
        // Missing forward — fill from reverse
        dist_map[fwd] = dist_map.at(rev);
        RCLCPP_DEBUG(this->get_logger(),
          "TSP: filled missing forward edge %u->%u = %.4f (from %u->%u)",
          i, j, dist_map.at(rev), j, i);
      } else {
        // Neither direction exists — unreachable pair
        unreachable_pairs.push_back(i);
        unreachable_pairs.push_back(j);
        RCLCPP_ERROR(this->get_logger(),
          "TSP: no distance entry for %u<->%u — unreachable pair.", i, j);
      }
    }
  }

  if (!unreachable_pairs.empty()) {
    return false;
  }
  return true;
}

// ------------------------------------------------------------------ //
//  Nearest-neighbor greedy heuristic (open path from robot position)
// ------------------------------------------------------------------ //
std::vector<uint32_t> TspSolverNode::solve_nearest_neighbor(
  const std::vector<uint32_t>& waypoint_ids,
  const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map)
{
  std::vector<uint32_t> order;
  std::unordered_set<uint32_t> visited;
  order.reserve(waypoint_ids.size());

  // Start from the robot's position (start_id_) — pick the nearest unvisited waypoint
  uint32_t current = start_id_;
  double best_dist = std::numeric_limits<double>::max();
  uint32_t best_next = 0;

  for (const auto& wp : waypoint_ids) {
    auto key = std::make_pair(current, wp);
    auto it  = dist_map.find(key);
    if (it != dist_map.end() && it->second < best_dist) {
      best_dist = it->second;
      best_next = wp;
    }
  }

  if (best_next == 0) {
    RCLCPP_WARN(this->get_logger(), "TSP: nearest_neighbor cannot reach any waypoint from robot (start_id=%u).", current);
    return order;
  }

  visited.insert(best_next);
  order.push_back(best_next);
  current = best_next;

  while (order.size() < waypoint_ids.size()) {
    best_dist = std::numeric_limits<double>::max();
    best_next = 0;

    for (const auto& wp : waypoint_ids) {
      if (visited.count(wp)) continue;
      auto key = std::make_pair(current, wp);
      auto it  = dist_map.find(key);
      if (it != dist_map.end() && it->second < best_dist) {
        best_dist = it->second;
        best_next = wp;
      }
    }

    if (best_next == 0) {
      // No reachable unvisited node — should not happen if matrix is complete
      RCLCPP_WARN(this->get_logger(), "TSP: nearest_neighbor stuck at node %u, no reachable next node.", current);
      break;
    }

    visited.insert(best_next);
    order.push_back(best_next);
    current = best_next;
  }

  return order;
}

// ------------------------------------------------------------------ //
//  Random-start nearest-neighbor (open path from robot position)
// ------------------------------------------------------------------ //
std::vector<uint32_t> TspSolverNode::solve_random_nearest(
  const std::vector<uint32_t>& waypoint_ids,
  const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map)
{
  std::mt19937 rng(random_seed_);

  // First step: deterministically pick the nearest waypoint from the robot (start_id_)
  double best_dist = std::numeric_limits<double>::max();
  uint32_t best_next = 0;

  for (const auto& wp : waypoint_ids) {
    auto key = std::make_pair(start_id_, wp);
    auto it  = dist_map.find(key);
    if (it != dist_map.end() && it->second < best_dist) {
      best_dist = it->second;
      best_next = wp;
    }
  }

  if (best_next == 0) {
    RCLCPP_WARN(this->get_logger(), "TSP: random_nearest cannot reach any waypoint from robot (start_id=%u).", start_id_);
    return {};
  }

  RCLCPP_INFO(this->get_logger(),
    "TSP: random_nearest starting from robot (start_id=%u) → first waypoint %u (seed=%d)",
    start_id_, best_next, random_seed_);

  // Build the rest of the path using nearest-neighbor
  std::vector<uint32_t> order;
  std::unordered_set<uint32_t> visited;
  order.reserve(waypoint_ids.size());

  visited.insert(best_next);
  order.push_back(best_next);
  uint32_t current = best_next;

  while (order.size() < waypoint_ids.size()) {
    best_dist = std::numeric_limits<double>::max();
    best_next = 0;

    for (const auto& wp : waypoint_ids) {
      if (visited.count(wp)) continue;
      auto key = std::make_pair(current, wp);
      auto it  = dist_map.find(key);
      if (it != dist_map.end() && it->second < best_dist) {
        best_dist = it->second;
        best_next = wp;
      }
    }

    if (best_next == 0) {
      RCLCPP_WARN(this->get_logger(), "TSP: random_nearest stuck at node %u.", current);
      break;
    }

    visited.insert(best_next);
    order.push_back(best_next);
    current = best_next;
  }

  return order;
}

// ------------------------------------------------------------------ //
//  Brute-force (all permutations)
// ------------------------------------------------------------------ //
std::vector<uint32_t> TspSolverNode::solve_brute_force(
  const std::vector<uint32_t>& waypoint_ids,
  const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map)
{
  size_t n = waypoint_ids.size();

  if (n > static_cast<size_t>(brute_force_max_nodes_)) {
    RCLCPP_WARN(this->get_logger(),
      "TSP: brute_force requested but %lu nodes > max %d. Falling back to nearest_neighbor.",
      n, brute_force_max_nodes_);
    return solve_nearest_neighbor(waypoint_ids, dist_map);
  }

  RCLCPP_INFO(this->get_logger(),
    "TSP: brute_force solving for %lu waypoints (%lu permutations)",
    n, static_cast<unsigned long>(std::tgamma(n + 1)));

  std::vector<uint32_t> best_order;
  double best_cost = std::numeric_limits<double>::max();

  std::vector<uint32_t> ids = waypoint_ids;
  std::sort(ids.begin(), ids.end());  // deterministic iteration order

  do {
    // Use open-path cost: start_id → wp[0] → wp[1] → ... → wp[n-1] (no return leg)
    double cost = open_tour_cost(ids, dist_map);
    if (cost < best_cost) {
      best_cost = cost;
      best_order = ids;
    }
  } while (std::next_permutation(ids.begin(), ids.end()));

  RCLCPP_INFO(this->get_logger(), "TSP: brute_force best open-path cost = %.4f", best_cost);
  return best_order;
}

// ------------------------------------------------------------------ //
//  Tour cost helper (closed tour — returns to start)
// ------------------------------------------------------------------ //
double TspSolverNode::tour_cost(
  const std::vector<uint32_t>& order,
  const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map) const
{
  double cost = 0.0;
  for (size_t i = 0; i < order.size(); ++i) {
    uint32_t from = order[i];
    uint32_t to   = order[(i + 1) % order.size()];
    auto key = std::make_pair(from, to);
    auto it  = dist_map.find(key);
    if (it != dist_map.end()) {
      cost += it->second;
    } else {
      cost += std::numeric_limits<double>::max();  // unreachable
    }
  }
  return cost;
}

// ------------------------------------------------------------------ //
//  Open-path cost: start_id → wp[0] → wp[1] → ... → wp[n-1]
// ------------------------------------------------------------------ //
double TspSolverNode::open_tour_cost(
  const std::vector<uint32_t>& order,
  const std::map<std::pair<uint32_t, uint32_t>, double>& dist_map) const
{
  if (order.empty()) return 0.0;

  double cost = 0.0;

  // Leg 1: robot (start_id_) → first waypoint
  {
    auto key = std::make_pair(start_id_, order[0]);
    auto it  = dist_map.find(key);
    if (it != dist_map.end()) {
      cost += it->second;
    } else {
      cost += std::numeric_limits<double>::max();  // unreachable
    }
  }

  // Legs 2..n: wp[i] → wp[i+1]
  for (size_t i = 0; i + 1 < order.size(); ++i) {
    uint32_t from = order[i];
    uint32_t to   = order[i + 1];
    auto key = std::make_pair(from, to);
    auto it  = dist_map.find(key);
    if (it != dist_map.end()) {
      cost += it->second;
    } else {
      cost += std::numeric_limits<double>::max();  // unreachable
    }
  }

  return cost;
}




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TspSolverNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()){
    rclcpp::shutdown();
  }
  return 0;
}
