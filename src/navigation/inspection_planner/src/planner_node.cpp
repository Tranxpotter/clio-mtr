#include <string>
#include <sstream>
#include <chrono>
#include <functional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "visibility_graph_msg/msg/graph.hpp"
#include "visibility_graph_msg/msg/node.hpp"

#include "inspection_planner_interfaces/msg/waypoint.hpp"
#include "inspection_planner_interfaces/msg/waypoints.hpp"

struct CustomGraphNode;
typedef std::shared_ptr<CustomGraphNode> CustomGraphNodePtr;

struct CustomGraphNode {
    uint32_t id;
    int freetype;
    Eigen::Vector3d position;
    std::vector<Eigen::Vector3d> surface_dirs;
    bool is_covered;
    bool is_frontier;
    bool is_navpoint;
    bool is_boundary;
    bool is_waypoint = false;
    std::vector<CustomGraphNodePtr> connect_nodes;
    std::vector<CustomGraphNodePtr> poly_connects;
    std::vector<CustomGraphNodePtr> contour_connects;
    std::vector<CustomGraphNodePtr> trajectory_connects;
};

struct Waypoint {
    uint32_t id;
    Eigen::Vector3d position;
};


class InspectionPlannerNode : public rclcpp::Node 
{
    public: 
        InspectionPlannerNode()
        : Node("inspection_planner_node")
        {
            vgraph_sub_ = this->create_subscription<visibility_graph_msg::msg::Graph>("/robot_vgraph", 5, std::bind(&InspectionPlannerNode::vgraph_callback, this, std::placeholders::_1));
            waypoints_sub_ = this->create_subscription<inspection_planner_interfaces::msg::Waypoints>("/inspection_waypoints", 5, std::bind(&InspectionPlannerNode::waypoints_callback, this, std::placeholders::_1));
            
        }
    

    private:
        rclcpp::Subscription<visibility_graph_msg::msg::Graph>::SharedPtr vgraph_sub_;
        rclcpp::Subscription<inspection_planner_interfaces::msg::Waypoints>::SharedPtr waypoints_sub_;
        
        std::unordered_map<uint32_t, CustomGraphNodePtr> global_graph_;
        std::vector<std::shared_ptr<Waypoint>> inspection_waypoints_;
        std::unordered_map<uint32_t, CustomGraphNodePtr> prev_graph_;
        std::vector<std::shared_ptr<Waypoint>> prev_waypoints_;
        uint32_t max_node_id = 0;
        uint32_t waypoint_count = 0;


        /**
         * @brief Rebuild global_map with received vgraph
         * 
         * @param msg 
         */
        void vgraph_callback(const visibility_graph_msg::msg::Graph::SharedPtr msg){
            prev_graph_ = global_graph_;
            global_graph_.clear();

            std::vector<std::pair<visibility_graph_msg::msg::Node::SharedPtr, CustomGraphNodePtr>> msg_node_pairs;
            for (const auto& msg_node : msg->nodes){
                CustomGraphNodePtr node = create_node(msg_node);
                if (add_node_to_graph(node)){
                    msg_node_pairs.emplace_back(msg_node, node);
                }
                if (msg_node.id >= max_node_id){
                    max_node_id = msg_node.id;
                }
            }

            for (const auto& p : msg_node_pairs){
                auto msg_node = p.first;
                auto node = p.second;
                assign_connect_nodes(msg_node->connect_nodes, node->connect_nodes);
                assign_connect_nodes(msg_node->poly_connects, node->poly_connects);
                assign_connect_nodes(msg_node->contour_connects, node->contour_connects);
                assign_connect_nodes(msg_node->trajectory_connects, node->trajectory_connects);
            }

            evaluate_change();
        }

        /**
         * @brief Create a custom graph node
         * 
         * @param node_msg input node message
         * @return CustomGraphNodePtr 
         */
        CustomGraphNodePtr create_node(const visibility_graph_msg::msg::Node& node_msg){
            CustomGraphNodePtr node = std::make_shared<CustomGraphNode>();
            node->id = node_msg.id;
            node->freetype = node_msg.freetype;
            node->position = Eigen::Vector3d(node_msg.position.x, node_msg.position.y, node_msg.position.z);
            
            for (const geometry_msgs::msg::Point& surface_dir_msg : node_msg.surface_dirs){
                node->surface_dirs.emplace_back(surface_dir_msg.x, surface_dir_msg.y, surface_dir_msg.z);
            }

            node->is_covered = node_msg.is_covered;
            node->is_frontier = node_msg.is_frontier;
            node->is_navpoint = node_msg.is_navpoint;
            node->is_boundary = node_msg.is_boundary;

            return node;
        }

        /**
         * @brief Add node connections
         * 
         * @param node_ids [in] vector of node ids to be added
         * @param node_connections [out] vector of connected nodes
         */
        void assign_connect_nodes(const std::vector<uint32_t> node_ids, std::vector<CustomGraphNodePtr>& connected_nodes){
            for (uint32_t node_id : node_ids){
                const auto it = global_graph_.find(node_id);
                if (it == global_graph_.end()){
                    continue;
                }
                connected_nodes.push_back(it->second);
            }
        }

        /**
         * @brief Add custom graph node to graph, returns if successful
         * 
         * @param node 
         * @return true 
         * @return false 
         */
        bool add_node_to_graph(CustomGraphNodePtr node){
            if (node == nullptr){
                return false;
            }
            global_graph_[node->id] = node;
            return true;
        }



        /* =============== Waypoints =================== */

        /**
         * @brief Create a waypoint object
         * 
         * @param waypoint_msg 
         * @return std::shared_ptr<Waypoint> 
         */
        std::shared_ptr<Waypoint> create_waypoint(const inspection_planner_interfaces::msg::Waypoint& waypoint_msg){
            auto waypoint = std::make_shared<Waypoint>();
            waypoint->id = waypoint_msg.id;
            waypoint->position = Eigen::Vector3d(waypoint_msg.point.x, waypoint_msg.point.y, waypoint_msg.point.z);
            return waypoint;
        }

        void waypoints_callback(const inspection_planner_interfaces::msg::Waypoints& msg){
            prev_waypoints_ = inspection_waypoints_;
            inspection_waypoints_.clear();
            for (const auto& waypoint_msg : msg.waypoints){
                inspection_waypoints_.push_back(create_waypoint(waypoint_msg));
            }
            evaluate_change();
        }


        /**
         * @brief evaluate whether a tsp needs to be done based on changes in graph and waypoints
         * 
         */
        void evaluate_change(){
            // TODO: evaluate to be implemented

            compute_order();
        }

        /** Compute inspection waypoint ordering */
        void compute_order(){
            std::unordered_map<uint32_t, CustomGraphNodePtr> new_graph = global_graph_;
            for (auto waypoint : inspection_waypoints_){
                add_waypoint_to_graph(waypoint, new_graph);
            }
            

        }

        /**
         * @brief Add waypoint to graph
         * 
         * @param waypoint [in] waypoint
         * @param graph [out] graph to add waypoint to
         */
        void add_waypoint_to_graph(std::shared_ptr<Waypoint> waypoint, std::unordered_map<uint32_t, CustomGraphNodePtr>& graph){
            uint32_t waypoint_id = max_node_id + waypoint_count + 1;
            waypoint_count++;

            CustomGraphNodePtr waypoint_node = std::make_shared<CustomGraphNode>();
            waypoint_node->id = waypoint_id;
            waypoint_node->position = waypoint->position;
            waypoint_node->freetype = 0; // Default freetype status
            waypoint_node->is_waypoint = true;
            waypoint_node->is_navpoint = true; // Set to true so planning queries recognize it as a valid navigation node
            waypoint_node->is_covered = true;
            waypoint_node->is_frontier = false;
            waypoint_node->is_boundary = false;

            // Connection threshold configuration (similar to kLocalPlanRange / sensor_range in FAR Planner)
            const double CONNECTION_RANGE_THRES = 10.0; // meters
            
            CustomGraphNodePtr nearest_node = nullptr;
            double min_dist = std::numeric_limits<double>::max();

            // Iterate through the existing graph nodes to evaluate connectivity criteria
            for (auto const& [id, graph_node] : graph) {
                // Skip connecting to other newly added virtual waypoints during this pass if desired,
                // or connect only to physical navigation nodes
                if (!graph_node->is_navpoint) continue;

                double dist = (waypoint_node->position - graph_node->position).norm();

                // Track the absolute closest node to guarantee graph connectivity even if out of threshold range
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_node = graph_node;
                }

                // If within the neighborhood range, establish a bidirectional edge connection
                if (dist <= CONNECTION_RANGE_THRES) {
                    waypoint_node->connect_nodes.push_back(graph_node);
                    graph_node->connect_nodes.push_back(waypoint_node);
                    
                    // Mirroring FAR Planner's multi-type tracking if applicable to the inspection task
                    if (graph_node->is_boundary) {
                        waypoint_node->contour_connects.push_back(graph_node);
                        graph_node->contour_connects.push_back(waypoint_node);
                    }
                }
            }

            // Fallback strategy: If no nodes were found within CONNECTION_RANGE_THRES,
            // connect to the single closest node to prevent an isolated graph component
            if (waypoint_node->connect_nodes.empty() && nearest_node != nullptr) {
                waypoint_node->connect_nodes.push_back(nearest_node);
                nearest_node->connect_nodes.push_back(waypoint_node);
                
                if (nearest_node->is_boundary) {
                    waypoint_node->contour_connects.push_back(nearest_node);
                    nearest_node->contour_connects.push_back(waypoint_node);
                }
            }

            // Explicitly insert the newly configured waypoint node into the graph map reference
            graph[waypoint_id] = waypoint_node;
        }

        
};

