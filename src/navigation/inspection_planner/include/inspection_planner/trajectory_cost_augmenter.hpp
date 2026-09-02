#ifndef INSPECTION_PLANNER__TRAJECTORY_COST_AUGMENTER_HPP_
#define INSPECTION_PLANNER__TRAJECTORY_COST_AUGMENTER_HPP_

#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>

#include <inspection_planner_interfaces/msg/tsp_distance_matrix.hpp>
#include <inspection_planner_interfaces/msg/tsp_distance_entry.hpp>
#include <inspection_planner_interfaces/msg/waypoints.hpp>
#include <inspection_planner_interfaces/msg/waypoint.hpp>
#include <inspection_planner_interfaces/msg/view_poses.hpp>
#include <inspection_planner_interfaces/msg/view_pose.hpp>
#include <inspection_planner_interfaces/srv/inject_trajectory_cost.hpp>

using ViewPose = inspection_planner_interfaces::msg::ViewPose;
using ViewPoses = inspection_planner_interfaces::msg::ViewPoses;
using Waypoints = inspection_planner_interfaces::msg::Waypoints;
using TspDistanceMatrix = inspection_planner_interfaces::msg::TspDistanceMatrix;
using InjectTrajectoryCost = inspection_planner_interfaces::srv::InjectTrajectoryCost;

class TrajectoryCostAugmenter : public rclcpp::Node
{
    public: 
        TrajectoryCostAugmenter();
    
    private:
        // Parameters
        double fixed_entry_cost_; // Fixed cost added to every entry 
        double cost_per_radian_; // m/rad rotation cost added
        
        // Service
        rclcpp::Service<InjectTrajectoryCost>::SharedPtr trajectory_cost_srv_;

        // Callbacks
        
        /**
         * @brief Adds trajectory cost to distance matrix, and respond with the modified distance matrix
         * 
         * @param req 
         * @param res 
         */
        void trajectory_cost_service_callback(const InjectTrajectoryCost::Request::SharedPtr req, InjectTrajectoryCost::Response::SharedPtr res);
        
        /**
         * @brief Calculate added trajectory cost from source pose to target pose
         * 
         * @param source 
         * @param target 
         * @return double 
         */
        double calculate_cost(const geometry_msgs::msg::Pose& source, const geometry_msgs::msg::Pose& target);
    
        /**
         * @brief Build poses map from ViewPoses message, key=id, value=ViewPose
         * 
         * @param msg [in] incoming message
         * @param map [out] resulting map
         */
        void build_poses_map(
            const ViewPoses& msg, 
            std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
        );


};


#endif
