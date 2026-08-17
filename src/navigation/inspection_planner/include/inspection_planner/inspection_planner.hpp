#ifndef INSPECTION_PLANNER__INSPECTION_PLANNER_HPP_
#define INSPECTION_PLANNER__INSPECTION_PLANNER_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <inspection_planner_interfaces/msg/tsp_distance_matrix.hpp>
#include <inspection_planner_interfaces/msg/tsp_distance_entry.hpp>
#include <inspection_planner_interfaces/msg/waypoints.hpp>
#include <inspection_planner_interfaces/msg/waypoint.hpp>
#include <inspection_planner_interfaces/msg/view_poses.hpp>
#include <inspection_planner_interfaces/msg/view_pose.hpp>
#include <inspection_planner_interfaces/srv/solve_tsp.hpp>
#include <inspection_planner_interfaces/action/nav_to_pose.hpp>

using NavToPose = inspection_planner_interfaces::action::NavToPose;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;
using ViewPose = inspection_planner_interfaces::msg::ViewPose;
using ViewPoses = inspection_planner_interfaces::msg::ViewPoses;
using SolveTsp = inspection_planner_interfaces::srv::SolveTsp;
using Waypoints = inspection_planner_interfaces::msg::Waypoints;
using TspDistanceMatrix = inspection_planner_interfaces::msg::TspDistanceMatrix;

class InspectionPlannerNode : public rclcpp::Node
{
    public: 
        InspectionPlannerNode();
    
    private:
        rclcpp::Subscription<ViewPoses>::SharedPtr inspection_poses_sub_;               // Inspection poses input topic
        rclcpp_action::Client<NavToPose>::SharedPtr nav_action_client_;              // Goal pose navigator action server client
        rclcpp::Client<SolveTsp>::SharedPtr tsp_solver_client_;                         // tsp solver client
        rclcpp::Publisher<Waypoints>::SharedPtr tsp_waypoints_pub_;                     // Publishes inspection waypoints to far planner tsp distance matrix calculator
        rclcpp::Subscription<TspDistanceMatrix>::SharedPtr tsp_distance_matrix_sub_;    // Get tsp distance matrix
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_status_sub_;       // Get FAR Planner planning status
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

        // Debug pubs
        rclcpp::Publisher<ViewPose>::SharedPtr next_goal_pub_;

        // TODO: Visualizer topics

        std::string inspection_poses_sub_topic_;    // inspection_poses_sub_
        std::string nav_action_server_name_;        // nav_action_client_
        std::string tsp_solver_srv_;                // tsp_solver_client_
        std::string tsp_waypoints_pub_topic_;       // tsp_waypoints_pub_
        std::string tsp_distance_matrix_sub_topic_; // tsp_distance_matrix_sub_
        std::string planner_status_sub_topic_;      // 

        // Other params
        double pose_merge_distance_tolerance_;
        double pose_merge_angular_tolerance_;
        double redo_tsp_period_;
        int success_count_for_retry_threshold_;


        /* Subscriber and Server Callbacks */

        /**
         * @brief Filter incoming poses and trigger recalculation of tsp if needed
         * 
         * @param msg 
         */
        void inspection_poses_callback(const ViewPoses::SharedPtr msg);

        /**
         * @brief Trigger get new tsp results
         * 
         * @param msg 
         */
        void distance_matrix_callback(const TspDistanceMatrix::SharedPtr msg);

        /**
         * @brief Get TSP result and start navigation
         * 
         * @param future_cmd 
         */
        void tsp_result_callback(rclcpp::Client<SolveTsp>::SharedFuture future_cmd);

        /**
         * @brief If planner status failed immediately do tsp again
         * 
         * @param msg 
         */
        void planner_status_callback(const std_msgs::msg::Bool::SharedPtr msg);

        /**
         * @brief Reset inspection progress and clear all inspection poses
         * 
         * @param request 
         * @param response 
         */
        void reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

        



        /* Attributes */
        std::unordered_map<uint32_t, geometry_msgs::msg::Pose> unvisited_poses_;
        std::unordered_map<uint32_t, geometry_msgs::msg::Pose> visited_poses_;
        std::unordered_map<uint32_t, geometry_msgs::msg::Pose> failed_poses_;
        std::vector<uint32_t> tsp_result_;

        rclcpp::TimerBase::SharedPtr redo_tsp_timer_{nullptr};

        bool waiting_new_tsp_result_ = false;
        int curr_nav_tsp_index_ = 0;
        uint8_t nav_state_ = 0;
        bool allow_pub_new_waypoint_ = true;

        int success_count_for_retry_ = 0;
        


        /* Navigation Handlers and Callbacks */
        NavGoalHandle::SharedPtr nav_goal_handle_;
        int curr_nav_goal_ = -1;
        bool inspection_active = false;
        bool planner_found_path_ = true;

        void pause_inspection();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool pub_next_nav_goal();

        void nav_goal_response_callback(const NavGoalHandle::SharedPtr& goal_handle);

        void nav_feedback_callback(const NavGoalHandle::SharedPtr& goal_handle, const std::shared_ptr<const NavToPose::Feedback> feedback);

        void nav_result_callback(const NavGoalHandle::WrappedResult& result, int goal_id);

        /**
         * @brief Publish unvisited waypoints for new distance matrix
         * 
         * @return true 
         * @return false 
         */
        void get_new_distance_matrix();

        /* Helper funcs */
        
        /**
         * @brief Merge inspection poses that are similar to each other
         * 
         * @param new_poses [in/out] incoming poses
         */
        void merge_similar_poses(std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_poses);

        /**
         * @brief filter out new poses that already appeared in prev poses based on ID and pose
         * 
         * @param new_poses [in/out] poses to be filtered
         * @param prev_poses [in] old poses
         */
        void filter_repeated_poses(
            std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_poses, 
            const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& prev_poses
        );
        
        /**
         * @brief Build poses map from ViewPoses message, key=id, value=ViewPose
         * 
         * @param msg [in] incoming message
         * @param map [out] resulting map
         */
        void build_poses_map(
            const ViewPoses::SharedPtr msg, 
            std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
        );
        
        /**
         * @brief Build ViewPoses message from pose map
         * 
         * @param map [in] input map
         * @param msg [out] output message
         */
        void build_poses_msg(
            const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
            ViewPoses& msg
        );

        /**
         * @brief Build Waypoints message from pose map
         * 
         * @param map [in] input map
         * @param msg [out] output message
         */
        void build_waypoints_msg(
            const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
            Waypoints& msg
        );

        /**
         * @brief Returns True if pose1 equals pose2
         * 
         * @param pose1 
         * @param pose2 
         * @return true 
         * @return false 
         */
        bool is_same_pose(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

        /**
         * @brief Merge new_map into map
         * 
         * @param map [in/out] map to be added to
         * @param new_map [in] entries to be added
         */
        void merge_poses_maps(
            std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
            const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_map
        );
        
        
        
};


#endif