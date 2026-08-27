#ifndef INSPECTION_PLANNER__LOGGER_HPP_
#define INSPECTION_PLANNER__LOGGER_HPP_

#include <unordered_map>
#include <string>
#include <vector>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <fstream>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>

#include <inspection_planner_interfaces/msg/log_msg.hpp>
#include <inspection_planner_interfaces/msg/view_poses.hpp>
#include <inspection_planner_interfaces/msg/view_pose.hpp>
#include <inspection_planner_interfaces/msg/view_poses_debug.hpp>
#include <inspection_planner_interfaces/srv/log_displacement.hpp>



using ViewPoses = inspection_planner_interfaces::msg::ViewPoses;


class LoggerNode : public rclcpp::Node 
{
    public:
    LoggerNode();
    void write_inspection_summary();

    private:
    // Parameters


    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
    rclcpp::Subscription<inspection_planner_interfaces::msg::LogMsg>::SharedPtr log_msg_sub_;
    rclcpp::Subscription<inspection_planner_interfaces::msg::ViewPosesDebug>::SharedPtr viewposes_debug_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr received_poses_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr added_poses_sub_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viewpoints_viz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viewposes_viz_pub_;

    // Services
    rclcpp::Service<inspection_planner_interfaces::srv::LogDisplacement>::SharedPtr log_displacement_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Attributes
    uint32_t total_poses_received_ = 0;
    uint32_t total_poses_added_ = 0;
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> unvisited_poses_;
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> visited_poses_;
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> failed_poses_;
    std::vector<uint32_t> ordered_ids_;

    // File paths and streams
    std::string root_dir_ = ROOT_DIR;
    std::ofstream log_file_stream_;
    std::ofstream csv_post_rotation_stream_;
    std::ofstream csv_pre_rotation_stream_;
    std::ofstream csv_pre_adjustment_stream_;
    std::ofstream summary_stream_;

    // Init Methods
    void files_init();
    void communications_init();

    // Callbacks
    void msg_callback(const std_msgs::msg::String::SharedPtr msg);

    void log_msg_callback(const inspection_planner_interfaces::msg::LogMsg::SharedPtr msg);

    void poses_debug_callback(const inspection_planner_interfaces::msg::ViewPosesDebug::SharedPtr msg);
    void pub_marker_array();
    void pub_pose_array();

    void received_poses_callback(const std_msgs::msg::UInt32::SharedPtr msg);

    void added_poses_callback(const std_msgs::msg::UInt32::SharedPtr msg);

    void log_displacement_callback(
        const inspection_planner_interfaces::srv::LogDisplacement::Request::SharedPtr req, 
        inspection_planner_interfaces::srv::LogDisplacement::Response::SharedPtr res
    );

    void reset_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr req, 
        std_srvs::srv::Trigger::Response::SharedPtr res
    );

    // Helper funcs
    /**
     * @brief Get ROS2-style timestamped prefix for log file
     * Format: [<timestamp>] [<level>] inspection_planner: 
     */
    std::string get_log_prefix(const uint8_t& level, const rclcpp::Time& time = rclcpp::Time());
    const std::unordered_map<uint8_t, std::string> LOG_LEVEL_MAP = {
        {0, "INFO"}, 
        {1, "WARNING"}, 
        {2, "ERROR"}
    };

    /**
     * @brief Build poses map from ViewPoses message, key=id, value=ViewPose
     * 
     * @param msg [in] incoming message
     * @param map [out] resulting map
     */
    void build_poses_map(const ViewPoses::SharedPtr msg, std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map);
    void build_poses_map(const ViewPoses msg, std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map);
    /**
     * @brief Build ViewPoses message from pose map
     * 
     * @param map [in] input map
     * @param msg [out] output message
     */
    void build_poses_msg(const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, ViewPoses& msg);

};

#endif