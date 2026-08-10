#include <inspection_planner/inspection_planner.hpp>


InspectionPlannerNode::InspectionPlannerNode()
: Node("inspection_planner_node")
{
    /* Declare and get node parameters */
    // Topics
    auto inspection_poses_sub_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
    inspection_poses_sub_topic_desc.description = "Inspection poses input topic";
    this->declare_parameter<std::string>("inspection_poses_topic", "/inspection_poses", inspection_poses_sub_topic_desc);

    auto nav_action_server_name_desc = rcl_interfaces::msg::ParameterDescriptor();
    nav_action_server_name_desc.description = "Pose navigator action server name";
    this->declare_parameter<std::string>("nav_action_server_name", "/nav_to_pose", nav_action_server_name_desc);

    auto tsp_solver_srv_name_desc = rcl_interfaces::msg::ParameterDescriptor();
    tsp_solver_srv_name_desc.description = "TSP Solver service name";
    this->declare_parameter<std::string>("tsp_solver", "/solve_tsp", tsp_solver_srv_name_desc);

    auto tsp_waypoints_pub_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
    tsp_waypoints_pub_topic_desc.description = "Inspection waypoints publish topic";
    this->declare_parameter<std::string>("tsp_waypoints_topic", "/inspection_waypoints", tsp_waypoints_pub_topic_desc);

    auto tsp_distance_matrix_sub_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
    tsp_distance_matrix_sub_topic_desc.description = "Distance Matrix subcription topic name";
    this->declare_parameter<std::string>("tsp_distance_matrix_topic", "/tsp_distance_matrix", tsp_distance_matrix_sub_topic_desc);

    auto planner_status_sub_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
    planner_status_sub_topic_desc.description = "FAR Planner planning status subscription topic";
    this->declare_parameter<std::string>("planner_status_topic", "/far_planning_status", planner_status_sub_topic_desc);

    

    // Other parameters
    auto pose_merge_distance_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor();
    pose_merge_distance_tolerance_desc.description = "Distance tolerance in meters. Poses within tolerance will be considered as the same. Both dist and angular tolerance must be passed.";
    this->declare_parameter<double>("pose_merge_distance_tolerance", 0.1, pose_merge_distance_tolerance_desc);

    auto pose_merge_angular_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor();
    pose_merge_angular_tolerance_desc.description = "Angular tolerance in radians. Poses within tolerance will be considered as the same. Both dist and angular tolerance must be passed.";
    this->declare_parameter<double>("pose_merge_angular_tolerance", 0.349066, pose_merge_angular_tolerance_desc);

    // Get parameters
    inspection_poses_sub_topic_ = this->get_parameter("inspection_poses_topic").as_string();
    nav_action_server_name_ = this->get_parameter("nav_action_server_name").as_string();
    tsp_solver_srv_ = this->get_parameter("tsp_solver").as_string();
    tsp_waypoints_pub_topic_ = this->get_parameter("tsp_waypoints_topic").as_string();
    tsp_distance_matrix_sub_topic_ = this->get_parameter("tsp_distance_matrix_topic").as_string();
    planner_status_sub_topic_ = this->get_parameter("planner_status_topic").as_string();

    pose_merge_distance_tolerance_ = this->get_parameter("pose_merge_distance_tolerance").as_double();
    pose_merge_angular_tolerance_ = this->get_parameter("pose_merge_angular_tolerance").as_double();




    /* Create ... Stuff */
    inspection_poses_sub_ = this->create_subscription<ViewPoses>(
        inspection_poses_sub_topic_, 
        5, 
        std::bind(&InspectionPlannerNode::inspection_poses_callback, this, std::placeholders::_1)
    );

    nav_action_client_ = rclcpp_action::create_client<inspection_planner_interfaces::action::NavToPose>(
        this, 
        nav_action_server_name_
    );

    tsp_solver_client_ = this->create_client<SolveTsp>(tsp_solver_srv_);

    tsp_waypoints_pub_ = this->create_publisher<Waypoints>(tsp_waypoints_pub_topic_, 1);

    tsp_distance_matrix_sub_ = this->create_subscription<TspDistanceMatrix>(
        tsp_distance_matrix_sub_topic_, 
        1, 
        std::bind(&InspectionPlannerNode::distance_matrix_callback, this, std::placeholders::_1)
    );

    planner_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        planner_status_sub_topic_, 
        5, 
        std::bind(&InspectionPlannerNode::planner_status_callback, this, std::placeholders::_1)
    );


    // Reset service
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/reset_inspection", 
        std::bind(&InspectionPlannerNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );



    // Debug topics
    next_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/inspection_debug/next_goal", 
        5
    );

}



void InspectionPlannerNode::inspection_poses_callback(const ViewPoses::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received new inspection poses. Number of poses: %ld", msg->poses.size());
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> new_pose_map;
    build_poses_map(msg, new_pose_map);

    merge_similar_poses(new_pose_map);

    filter_repeated_poses(new_pose_map, unvisited_poses_);
    filter_repeated_poses(new_pose_map, visited_poses_);
    RCLCPP_INFO(this->get_logger(), "Filtered received poses. Number of poses remaining: %ld", new_pose_map.size());
    if (new_pose_map.size() == 0) return;
    
    merge_poses_maps(unvisited_poses_, new_pose_map);
    
    Waypoints waypoints_msg;
    build_waypoints_msg(unvisited_poses_, waypoints_msg);
    tsp_waypoints_pub_->publish(waypoints_msg);
    waiting_new_tsp_result_ = true;
}

void InspectionPlannerNode::distance_matrix_callback(const TspDistanceMatrix::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Received new distance matrix. Pausing robot navigation. Requesting TSP Solver.");

    pause_inspection();

    auto request = std::make_shared<SolveTsp::Request>();
    request->matrix = *msg;
    auto future = tsp_solver_client_->async_send_request(request, std::bind(&InspectionPlannerNode::tsp_result_callback, this, std::placeholders::_1));
}

void InspectionPlannerNode::tsp_result_callback(rclcpp::Client<SolveTsp>::SharedFuture future){
    auto response = future.get();
    if (response->success){
        this->tsp_result_ = response->ordered_waypoint_ids;
        RCLCPP_INFO(this->get_logger(), "TSP solved. Ordered waypoints: %zu", this->tsp_result_.size());
        if (!response->message.empty()) {
            RCLCPP_INFO(this->get_logger(), "TSP solver message: %s", response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "TSP solver failed: %s", response->message.c_str());
        this->tsp_result_.clear();
    }
    curr_nav_tsp_index_ = 0;
    start_inspection();
}


void InspectionPlannerNode::planner_status_callback(const std_msgs::msg::Bool::SharedPtr msg){
    planner_found_path_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Planner status: %d", planner_found_path_);
    if (msg->data) return;

    Waypoints waypoints_msg;
    build_waypoints_msg(unvisited_poses_, waypoints_msg);
    tsp_waypoints_pub_->publish(waypoints_msg);
    waiting_new_tsp_result_ = true;
}



void InspectionPlannerNode::reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response){
    (void) request;
    (void) response;
    unvisited_poses_.clear();
    visited_poses_.clear();
    tsp_result_.clear();
    failed_ids_.clear();
    pause_inspection();
}






void InspectionPlannerNode::start_inspection(){
    inspection_active = true;
    RCLCPP_INFO(this->get_logger(), "Starting inspection...");
    pub_next_nav_goal();
}


void InspectionPlannerNode::pause_inspection(){
    inspection_active = false;
    if (nav_goal_handle_){
        auto status = nav_goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED || 
            nav_goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
            RCLCPP_INFO(this->get_logger(), "Cancelling navigation goal pose...");
            nav_action_client_->async_cancel_goal(nav_goal_handle_);
        }
        nav_goal_handle_ = nullptr;
    }   
}


bool InspectionPlannerNode::pub_next_nav_goal(){
    int next_pose_id = -1;
    // Search for valid nav goal pose id
    while (true){
        if (curr_nav_tsp_index_ >= tsp_result_.size()){
            // No more tsp results
            if (unvisited_poses_.size() == 0){
                RCLCPP_INFO(this->get_logger(), "All Inspection Waypoints Visited.");
                return true; // FIXME: Review this return
            } else {
                RCLCPP_INFO(this->get_logger(), "No TSP result, fall back to direct waypoint navigation.");
                next_pose_id = unvisited_poses_.begin()->first;
            }
            break;
        }
        auto next_tsp_id = tsp_result_[curr_nav_tsp_index_];
        curr_nav_tsp_index_++;
        if (unvisited_poses_.find(next_tsp_id) != unvisited_poses_.end()){
            // Found valid TSP result
            next_pose_id = next_tsp_id;
            break;
        }
        RCLCPP_ERROR(this->get_logger(), "Cannot find node in unvisited poses with ID: %d, skipping TSP result", next_tsp_id);
    }

    if (next_pose_id == -1) return false;

    if (!nav_action_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Navigation action server is not available!");
        return false; 
    }
    curr_nav_goal_ = next_pose_id;
    auto goal_pose = unvisited_poses_.at(curr_nav_goal_);
    inspection_planner_interfaces::action::NavToPose::Goal goal;
    goal.pose.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Navigating to goal pose with ID: %d", curr_nav_goal_);

    // TODO Custom header settings, ViewPoseStamped...
    goal.pose.header.stamp = this->get_clock()->now();
    goal.pose.header.frame_id = "map";

    next_goal_pub_->publish(goal.pose);

    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&InspectionPlannerNode::nav_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
        std::bind(&InspectionPlannerNode::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
        std::bind(&InspectionPlannerNode::nav_result_callback, this, std::placeholders::_1, next_pose_id);
    nav_action_client_->async_send_goal(goal, send_goal_options);

    return true;
}


void InspectionPlannerNode::nav_goal_response_callback(const NavGoalHandle::SharedPtr& goal_handle){
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server!");
        if (inspection_active) pub_next_nav_goal();
        return;
    }
    nav_goal_handle_ = goal_handle;
}

void InspectionPlannerNode::nav_feedback_callback(
    const NavGoalHandle::SharedPtr& goal_handle, 
    const std::shared_ptr<const NavToPose::Feedback> feedback
)
{
    (void)goal_handle;
    RCLCPP_DEBUG(this->get_logger(), "Received feedback from navigator, code: %d", feedback->state);
}

void InspectionPlannerNode::nav_result_callback(const NavGoalHandle::WrappedResult& result, int goal_id){
    if (!inspection_active || !nav_goal_handle_) return;
    auto node = unvisited_poses_.extract(goal_id);

    nav_goal_handle_ = nullptr;

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
        if (!node){
            RCLCPP_ERROR(this->get_logger(), "Navigated to unknown/visited inspection pose. ID: %d", goal_id);
        } else {
            visited_poses_.insert(std::move(node));
            RCLCPP_INFO(this->get_logger(), "Completed navigation to inspection pose. ID: %d", goal_id);
        }
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED){
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted! ID: %d", goal_id);
        if (!node){
            RCLCPP_ERROR(this->get_logger(), "Navigated to unknown/visited inspection pose. ID: %d", goal_id);
        }
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED){
        RCLCPP_INFO(this->get_logger(), "Goal was canceled! ID: %d", goal_id);
        if (!node){
            RCLCPP_ERROR(this->get_logger(), "Navigated to unknown/visited inspection pose. ID: %d", goal_id);
        }
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Unknown result code. ID: %d", goal_id);
        if (!node){
            RCLCPP_ERROR(this->get_logger(), "Navigated to unknown/visited inspection pose. ID: %d", goal_id);
        }
    }

    pub_next_nav_goal();
}


// My version, idk if the code is actually valid
// void InspectionPlannerNode::merge_similar_poses(std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_poses){
//     for (auto it = new_poses.begin(); it != new_poses.end();){
//         auto pose = it->second;
//         for (auto it2 = it; it2 != new_poses.end();){
//             auto pose2 = it2->second;
//             if (is_same_pose(pose, pose2)){
//                 it2 = new_poses.erase(it2);
//             }
//         }
//     }
// }



void InspectionPlannerNode::merge_similar_poses(std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_poses){
    for (auto it = new_poses.begin(); it != new_poses.end(); ++it){
        uint32_t keeper_id = it->first;
        geometry_msgs::msg::Pose& keeper_pose = it->second;

        // Collect all iterators to similar poses (including self)
        std::vector<std::pair<uint32_t, geometry_msgs::msg::Pose>> similar;
        similar.reserve(new_poses.size());
        for (auto it2 = new_poses.begin(); it2 != new_poses.end(); ++it2){
            if (is_same_pose(keeper_pose, it2->second)){
                similar.push_back({it2->first, it2->second});
            }
        }

        if (similar.size() < 2) continue;

        // Average position
        double avg_x = 0, avg_y = 0, avg_z = 0;
        for (auto& [id, p] : similar){
            avg_x += p.position.x;
            avg_y += p.position.y;
            avg_z += p.position.z;
        }
        avg_x /= similar.size();
        avg_y /= similar.size();
        avg_z /= similar.size();

        // Average orientation using quaternion mean
        double avg_qx = 0, avg_qy = 0, avg_qz = 0, avg_qw = 0;
        for (auto& [id, p] : similar){
            // Ensure quaternions point in the same hemisphere
            double sign = (similar[0].second.orientation.w >= 0) ? 1.0 : -1.0;
            avg_qx += sign * p.orientation.x;
            avg_qy += sign * p.orientation.y;
            avg_qz += sign * p.orientation.z;
            avg_qw += sign * p.orientation.w;
        }
        avg_qx /= similar.size();
        avg_qy /= similar.size();
        avg_qz /= similar.size();
        avg_qw /= similar.size();

        // Normalize
        double norm = std::sqrt(avg_qx*avg_qx + avg_qy*avg_qy + avg_qz*avg_qz + avg_qw*avg_qw);
        if (norm > 1e-6){
            avg_qx /= norm; avg_qy /= norm; avg_qz /= norm; avg_qw /= norm;
        }

        // Update keeper with averaged pose
        keeper_pose.position.x = avg_x;
        keeper_pose.position.y = avg_y;
        keeper_pose.position.z = avg_z;
        keeper_pose.orientation.x = avg_qx;
        keeper_pose.orientation.y = avg_qy;
        keeper_pose.orientation.z = avg_qz;
        keeper_pose.orientation.w = avg_qw;

        // Erase all similar poses except the keeper
        for (auto& [id, p] : similar){
            if (id != keeper_id){
                new_poses.erase(id);
            }
        }
    }
}




void InspectionPlannerNode::filter_repeated_poses(
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_poses, 
    const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& prev_poses
)
{
    for (auto it = new_poses.begin(); it != new_poses.end();){
        auto id = it->first;
        if (prev_poses.find(id) != prev_poses.end()){
            it = new_poses.erase(it);
            continue;
        }
        auto pose = it->second;
        bool found_overlap = false;
        for (auto [_, prev_pose] : prev_poses){
            if (is_same_pose(pose, prev_pose)){
                it = new_poses.erase(it);
                found_overlap = true;
                break;
            }
        }
        if (!found_overlap){
            it++;
        }
    }
}


void InspectionPlannerNode::build_poses_map(
    const ViewPoses::SharedPtr msg, 
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
)
{
    map.clear();
    for (auto pose : msg->poses){
        map[pose.id] = pose.pose;
    }
}

void InspectionPlannerNode::build_poses_msg(
    const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
    ViewPoses& msg
)
{
    for (auto [id, pose] : map){
        inspection_planner_interfaces::msg::ViewPose pose_msg;
        pose_msg.id = id;
        pose_msg.pose = pose;
        msg.poses.push_back(pose_msg);
    }
}

void InspectionPlannerNode::build_waypoints_msg(
    const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
    Waypoints& msg
)
{
    for (auto [id, pose] : map){
        inspection_planner_interfaces::msg::Waypoint waypoint;
        waypoint.id = id;
        waypoint.point.x = pose.position.x;
        waypoint.point.y = pose.position.y;
        waypoint.point.z = pose.position.z;
        msg.waypoints.push_back(waypoint);
    }
}


bool InspectionPlannerNode::is_same_pose(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2){
    // Check position distance
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    double dist_sq = (dx*dx) + (dy*dy) + (dz*dz);
    if (dist_sq > pose_merge_distance_tolerance_ * pose_merge_distance_tolerance_) return false;

    auto normalize_angle_diff = [](double diff) {
        diff = std::abs(diff);
        if (diff > M_PI) diff = 2.0 * M_PI - diff;
        return diff;
    };

    // Check yaw
    {
        double x1 = pose1.orientation.x, y1 = pose1.orientation.y,
               z1 = pose1.orientation.z, w1 = pose1.orientation.w;
        double x2 = pose2.orientation.x, y2 = pose2.orientation.y,
               z2 = pose2.orientation.z, w2 = pose2.orientation.w;

        double yaw1 = std::atan2(2.0 * (w1 * z1 + x1 * y1), 1.0 - 2.0 * (y1 * y1 + z1 * z1));
        double yaw2 = std::atan2(2.0 * (w2 * z2 + x2 * y2), 1.0 - 2.0 * (y2 * y2 + z2 * z2));
        if (normalize_angle_diff(yaw1 - yaw2) > pose_merge_angular_tolerance_) return false;
    }
    
    // No need check pitch or roll since robot can't actually control these
    // // Check pitch
    // {
    //     double x1 = pose1.orientation.x, y1 = pose1.orientation.y,
    //            z1 = pose1.orientation.z, w1 = pose1.orientation.w;
    //     double x2 = pose2.orientation.x, y2 = pose2.orientation.y,
    //            z2 = pose2.orientation.z, w2 = pose2.orientation.w;

    //     double pitch1 = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (w1 * y1 - z1 * x1))));
    //     double pitch2 = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (w2 * y2 - z2 * x2))));
    //     if (normalize_angle_diff(pitch1 - pitch2) > pose_merge_angular_tolerance_) return false;
    // }

    // // Check roll
    // {
    //     double x1 = pose1.orientation.x, y1 = pose1.orientation.y,
    //            z1 = pose1.orientation.z, w1 = pose1.orientation.w;
    //     double x2 = pose2.orientation.x, y2 = pose2.orientation.y,
    //            z2 = pose2.orientation.z, w2 = pose2.orientation.w;

    //     double roll1 = std::atan2(2.0 * (w1 * x1 + y1 * z1), 1.0 - 2.0 * (x1 * x1 + y1 * y1));
    //     double roll2 = std::atan2(2.0 * (w2 * x2 + y2 * z2), 1.0 - 2.0 * (x2 * x2 + y2 * y2));
    //     if (normalize_angle_diff(roll1 - roll2) > pose_merge_angular_tolerance_) return false;
    // }

    return true;
}



void InspectionPlannerNode::merge_poses_maps(
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map, 
    const std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& new_map
)
{
    for (const auto& [id, pose] : new_map) {
        map[id] = pose;
    }
}



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InspectionPlannerNode>();
    rclcpp::spin(node);
    if (rclcpp::ok()){
        rclcpp::shutdown();
    }
    return 0;
}