#include <csignal>
#include <inspection_planner/inspection_planner.hpp>

// Global pointer for signal handler
static InspectionPlannerNode* g_inspection_node = nullptr;

static void shutdown_handler(int) {
    if (g_inspection_node) {
        g_inspection_node->write_inspection_summary();
    }
    rclcpp::shutdown();
    exit(0);
}

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

    auto redo_tsp_period_desc = rcl_interfaces::msg::ParameterDescriptor();
    redo_tsp_period_desc.description = "Recalculates TSP result on this period while navigating.";
    this->declare_parameter<double>("redo_tsp_period", 5.0, redo_tsp_period_desc);

    auto success_count_for_retry_threshold_desc = rcl_interfaces::msg::ParameterDescriptor();
    success_count_for_retry_threshold_desc.description = "Adds failed poses to tsp calculation if threshold amount of viewposes successfully reached.";
    this->declare_parameter<int>("success_count_for_retry_threshold", 5, success_count_for_retry_threshold_desc);

    auto use_tsp_desc = rcl_interfaces::msg::ParameterDescriptor();
    use_tsp_desc.description = "Whether to use TSP for ordering waypoints or not. If not, navigate by waypoint ids ascending.";
    this->declare_parameter<bool>("use_tsp", true, use_tsp_desc);

    // Get parameters
    inspection_poses_sub_topic_ = this->get_parameter("inspection_poses_topic").as_string();
    nav_action_server_name_ = this->get_parameter("nav_action_server_name").as_string();
    tsp_solver_srv_ = this->get_parameter("tsp_solver").as_string();
    tsp_waypoints_pub_topic_ = this->get_parameter("tsp_waypoints_topic").as_string();
    tsp_distance_matrix_sub_topic_ = this->get_parameter("tsp_distance_matrix_topic").as_string();
    planner_status_sub_topic_ = this->get_parameter("planner_status_topic").as_string();

    pose_merge_distance_tolerance_ = this->get_parameter("pose_merge_distance_tolerance").as_double();
    pose_merge_angular_tolerance_ = this->get_parameter("pose_merge_angular_tolerance").as_double();
    redo_tsp_period_ = this->get_parameter("redo_tsp_period").as_double();
    success_count_for_retry_threshold_ = this->get_parameter("success_count_for_retry_threshold").as_int();
    use_tsp_ = this->get_parameter("use_tsp").as_bool();



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
    next_goal_pub_ = this->create_publisher<ViewPose>(
        "/inspection_debug/next_goal", 
        5
    );

    waypoint_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/inspection_debug/waypoints", 
        5
    );


    // TF Init
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    logger_init();
    csv_logger_init();
}





void InspectionPlannerNode::inspection_poses_callback(const ViewPoses::SharedPtr msg){
    total_poses_received_ += static_cast<uint32_t>(msg->poses.size());
    this->log_msg("Received new inspection poses. Number of poses: " + std::to_string(msg->poses.size()));
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> new_pose_map;
    build_poses_map(msg, new_pose_map);

    merge_similar_poses(new_pose_map);

    filter_repeated_poses(new_pose_map, unvisited_poses_);
    filter_repeated_poses(new_pose_map, visited_poses_);
    this->log_msg("Filtered received poses. Number of poses remaining: " + std::to_string(new_pose_map.size()));
    if (new_pose_map.size() == 0) return;
    
    merge_poses_maps(unvisited_poses_, new_pose_map);
    total_poses_added_ += static_cast<uint32_t>(new_pose_map.size());

    this->retrying_failed_poses_ = false; //TODO: Add inspection initailization method
    
    get_new_waypoints_order();
}

void InspectionPlannerNode::distance_matrix_callback(const TspDistanceMatrix::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Received new distance matrix. Pausing robot navigation. Requesting TSP Solver.");

    auto request = std::make_shared<SolveTsp::Request>();
    request->matrix = *msg;
    auto future = tsp_solver_client_->async_send_request(request, std::bind(&InspectionPlannerNode::tsp_result_callback, this, std::placeholders::_1));
}

void InspectionPlannerNode::tsp_result_callback(rclcpp::Client<SolveTsp>::SharedFuture future){
    auto response = future.get();
    if (response->success){
        this->tsp_result_ = response->ordered_waypoint_ids;
        this->log_msg("TSP solved. Ordered waypoints: " + std::to_string(this->tsp_result_.size()));
        if (!response->message.empty()) {
            this->log_msg("TSP solver message: " + response->message);
        }
    } else {
        this->log_msg("TSP solver failed: %s" + response->message);
        this->tsp_result_.clear();
    }
    waiting_new_tsp_result_ = false;

    // For every viewpose that is not in tsp result, put them in the 'failed' list
    std::string failed_poses_log;
    for (auto itr = unvisited_poses_.begin(); itr != unvisited_poses_.end();){
        auto pose = *itr;
        bool found = false;
        for (auto valid_pose_id : tsp_result_){
            if (valid_pose_id == pose.first){
                found = true;
                break;
            }
        }
        if (!found){
            failed_poses_log += std::to_string(pose.first) + " ";
            failed_poses_[pose.first] = pose.second;
            itr = unvisited_poses_.erase(itr);
        } else {
            itr++;
        }
    }

    if (!failed_poses_log.empty()){
        RCLCPP_INFO(this->get_logger(), "The following waypoints failed: %s", failed_poses_log.c_str());
    }

    if (tsp_result_.empty()){
        inspection_active = false;
        cancel_current_goal();
        RCLCPP_INFO(this->get_logger(), "Inspection complete."); // TODO: inspection statistics, placeholder next waypoint?
        return;
    }

    curr_nav_tsp_index_ = 0;
    inspection_active = true;
    pub_next_nav_goal();
}


void InspectionPlannerNode::planner_status_callback(const std_msgs::msg::Bool::SharedPtr msg){
    planner_found_path_ = msg->data;
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Planner status: %d", planner_found_path_);
        return;
    }
    this->log_error_msg("Planner failed to find a path.");
    cancel_current_goal();
}



void InspectionPlannerNode::reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response){
    (void) request;
    (void) response;
    this->log_msg("Resetting inspection...");
    inspection_active = false;
    unvisited_poses_.clear();
    visited_poses_.clear();
    failed_poses_.clear();
    tsp_result_.clear();
    total_poses_received_ = 0;
    total_poses_added_ = 0;
    cancel_current_goal();
}


void InspectionPlannerNode::cancel_current_goal(){
    if (nav_goal_handle_){
        auto status = nav_goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED || 
            nav_goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
            RCLCPP_INFO(this->get_logger(), "Cancelling navigation goal pose...");
            nav_action_client_->async_cancel_goal(nav_goal_handle_);
        }
    }
}


bool InspectionPlannerNode::pub_next_nav_goal(){
    int next_pose_id = -1;
    // Search for valid nav goal pose id
    while (true){
        if (curr_nav_tsp_index_ >= tsp_result_.size()){
            this->log_msg("No more TSP result, requesting for new waypoints order.");
            this->get_new_waypoints_order();
            return true;
        }
        auto next_tsp_id = tsp_result_[curr_nav_tsp_index_];
        curr_nav_tsp_index_++;
        if (unvisited_poses_.find(next_tsp_id) != unvisited_poses_.end()){
            // Found valid TSP result
            next_pose_id = next_tsp_id;
            break;
        }
        this->log_error_msg("Cannot find node in unvisited poses with ID: " + std::to_string(next_tsp_id) + ", skipping TSP result");
    }

    if (next_pose_id == -1) return false;

    if (!nav_action_client_->action_server_is_ready()) {
        this->log_error_msg("Navigation action server is not available!");
        return false; 
    }
    if (curr_nav_goal_ == next_pose_id){
        return false;
    }
    curr_nav_goal_ = next_pose_id;
    auto goal_pose = unvisited_poses_.at(curr_nav_goal_);
    inspection_planner_interfaces::action::NavToPose::Goal goal;
    goal.pose.pose = goal_pose;

    this->log_msg("Navigating to goal pose with ID: " + std::to_string(curr_nav_goal_));

    // TODO Custom header settings, ViewPoseStamped...
    goal.pose.header.stamp = this->get_clock()->now();
    goal.pose.header.frame_id = "map";

    // Debug pose
    ViewPose debug_pose;
    debug_pose.pose = goal.pose.pose;
    debug_pose.id = next_pose_id;
    next_goal_pub_->publish(debug_pose);

    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&InspectionPlannerNode::nav_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
        std::bind(&InspectionPlannerNode::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
        std::bind(&InspectionPlannerNode::nav_result_callback, this, std::placeholders::_1, next_pose_id);
    nav_action_client_->async_send_goal(goal, send_goal_options);

    if (use_tsp_){
        // Redo tsp timer
        if (this->redo_tsp_timer_ && !this->redo_tsp_timer_->is_canceled()){
            this->redo_tsp_timer_->cancel();
        }
        this->redo_tsp_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->redo_tsp_period_), 
            [this](){
                if (allow_pub_new_waypoint_ && inspection_active && !waiting_new_tsp_result_){
                    get_new_distance_matrix();
                } else {
                    this->redo_tsp_timer_->cancel();
                }
            }
        );
    }

    return true;
}


void InspectionPlannerNode::nav_goal_response_callback(const NavGoalHandle::SharedPtr& goal_handle){
    if (!goal_handle) {
        this->log_error_msg("Goal was rejected by the action server!");
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
    nav_state_ = feedback->state;
    RCLCPP_DEBUG(this->get_logger(), "Received feedback from navigator, code: %d", feedback->state);
    if (feedback->state > 1){
        RCLCPP_INFO(this->get_logger(), "Waiting for adjusting / rotating: %d", feedback->state);
        allow_pub_new_waypoint_ = false;
    } else {
        allow_pub_new_waypoint_ = true;
    }
    // State constants from goal_rotator_node
    const int ADJUSTING = 2;
    const int ROTATING = 3;

    if (curr_nav_goal_ < 0 || unvisited_poses_.find(curr_nav_goal_) == unvisited_poses_.end()) {
        return;
    }
    geometry_msgs::msg::Pose target_pose = unvisited_poses_.at(curr_nav_goal_);

    if (feedback->state == ROTATING) {
        log_displacement(target_pose, "Unknown", DisplacementPhase::PRE_ROTATION);
    } else if (feedback->state == ADJUSTING) {
        log_displacement(target_pose, "Unknown", DisplacementPhase::PRE_ADJUSTMENT);
    }
}

void InspectionPlannerNode::nav_result_callback(const NavGoalHandle::WrappedResult& result, int goal_id){
    if (!inspection_active || !nav_goal_handle_) return;
    auto node = unvisited_poses_.extract(goal_id);

    nav_goal_handle_ = nullptr;
    if (node) {
        std::string nav_result = (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? "Success" : "Failed";
        this->log_displacement(node.mapped(), nav_result, DisplacementPhase::POST_ROTATION);
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            visited_poses_.insert(std::move(node));
            this->success_count_for_retry_++;
            RCLCPP_INFO(this->get_logger(), "Completed navigation to inspection pose. ID: %d", goal_id);
        }
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED){
        this->log_error_msg("Goal was aborted! ID: " + std::to_string(goal_id));
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            this->failed_poses_.insert(std::move(node));
        }
        log_waypoint_failed();
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED){
        this->log_msg("Goal was canceled! ID: " + std::to_string(goal_id));
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            this->failed_poses_.insert(std::move(node));
        }
        log_waypoint_failed();
    }
    else{
        this->log_error_msg("Unknown result code. ID: " + std::to_string(goal_id));
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            this->failed_poses_.insert(std::move(node));
        }
        log_waypoint_failed();
    }
    update_visualization();
    
    if (inspection_active){
        pub_next_nav_goal();
    }
}


void InspectionPlannerNode::get_new_waypoints_order(){
    if (use_tsp_){
        this->get_new_distance_matrix();
        return;
    }

    this->tsp_result_.clear();
    this->curr_nav_tsp_index_ = 0;
    // No TSP inspection terminating condition
    if (unvisited_poses_.size() == 0){
        // End inspection if already retried failed poses
        if (this->retrying_failed_poses_ == true || failed_poses_.size() == 0){
            this->log_msg("Inspection complete.");
            return;
        } else {
            // Retry failed poses
            this->retrying_failed_poses_ = true;
            this->move_failed_to_unvisited();
            this->log_msg("Retrying failed poses...");
        }
    }

    // No TSP, order unvisited waypoints by waypoint ids
    for (auto [id, pose] : unvisited_poses_){
        this->tsp_result_.push_back(id);
    }
    std::sort(this->tsp_result_.begin(), this->tsp_result_.end());
    RCLCPP_INFO(this->get_logger(), "Not using TSP solver, number of unvisited viewposes: %zu", this->tsp_result_.size());

    // Build ordered waypoint message
    std::string msg;
    for (auto id : this->tsp_result_){
        msg += std::to_string(id) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Ordered waypoints: %s", msg.c_str());
    update_visualization();
    inspection_active = true;
    pub_next_nav_goal();
}


void InspectionPlannerNode::get_new_distance_matrix(){
    if (unvisited_poses_.size() == 0){
        if (failed_poses_.size() == 0){
            inspection_active = false;
            RCLCPP_INFO(this->get_logger(), "Inspection complete. All inspection poses visited."); // TODO: placeholder next waypoint?
            return;
        }
        move_failed_to_unvisited();
    }

    if (this->success_count_for_retry_ >= this->success_count_for_retry_threshold_){
        move_failed_to_unvisited();
        this->success_count_for_retry_ = 0;
    }

    Waypoints waypoints_msg;
    build_waypoints_msg(unvisited_poses_, waypoints_msg);
    tsp_waypoints_pub_->publish(waypoints_msg);
    waiting_new_tsp_result_ = true;
    return;
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


void InspectionPlannerNode::move_failed_to_unvisited(){
    for (auto& [id, pose] : failed_poses_){
        unvisited_poses_[id] = pose;
    }
    failed_poses_.clear();
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






std::string get_datetime_string() {
    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    // Convert to local time structure
    std::tm local_tm = *std::localtime(&now_time);
    
    // Stream into string using format specifiers
    std::stringstream ss;
    ss << std::put_time(&local_tm, "%Y-%m-%dT%H_%M_%S");
    return ss.str();
}

/* Logging Functions */
void InspectionPlannerNode::logger_init(){
    std::string datetime = get_datetime_string();
    std::string log_file_path = root_dir_ + "logs/" + datetime + ".log";
    log_file_stream_.open(log_file_path);
    RCLCPP_INFO(this->get_logger(), "Saving logs to %s", log_file_path.c_str());
}

void InspectionPlannerNode::csv_logger_init(){
    std::string datetime = get_datetime_string();
    std::string analysis_dir = root_dir_ + "analysis/";
    std::filesystem::create_directories(analysis_dir);

    std::string csv_header = "ViewPose_ID,dx_m,dy_m,dz_m,euclidean_m,yaw_diff_deg,yaw_diff_rad,curr_x,curr_y,curr_z,curr_yaw_deg,target_x,target_y,target_z,target_yaw_deg,Result";

    csv_post_rotation_stream_.open(analysis_dir + datetime + "_post_rotation.csv");
    csv_post_rotation_stream_ << csv_header << std::endl;

    csv_pre_rotation_stream_.open(analysis_dir + datetime + "_pre_rotation.csv");
    csv_pre_rotation_stream_ << csv_header << std::endl;

    csv_pre_adjustment_stream_.open(analysis_dir + datetime + "_pre_adjustment.csv");
    csv_pre_adjustment_stream_ << csv_header << std::endl;

    // Summary log file
    summary_stream_.open(analysis_dir + datetime + "_summary.log");

    RCLCPP_INFO(this->get_logger(), "Saving CSV analysis to %s", analysis_dir.c_str());
}

void InspectionPlannerNode::write_inspection_summary(){
    if (!summary_stream_.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Summary file is not opened, cannot write summary!");
        return;
    }

    uint32_t successful = static_cast<uint32_t>(visited_poses_.size());
    uint32_t failed = static_cast<uint32_t>(failed_poses_.size());
    uint32_t unvisited = static_cast<uint32_t>(unvisited_poses_.size());
    bool complete = (unvisited_poses_.empty());

    double pct_remaining = (total_poses_received_ > 0) ? (static_cast<double>(total_poses_added_) / total_poses_received_ * 100.0) : 0.0;
    double pct_success = (total_poses_added_ > 0) ? (static_cast<double>(successful) / total_poses_added_ * 100.0) : 0.0;
    double pct_failed = (total_poses_added_ > 0) ? (static_cast<double>(failed) / total_poses_added_ * 100.0) : 0.0;
    double pct_unvisited = (total_poses_added_ > 0) ? (static_cast<double>(unvisited) / total_poses_added_ * 100.0) : 0.0;

    summary_stream_ << "========================================" << std::endl;
    summary_stream_ << "INSPECTION SUMMARY" << std::endl;
    summary_stream_ << "========================================" << std::endl;
    summary_stream_ << "Inspection Complete: " << (complete ? "Yes" : "No") << std::endl;
    summary_stream_ << "Total Waypoints Inputted: " << total_poses_received_ << std::endl;
    summary_stream_ << "Total Unique (Filtered) Waypoints: " << total_poses_added_ << std::endl;
    summary_stream_ << std::fixed << std::setprecision(2);
    summary_stream_ << "Percentage Remaining After Filtering: " << pct_remaining << "%" << std::endl;
    summary_stream_ << "Success: " << successful << std::endl;
    summary_stream_ << "Failed: " << failed << std::endl;
    summary_stream_ << "Not Visited: " << unvisited << std::endl;
    summary_stream_ << "Success Rate: " << pct_success << "%" << std::endl;
    summary_stream_ << "Failure Rate: " << pct_failed << "%" << std::endl;
    summary_stream_ << "Not Visited Rate: " << pct_unvisited << "%" << std::endl;
    summary_stream_ << "========================================" << std::endl;
    summary_stream_.flush();

    RCLCPP_INFO(this->get_logger(), "Inspection summary written to analysis folder.");

    // Close all analysis streams
    if (summary_stream_.is_open()) summary_stream_.close();
    if (csv_post_rotation_stream_.is_open()) csv_post_rotation_stream_.close();
    if (csv_pre_rotation_stream_.is_open()) csv_pre_rotation_stream_.close();
    if (csv_pre_adjustment_stream_.is_open()) csv_pre_adjustment_stream_.close();
}



void InspectionPlannerNode::log_waypoint(){
    if (!log_file_stream_.is_open()){
        RCLCPP_WARN(this->get_logger(), "Log file is not opened!");
        return;
    }
    log_file_stream_ << "Navigating to waypoint: " << this->curr_nav_goal_ << std::endl;
}

void InspectionPlannerNode::log_waypoint_failed(){
    if (!log_file_stream_.is_open()){
        RCLCPP_WARN(this->get_logger(), "Log file is not opened!");
        return;
    }
    log_file_stream_ << "Failed to navigate to waypoint: " << this->curr_nav_goal_ << std::endl;
}

void InspectionPlannerNode::log_displacement(const geometry_msgs::msg::Pose& target_pose, const std::string& result, DisplacementPhase phase){
    // 1. Get current robot pose from TF
    geometry_msgs::msg::Pose current_pose;
    try{
        auto transform = tf_buffer_->lookupTransform("map", "robot_footprint", tf2::TimePointZero);
        current_pose.position.x = transform.transform.translation.x;
        current_pose.position.y = transform.transform.translation.y;
        current_pose.position.z = transform.transform.translation.z;
        current_pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException &ex) {
        this->log_error_msg("Failed to get tf transform from map to robot_footprint: " + std::string(ex.what()));
        return;
    }

    // 2. Translation displacement
    double dx = current_pose.position.x - target_pose.position.x;
    double dy = current_pose.position.y - target_pose.position.y;
    double dz = current_pose.position.z - target_pose.position.z;
    double euc_dist = std::sqrt(dx*dx + dy*dy);

    // 3. Angular (yaw) displacement normalized to [-pi, pi]
    auto get_yaw = [](const geometry_msgs::msg::Quaternion& q) -> double {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    };
    double current_yaw = get_yaw(current_pose.orientation);
    double target_yaw  = get_yaw(target_pose.orientation);
    double raw_yaw_diff = current_yaw - target_yaw;
    double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

    // 4. Determine phase name and target CSV stream
    std::string phase_name;
    std::ofstream& csv_stream = (phase == DisplacementPhase::PRE_ROTATION)   ? csv_pre_rotation_stream_
                                  : (phase == DisplacementPhase::PRE_ADJUSTMENT) ? csv_pre_adjustment_stream_
                                                                                  : csv_post_rotation_stream_;

    if (phase == DisplacementPhase::PRE_ROTATION)   phase_name = "Pre-Rotation";
    else if (phase == DisplacementPhase::PRE_ADJUSTMENT) phase_name = "Pre-Adjustment";
    else phase_name = "Post-Rotation";

    // 5. Write to .log file
    if (log_file_stream_.is_open()) {
        log_file_stream_
            << "Viewpose ID: " << curr_nav_goal_
            << " | Phase: " << phase_name
            << " | Translation: " << std::fixed << std::setprecision(3)
            << "dx=" << dx << "m dy=" << dy << "m dz=" << dz << "m euclidean=" << euc_dist << "m"
            << " | Angular: " << std::setprecision(2) << std::abs(yaw_diff * 180.0 / M_PI) << "\u00b0 ("
            << std::setprecision(4) << yaw_diff << " rad)"
            << std::endl;

        log_file_stream_
            << "  Current: (" << std::setprecision(3)
            << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << ")"
            << " yaw=" << std::setprecision(2) << (current_yaw * 180.0 / M_PI) << "\u00b0"
            << " | Target: (" << std::setprecision(3)
            << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ")"
            << " yaw=" << std::setprecision(2) << (target_yaw * 180.0 / M_PI) << "\u00b0"
            << std::endl;
    }

    // 6. Write to CSV file
    if (csv_stream.is_open()) {
        csv_stream << std::fixed << std::setprecision(4)
            << curr_nav_goal_ << ","
            << dx << "," << dy << "," << dz << "," << euc_dist << ","
            << (yaw_diff * 180.0 / M_PI) << "," << yaw_diff << ","
            << current_pose.position.x << "," << current_pose.position.y << "," << current_pose.position.z << ","
            << (current_yaw * 180.0 / M_PI) << ","
            << target_pose.position.x << "," << target_pose.position.y << "," << target_pose.position.z << ","
            << (target_yaw * 180.0 / M_PI) << ","
            << result << std::endl;
    }

    // 7. Log displacement to terminal
    RCLCPP_INFO(this->get_logger(),
        "[%s] Viewpose ID: %d | dx=%.3fm dy=%.3fm dz=%.3fm euclidean=%.3fm | yaw_diff=%.2f\u00b0 (%.4f rad) | Result: %s",
        phase_name.c_str(), curr_nav_goal_, dx, dy, dz, euc_dist,
        std::abs(yaw_diff * 180.0 / M_PI), yaw_diff, result.c_str());
}



std::string InspectionPlannerNode::log_prefix(const std::string& level){
    auto now = this->get_clock()->now();
    int64_t ns = now.nanoseconds();
    int64_t secs = ns / 1'000'000'000;
    int64_t nsecs = ns % 1'000'000'000;
    std::ostringstream oss;
    oss << "[" << secs << "." << std::setfill('0') << std::setw(9) << nsecs << "] [" << level << "] inspection_planner: ";
    return oss.str();
}

void InspectionPlannerNode::log_msg(const std::string& msg){
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    if (!log_file_stream_.is_open()){
        RCLCPP_WARN(this->get_logger(), "Log file is not opened!");
        return;
    }
    log_file_stream_ << log_prefix("INFO") << msg << std::endl;
}

void InspectionPlannerNode::log_error_msg(const std::string& msg){
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    if (!log_file_stream_.is_open()){
        RCLCPP_WARN(this->get_logger(), "Log file is not opened!");
        return;
    }
    log_file_stream_ << log_prefix("ERROR") << msg << std::endl;
}

void InspectionPlannerNode::log_warning_msg(const std::string& msg){
    RCLCPP_WARN(this->get_logger(), msg.c_str());
    if (!log_file_stream_.is_open()){
        RCLCPP_WARN(this->get_logger(), "Log file is not opened!");
        return;
    }
    log_file_stream_ << log_prefix("WARN") << msg << std::endl;
}



/* Visualization Functions */
void InspectionPlannerNode::update_visualization(){
    visualization_msgs::msg::MarkerArray marker_array;

    // Helper lambda to create a sphere marker for a single waypoint
    auto create_marker = [&](uint32_t id, const geometry_msgs::msg::Pose& pose,
                             const std::string& ns, 
                             float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = pose.position.x;
        marker.pose.position.y = pose.position.y;
        marker.pose.position.z = pose.position.z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration(0, 0);  // persistent

        marker_array.markers.push_back(marker);
    };

    // Unvisited waypoints — blue
    for (const auto& [id, pose] : unvisited_poses_){
        create_marker(id, pose, "inspection_pose", 0.0f, 0.0f, 1.0f);
    }

    // Visited waypoints — green
    for (const auto& [id, pose] : visited_poses_){
        create_marker(id, pose, "inspection_pose", 0.0f, 1.0f, 0.0f);
    }

    // Failed waypoints — red
    for (const auto& [id, pose] : failed_poses_){
        create_marker(id, pose, "inspection_pose", 1.0f, 0.0f, 0.0f);
    }

    // Helper to look up a pose from any of the three maps
    auto find_pose = [&](uint32_t id) -> const geometry_msgs::msg::Pose* {
        auto it = unvisited_poses_.find(id);
        if (it != unvisited_poses_.end()) return &it->second;
        it = visited_poses_.find(id);
        if (it != visited_poses_.end()) return &it->second;
        it = failed_poses_.find(id);
        if (it != failed_poses_.end()) return &it->second;
        return nullptr;
    };

    // Draw arrows between consecutive TSP waypoints showing visit order
    if (tsp_result_.size() > 1){
        for (size_t i = 0; i < tsp_result_.size() - 1; ++i){
            const auto* from_pose = find_pose(tsp_result_[i]);
            const auto* to_pose   = find_pose(tsp_result_[i + 1]);
            if (!from_pose || !to_pose) continue;

            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->get_clock()->now();
            arrow.ns = "tsp_edges";
            arrow.id = 10000 + i;  // offset to avoid collision with waypoint IDs
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            arrow.points.emplace_back();
            arrow.points.back().x = from_pose->position.x;
            arrow.points.back().y = from_pose->position.y;
            arrow.points.back().z = from_pose->position.z;

            arrow.points.emplace_back();
            arrow.points.back().x = to_pose->position.x;
            arrow.points.back().y = to_pose->position.y;
            arrow.points.back().z = to_pose->position.z;

            arrow.scale.x = 0.1;  // shaft radius
            arrow.scale.y = 0.2;  // head radius

            arrow.color.r = 0.8f;
            arrow.color.g = 0.0f;
            arrow.color.b = 0.3f;
            arrow.color.a = 0.6f;

            arrow.lifetime = rclcpp::Duration(0, 0);

            marker_array.markers.push_back(arrow);
        }
    }

    waypoint_viz_pub_->publish(marker_array);
}



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InspectionPlannerNode>();
    g_inspection_node = node.get();

    // Register signal handler to write inspection summary on shutdown
    signal(SIGINT, shutdown_handler);

    rclcpp::spin(node);
    if (rclcpp::ok()){
        rclcpp::shutdown();
    }
    return 0;
}