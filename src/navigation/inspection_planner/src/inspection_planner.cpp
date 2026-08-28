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

    auto redo_tsp_period_desc = rcl_interfaces::msg::ParameterDescriptor();
    redo_tsp_period_desc.description = "Recalculates TSP result on this period while navigating.";
    this->declare_parameter<double>("redo_tsp_period", 5.0, redo_tsp_period_desc);

    auto success_count_for_retry_threshold_desc = rcl_interfaces::msg::ParameterDescriptor();
    success_count_for_retry_threshold_desc.description = "Adds failed poses to tsp calculation if threshold amount of viewposes successfully reached.";
    this->declare_parameter<int>("success_count_for_retry_threshold", 5, success_count_for_retry_threshold_desc);

    auto planner_mode_desc = rcl_interfaces::msg::ParameterDescriptor();
    planner_mode_desc.description = "Set inspection planning mode. Supported values: ascending, tsp-once, tsp, rolling-tsp";
    this->declare_parameter<std::string>("planner_mode", "tsp", planner_mode_desc);
    
    auto do_retry_desc = rcl_interfaces::msg::ParameterDescriptor();
    do_retry_desc.description = "Whether to retry failed waypoints or not.";
    this->declare_parameter<bool>("do_retry", true, do_retry_desc);
    
    auto rolling_window_size_desc = rcl_interfaces::msg::ParameterDescriptor();
    rolling_window_size_desc.description = "Size of rolling window for rolling-tsp mode.";
    this->declare_parameter<int>("rolling_window_size", 5, rolling_window_size_desc);

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
    auto planner_mode_str = this->get_parameter("planner_mode").as_string();

    auto mode_itr = PLANNER_MODE_MATCH.find(planner_mode_str);
    if (mode_itr == PLANNER_MODE_MATCH.end()){
        RCLCPP_ERROR(this->get_logger(), "Unsupported planner mode %s, using default value: tsp", planner_mode_str.c_str());
        planner_mode_ = PlannerMode::Tsp;
    } else {
        planner_mode_ = mode_itr->second;
    }

    do_retry_ = this->get_parameter("do_retry").as_bool();
    rolling_window_size_ = this->get_parameter("rolling_window_size").as_int();
    if (rolling_window_size_ <= 0){
        RCLCPP_ERROR(this->get_logger(), "Rolling window size cannot be smaller than 1, defaulting to 5...");
        rolling_window_size_ = 5;
    }



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
    msg_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/inspection_log/msg", 
        10
    );

    log_msg_pub_ = this->create_publisher<inspection_planner_interfaces::msg::LogMsg>(
        "/inspection_log/log_msg", 
        10
    ); 

    viewposes_debug_pub_ = this->create_publisher<inspection_planner_interfaces::msg::ViewPosesDebug>(
        "/inspection_log/poses_debug", 
        5
    ); 

    num_poses_received_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
        "/inspection_log/num_poses_received", 
        5
    ), 

    num_poses_added_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
        "/inspection_log/num_poses_added", 
        5
    );

    log_displacement_client_ = this->create_client<inspection_planner_interfaces::srv::LogDisplacement>(
        "/inspection_log/log_displacement"
    );

    latest_pose_pub_ = this->create_publisher<inspection_planner_interfaces::msg::ViewPose>(
       "/inspection_debug/latest_pose", 
       5 
    );
}





void InspectionPlannerNode::inspection_poses_callback(const ViewPoses::SharedPtr msg){
    std_msgs::msg::UInt32 num_poses_received_msg;
    num_poses_received_msg.data = static_cast<uint32_t>(msg->poses.size());
    this->num_poses_received_pub_->publish(num_poses_received_msg);

    this->log_msg("Received new inspection poses. Number of poses: " + std::to_string(msg->poses.size()));
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> new_pose_map;
    build_poses_map(msg, new_pose_map);

    merge_similar_poses(new_pose_map);

    filter_repeated_poses(new_pose_map, unvisited_poses_);
    filter_repeated_poses(new_pose_map, visited_poses_);
    this->log_msg("Filtered received poses. Number of poses remaining: " + std::to_string(new_pose_map.size()));
    if (new_pose_map.size() == 0) return;
    
    merge_poses_maps(unvisited_poses_, new_pose_map);

    std_msgs::msg::UInt32 num_poses_added_msg;
    num_poses_added_msg.data = static_cast<uint32_t>(new_pose_map.size());
    this->num_poses_added_pub_->publish(num_poses_added_msg);

    this->is_retrying_ = false; //TODO: Add inspection initailization method
    
    get_new_waypoints_order(true);
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
        this->poses_order_ = response->ordered_waypoint_ids;
        this->log_msg("TSP solved. Ordered waypoints: " + std::to_string(this->poses_order_.size()));
        if (!response->message.empty()) {
            this->log_msg("TSP solver message: " + response->message);
        }
    } else {
        this->log_msg("TSP solver failed: " + response->message);
        this->poses_order_.clear();
    }
    waiting_new_tsp_result_ = false;

    if (planner_mode_ == PlannerMode::Tsp || planner_mode_ == PlannerMode::TspOnce){
        // For every viewpose that is not in tsp result, put them in the 'failed' list
        std::string failed_poses_log;
        for (auto itr = unvisited_poses_.begin(); itr != unvisited_poses_.end();){
            auto pose = *itr;
            bool found = false;
            for (auto valid_pose_id : poses_order_){
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
            this->log_msg("The following waypoints failed: " + failed_poses_log);
        }
    }
    else if (planner_mode_ == PlannerMode::RollingTsp){
        // Put every viewpose inside rolling window that isnt in tsp result into failed rolling window and failed poses
        std::string failed_poses_log;
        for (auto itr = rolling_window_.begin(); itr != rolling_window_.end();){
            bool found = false;
            uint32_t id = *itr;
            for (auto valid_pos_id : poses_order_){
                if (valid_pos_id == id){
                    found = true;
                    break;
                }
            }
            if (!found){
                failed_rolling_window_.push_back(id);
                itr = rolling_window_.erase(itr);
                auto unvisited_node = unvisited_poses_.extract(id);
                if (unvisited_node){
                    failed_poses_.insert(std::move(unvisited_node));
                }
                failed_poses_log += std::to_string(id) + " ";
            } else {
                itr++;
            }
        }
    }

    update_visualization();

    // if (poses_order_.empty()){
    //     inspection_active_ = false;
    //     cancel_current_goal();
    //     this->log_msg("Inspection complete."); 
    //     return;
    // }

    curr_nav_tsp_index_ = 0;
    inspection_active_ = true;
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
    inspection_active_ = false;
    unvisited_poses_.clear();
    visited_poses_.clear();
    failed_poses_.clear();
    poses_order_.clear();

    cancel_current_goal();
}


bool InspectionPlannerNode::cancel_current_goal(){
    if (nav_goal_handle_){
        auto status = nav_goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED || 
            nav_goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
            RCLCPP_INFO(this->get_logger(), "Cancelling navigation goal pose...");
            nav_action_client_->async_cancel_goal(nav_goal_handle_);
            return true;
        }
    }
    return false;
}


bool InspectionPlannerNode::pub_next_nav_goal(){
    int next_pose_id = -1;
    // Search for valid nav goal pose id
    while (true){
        if (curr_nav_tsp_index_ >= poses_order_.size()){
            this->log_msg("No more ordered waypoints, requesting for new waypoints order.");
            this->get_new_waypoints_order();
            return true;
        }
        auto next_tsp_id = poses_order_[curr_nav_tsp_index_];
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
    if (cancel_current_goal()){
        // Has active goal
        curr_nav_tsp_index_ = 0;
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
    
    auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
    std::bind(&InspectionPlannerNode::nav_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
    std::bind(&InspectionPlannerNode::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
    std::bind(&InspectionPlannerNode::nav_result_callback, this, std::placeholders::_1, next_pose_id);
    nav_action_client_->async_send_goal(goal, send_goal_options);
    
    if (planner_mode_ == PlannerMode::Tsp){
        // Redo tsp timer
        if (this->redo_tsp_timer_ && !this->redo_tsp_timer_->is_canceled()){
            this->redo_tsp_timer_->cancel();
        }
        this->redo_tsp_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->redo_tsp_period_), 
            [this](){
                if (allow_pub_new_waypoint_ && inspection_active_ && !waiting_new_tsp_result_){
                    get_new_distance_matrix(unvisited_poses_);
                } else {
                    this->redo_tsp_timer_->cancel();
                }
            }
        );
    }
    
    // Debug pose publishing
    ViewPose debug_pose_msg;
    debug_pose_msg.pose = goal_pose;
    debug_pose_msg.id = curr_nav_goal_;
    this->latest_pose_pub_->publish(debug_pose_msg);

    return true;
}


void InspectionPlannerNode::nav_goal_response_callback(const NavGoalHandle::SharedPtr& goal_handle){
    if (!goal_handle) {
        this->log_error_msg("Goal was rejected by the action server!");
        if (inspection_active_) pub_next_nav_goal();
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
    
    ViewPose target_pose;
    target_pose.pose = unvisited_poses_.at(curr_nav_goal_);
    target_pose.id = curr_nav_goal_;

    if (feedback->state == ROTATING) {
        log_displacement(target_pose, "Unknown", 1);
    } else if (feedback->state == ADJUSTING) {
        log_displacement(target_pose, "Unknown", 0);
    }
}

void InspectionPlannerNode::nav_result_callback(const NavGoalHandle::WrappedResult& result, int goal_id){
    if (!inspection_active_ || !nav_goal_handle_) return;
    auto node = unvisited_poses_.extract(goal_id);

    nav_goal_handle_ = nullptr;
    if (node) {
        std::string nav_result = (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? "Success" : "Failed";
        ViewPose target_pose;
        target_pose.pose = node.mapped();
        target_pose.id = goal_id;
        this->log_displacement(target_pose, nav_result, 2);
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
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED){
        this->log_msg("Goal was canceled! ID: " + std::to_string(goal_id));
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            this->failed_poses_.insert(std::move(node));
        }
    }
    else{
        this->log_error_msg("Unknown result code. ID: " + std::to_string(goal_id));
        if (!node){
            this->log_error_msg("Navigated to unknown/visited inspection pose. ID: " + std::to_string(goal_id));
        } else {
            this->failed_poses_.insert(std::move(node));
        }
    }
    update_visualization();
    
    if (inspection_active_){
        if (planner_mode_ == PlannerMode::Tsp || planner_mode_ == PlannerMode::RollingTsp){
            this->get_new_waypoints_order();
        } else {
            pub_next_nav_goal();
        }
    }
}


void InspectionPlannerNode::get_new_waypoints_order(bool init){
    if (planner_mode_ == PlannerMode::Tsp){
        // TODO: Do finish condition check and moving poses HERE
        if (unvisited_poses_.size() == 0){
            if (failed_poses_.size() == 0){
                inspection_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Inspection complete. All inspection poses visited."); // TODO: placeholder next waypoint?
                return;
            }
            move_failed_to_unvisited();
        }

        if (this->success_count_for_retry_ >= this->success_count_for_retry_threshold_){
            move_failed_to_unvisited();
            this->success_count_for_retry_ = 0;
        }
        this->get_new_distance_matrix(unvisited_poses_);
        return;
    }
    else if (planner_mode_ == PlannerMode::TspOnce){
        if (init){
            this->get_new_distance_matrix(unvisited_poses_);
            return;
        }
        if (do_retry_ && !is_retrying_){
            this->log_msg("Retrying failed poses...");
            this->move_failed_to_unvisited();
            is_retrying_ = true;
            this->get_new_distance_matrix(unvisited_poses_);
            return;
        }
        this->inspection_active_ = false;
        this->log_msg("Inspection complete.");
        return;
    }

    else if (planner_mode_ == PlannerMode::Ascending){
        this->poses_order_.clear();
        this->curr_nav_tsp_index_ = 0;
        // Ascending mode terminating condition
        if (unvisited_poses_.size() == 0){
            // End inspection if already retried failed poses
            if (this->do_retry_ && !this->is_retrying_ && failed_poses_.size() != 0){
                // Retry failed poses
                this->is_retrying_ = true;
                this->move_failed_to_unvisited();
                this->log_msg("Retrying failed poses...");
            } else {
                this->inspection_active_ = false;
                this->log_msg("Inspection complete.");
                return;
            }
        }
    
        // No TSP, order unvisited waypoints by waypoint ids
        for (auto [id, pose] : unvisited_poses_){
            this->poses_order_.push_back(id);
        }
        std::sort(this->poses_order_.begin(), this->poses_order_.end());
        RCLCPP_INFO(this->get_logger(), "Not using TSP solver, number of unvisited viewposes: %zu", this->poses_order_.size());
    
        // Build ordered waypoint message
        std::string msg;
        for (auto id : this->poses_order_){
            msg += std::to_string(id) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Ordered waypoints: %s", msg.c_str());
        update_visualization();
        inspection_active_ = true;
        pub_next_nav_goal();
        return;
    }

    else if (planner_mode_ == PlannerMode::RollingTsp){
        // Remove failed/success rolling window ids and add failed to temp failed window
        std::vector<uint32_t> temp_failed_window; // To store latest failed waypoints, avoid retrying same waypoint on intentional abort
        for (auto itr = rolling_window_.begin(); itr != rolling_window_.end();){
            auto id = *itr;
            if (visited_poses_.find(id) != visited_poses_.end()){
                itr = rolling_window_.erase(itr);
            } else if (failed_poses_.find(id) != failed_poses_.end()){
                itr = rolling_window_.erase(itr);
                temp_failed_window.push_back(id);
            } else {
                itr++;
            }
        }
        
        // If empty rolling window, get new rolling window
        if (rolling_window_.size() <= 0){
            if (unvisited_poses_.size() == 0){
                inspection_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Inspection complete."); // TODO: placeholder next waypoint?
                return;
            }

            std::vector<uint32_t> ordered_ids;
            for (auto [id, pose] : unvisited_poses_){
                ordered_ids.push_back(id);
            }
            std::sort(ordered_ids.begin(), ordered_ids.end());
            if (static_cast<int>(ordered_ids.size()) <= rolling_window_size_){
                rolling_window_ = ordered_ids;
            } else {
                rolling_window_.insert(rolling_window_.end(), ordered_ids.begin(), ordered_ids.begin()+rolling_window_size_);
            }
            failed_rolling_window_.clear();
            this->get_new_distance_matrix(rolling_window_);
            return;
        }

        if (do_retry_){
            // Move the rolling window failed poses back into unvisited poses
            move_failed_to_unvisited(failed_rolling_window_);
            // Join failed poses back into rolling window
            rolling_window_.reserve(failed_rolling_window_.size());
            rolling_window_.insert(rolling_window_.end(), failed_rolling_window_.begin(), failed_rolling_window_.end());
            failed_rolling_window_.clear();
        }
        failed_rolling_window_.insert(failed_rolling_window_.end(), temp_failed_window.begin(), temp_failed_window.end());

        this->get_new_distance_matrix(rolling_window_);        
        return;
    }

}


void InspectionPlannerNode::get_new_distance_matrix(const std::unordered_map<uint32_t, geometry_msgs::msg::Pose> poses){
    Waypoints waypoints_msg;
    build_waypoints_msg(poses, waypoints_msg);
    tsp_waypoints_pub_->publish(waypoints_msg);
    waiting_new_tsp_result_ = true;
    return;
}


void InspectionPlannerNode::get_new_distance_matrix(const std::vector<uint32_t> pose_ids){
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> poses;
    for (auto id : pose_ids){
        auto itr = unvisited_poses_.find(id);
        if (itr != unvisited_poses_.end()){
            poses[id] = itr->second;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Cannot find id %d in unvisited poses, skipping id...", id);
        }
    }
    Waypoints waypoints_msg;
    build_waypoints_msg(poses, waypoints_msg);
    tsp_waypoints_pub_->publish(waypoints_msg);
    waiting_new_tsp_result_ = true;
    return;
}

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

void InspectionPlannerNode::move_failed_to_unvisited(std::vector<uint32_t> ids){
    for (uint32_t id : ids){
        auto node = failed_poses_.extract(id);
        if (node){
            unvisited_poses_.insert(std::move(node));
        }
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


void InspectionPlannerNode::log_msg(const std::string &msg)
{
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    inspection_planner_interfaces::msg::LogMsg log_msg;
    log_msg.msg = msg;
    log_msg.header.stamp = this->get_clock()->now();
    log_msg.msg_type = log_msg.MSG_TYPE_INFO;
    this->log_msg_pub_->publish(log_msg);
}

void InspectionPlannerNode::log_error_msg(const std::string &msg)
{
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    inspection_planner_interfaces::msg::LogMsg log_msg;
    log_msg.msg = msg;
    log_msg.header.stamp = this->get_clock()->now();
    log_msg.msg_type = log_msg.MSG_TYPE_ERROR;
    this->log_msg_pub_->publish(log_msg);
}

void InspectionPlannerNode::update_visualization()
{
    ViewPoses unvisited_poses_msg, visited_poses_msg, failed_poses_msg;
    this->build_poses_msg(this->unvisited_poses_, unvisited_poses_msg);
    this->build_poses_msg(this->visited_poses_, visited_poses_msg);
    this->build_poses_msg(this->failed_poses_, failed_poses_msg);

    inspection_planner_interfaces::msg::ViewPosesDebug debug_msg;
    debug_msg.unvisited = unvisited_poses_msg;
    debug_msg.visited = visited_poses_msg;
    debug_msg.failed = failed_poses_msg;
    debug_msg.ordered_ids = poses_order_;

    viewposes_debug_pub_->publish(debug_msg);
}

void InspectionPlannerNode::log_displacement(const ViewPose& target_pose, const std::string& result, uint8_t phase)
{
    auto req = std::make_shared<inspection_planner_interfaces::srv::LogDisplacement::Request>();
    req->pose = target_pose;
    req->phase = phase;
    req->result = result;
    this->log_displacement_client_->async_send_request(req);
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