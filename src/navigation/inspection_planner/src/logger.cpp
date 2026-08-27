#include <inspection_planner/logger.hpp>

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

LoggerNode::LoggerNode()
: Node("inspection_logger")
{
    params_init();
    files_init();
    communications_init();

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LoggerNode::params_init(){
    this->declare_parameter<double>("edge_gradient_max_len", 10.0);
    this->declare_parameter<double>("edge_gradient_min_len", 1.0);

    this->edge_gradient_max_len_ = this->get_parameter("edge_gradient_max_len").as_double();
    this->edge_gradient_min_len_ = this->get_parameter("edge_gradient_min_len").as_double();

    if (this->edge_gradient_max_len_ <= this->edge_gradient_min_len_){
        RCLCPP_ERROR(this->get_logger(), "Gradient max length must be bigger than min length! Using default values...");
        this->edge_gradient_max_len_ = 10.0; 
        this->edge_gradient_min_len_ = 1.0;
    }
}

void LoggerNode::files_init(){
    std::string datetime = get_datetime_string();
    std::string log_file_path = root_dir_ + "logs/" + datetime + ".log";
    log_file_stream_.open(log_file_path);
    RCLCPP_INFO(this->get_logger(), "Saving logs to %s", log_file_path.c_str());

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


void LoggerNode::communications_init(){
    // Subsriptions
    this->msg_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/inspection_log/msg", 
        10, 
        std::bind(&LoggerNode::msg_callback, this, std::placeholders::_1)
    );

    this->log_msg_sub_ = this->create_subscription<inspection_planner_interfaces::msg::LogMsg>(
        "/inspection_log/log_msg", 
        10, 
        std::bind(&LoggerNode::log_msg_callback, this, std::placeholders::_1)
    );

    this->viewposes_debug_sub_ = this->create_subscription<inspection_planner_interfaces::msg::ViewPosesDebug>(
        "/inspection_log/poses_debug", 
        5, 
        std::bind(&LoggerNode::poses_debug_callback, this, std::placeholders::_1)
    );

    this->received_poses_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
        "/inspection_log/num_poses_received", 
        5, 
        std::bind(&LoggerNode::received_poses_callback, this, std::placeholders::_1)
    );

    this->added_poses_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
        "/inspection_log/num_poses_added", 
        5, 
        std::bind(&LoggerNode::added_poses_callback, this, std::placeholders::_1)
    );

    // Publishers
    this->viewpoints_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/inspection_log/viewpoints_viz", 
        5
    ); 

    this->viewposes_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/inspection_log/viewposes_viz", 
        5
    ); 

    // Services
    this->log_displacement_srv_ = this->create_service<inspection_planner_interfaces::srv::LogDisplacement>(
        "/inspection_log/log_displacement", 
        std::bind(&LoggerNode::log_displacement_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/inspection_log/reset", 
        std::bind(&LoggerNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

}


// Callbacks
void LoggerNode::msg_callback(const std_msgs::msg::String::SharedPtr msg){
    log_file_stream_ << msg->data << std::endl;
}

void LoggerNode::log_msg_callback(const inspection_planner_interfaces::msg::LogMsg::SharedPtr msg){
    std::string prefix;
    if (msg->header.stamp == rclcpp::Time()){
        prefix = this->get_log_prefix(msg->msg_type);
    } else {
        prefix = this->get_log_prefix(msg->msg_type);
    }

    log_file_stream_ << prefix << msg->msg << std::endl;
    if (!msg->to_terminal){
        return;
    }
    if (msg->msg_type == msg->MSG_TYPE_INFO){
        RCLCPP_INFO(this->get_logger(), ("Received message: " + msg->msg).c_str());
    } else if (msg->msg_type == msg->MSG_TYPE_WARN){
        RCLCPP_WARN(this->get_logger(), ("Received message: " + msg->msg).c_str());
    } else if (msg->msg_type == msg->MSG_TYPE_ERROR){
        RCLCPP_ERROR(this->get_logger(), ("Received message: " + msg->msg).c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), ("Received message of unknown type: " + msg->msg).c_str());
    }
}


void LoggerNode::poses_debug_callback(const inspection_planner_interfaces::msg::ViewPosesDebug::SharedPtr msg){
    

    build_poses_map(msg->unvisited, unvisited_poses_);
    build_poses_map(msg->visited, visited_poses_);
    build_poses_map(msg->failed, failed_poses_);
    ordered_ids_ = msg->ordered_ids;

    pub_marker_array();
    pub_pose_array();
}

void LoggerNode::pub_marker_array()
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Unvisited waypoints — blue
    for (const auto& [id, pose] : unvisited_poses_){
        this->add_waypoint_marker(marker_array, id, pose, 0.0f, 0.0f, 1.0f);
    }

    // Visited waypoints — green
    for (const auto& [id, pose] : visited_poses_){
        this->add_waypoint_marker(marker_array, id, pose, 0.0f, 1.0f, 0.0f);
    }

    // Failed waypoints — red
    for (const auto& [id, pose] : failed_poses_){
        this->add_waypoint_marker(marker_array, id, pose, 1.0f, 0.0f, 0.0f);
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
    if (ordered_ids_.size() > 1){
        for (size_t i = 0; i < ordered_ids_.size() - 1; ++i){
            const auto* from_pose = find_pose(ordered_ids_[i]);
            const auto* to_pose   = find_pose(ordered_ids_[i + 1]);
            if (!from_pose || !to_pose) continue;

            add_edge_marker(marker_array, i, from_pose, to_pose);
            add_gradient_edge_marker(marker_array, i, from_pose, to_pose);
        }
    }

    viewpoints_viz_pub_->publish(marker_array);
}



void LoggerNode::pub_pose_array()
{
    geometry_msgs::msg::PoseArray pose_array;

    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->get_clock()->now();
    for (auto [id, pose] : unvisited_poses_){
        pose_array.poses.push_back(pose);
    }
    for (auto [id, pose] : visited_poses_){
        pose_array.poses.push_back(pose);
    }
    for (auto [id, pose] : failed_poses_){
        pose_array.poses.push_back(pose);
    }

    this->viewposes_viz_pub_->publish(pose_array);
}




void LoggerNode::received_poses_callback(const std_msgs::msg::UInt32::SharedPtr msg){
    this->total_poses_received_ += msg->data;
}

void LoggerNode::added_poses_callback(const std_msgs::msg::UInt32::SharedPtr msg){
    this->total_poses_added_ += msg->data;
}


void LoggerNode::log_displacement_callback(
    const inspection_planner_interfaces::srv::LogDisplacement::Request::SharedPtr req, 
    inspection_planner_interfaces::srv::LogDisplacement::Response::SharedPtr res
)
{
    // 0. Get parameters
    auto target_pose = req->pose.pose;
    auto target_pose_id = req->pose.id;
    auto phase = req->phase;
    auto result = req->result;
    
    // 1. Get current robot pose from TF
    geometry_msgs::msg::Pose current_pose;
    try{
        auto transform = tf_buffer_->lookupTransform("map", "robot_footprint", tf2::TimePointZero);
        current_pose.position.x = transform.transform.translation.x;
        current_pose.position.y = transform.transform.translation.y;
        current_pose.position.z = transform.transform.translation.z;
        current_pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException &ex) {
        std::string msg = "Failed to get tf transform from map to robot_footprint: " + std::string(ex.what());
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        res->success = false;
        res->message = msg;
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
    std::ofstream& csv_stream = (phase == req->PRE_ROTATION_PHASE)   ? csv_pre_rotation_stream_
                                  : (phase == req->PRE_ADJUSTMENT_PHASE) ? csv_pre_adjustment_stream_
                                                                                  : csv_post_rotation_stream_;

    if (phase == req->PRE_ROTATION_PHASE)   phase_name = "Pre-Rotation";
    else if (phase == req->PRE_ADJUSTMENT_PHASE) phase_name = "Pre-Adjustment";
    else phase_name = "Post-Rotation";

    // 5. Write to .log file
    if (log_file_stream_.is_open()) {
        log_file_stream_
            << "Viewpose ID: " << target_pose_id
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
            << target_pose_id << ","
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
        phase_name.c_str(), target_pose_id, dx, dy, dz, euc_dist,
        std::abs(yaw_diff * 180.0 / M_PI), yaw_diff, result.c_str());
    
    res->success = true;
    return;
}

void LoggerNode::reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req, 
    std_srvs::srv::Trigger::Response::SharedPtr res
)
{
    (void)req, (void)res;
    total_poses_received_ = 0;
    total_poses_added_ = 0;
    unvisited_poses_.clear(); 
    visited_poses_.clear();
    failed_poses_.clear();
}



// Helper functions
std::string LoggerNode::get_log_prefix(const uint8_t& level, const rclcpp::Time& time){
    rclcpp::Time log_time = time;
    if (log_time == rclcpp::Time()){
        log_time = this->get_clock()->now();
    }
    std::string level_str = LOG_LEVEL_MAP.at(level);
    int64_t ns = log_time.nanoseconds();
    int64_t secs = ns / 1'000'000'000;
    int64_t nsecs = ns % 1'000'000'000;
    std::ostringstream oss;
    oss << "[" << secs << "." << std::setfill('0') << std::setw(9) << nsecs << "] [" << level_str << "] inspection_planner: ";
    return oss.str();
}


void LoggerNode::add_waypoint_marker(
    visualization_msgs::msg::MarkerArray& marker_array, 
    const uint32_t id, 
    const geometry_msgs::msg::Pose &pose, float r, float g, float b)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "waypoint";
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

    // Create text marker
    visualization_msgs::msg::Marker text_marker;

    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = "waypoint_text";
    text_marker.id = id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = pose.position.x;
    text_marker.pose.position.y = pose.position.y;
    text_marker.pose.position.z = pose.position.z + 0.3;

    // text_marker.scale.x = 0.5;
    // text_marker.scale.y = 0.5;
    text_marker.scale.z = 0.2;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(id);
    marker_array.markers.push_back(text_marker);
}

void LoggerNode::add_edge_marker(visualization_msgs::msg::MarkerArray &marker_array, const uint32_t id, const geometry_msgs::msg::Pose *from_pose, const geometry_msgs::msg::Pose *to_pose)
{
    auto edge = this->create_simple_edge(id, "edge", from_pose, to_pose);
    edge.scale.x = 0.1;  // shaft radius
    edge.scale.y = 0.2;  // head radius

    edge.color.r = 0.8f;
    edge.color.g = 0.0f;
    edge.color.b = 0.3f;
    edge.color.a = 0.8f;

    edge.lifetime = rclcpp::Duration(0, 0);
    marker_array.markers.push_back(edge);
}


void LoggerNode::add_gradient_edge_marker(visualization_msgs::msg::MarkerArray &marker_array, const uint32_t id, const geometry_msgs::msg::Pose *from_pose, const geometry_msgs::msg::Pose *to_pose)
{
    auto edge = this->create_simple_edge(id, "gradient_edge", from_pose, to_pose);
    auto color = get_edge_gradient_color(calculate_distance(from_pose->position, to_pose->position));
    edge.color = color;

    edge.scale.x = 0.1;  // shaft radius
    edge.scale.y = 0.2;  // head radius

    edge.lifetime = rclcpp::Duration(0, 0);
    marker_array.markers.push_back(edge);
}

std_msgs::msg::ColorRGBA LoggerNode::get_edge_gradient_color(double length)
{
    std_msgs::msg::ColorRGBA color;
    double clamped_len = std::clamp(length, this->edge_gradient_min_len_, this->edge_gradient_max_len_);
    double t = static_cast<double>((clamped_len - edge_gradient_min_len_) / (edge_gradient_max_len_ - edge_gradient_min_len_));

    // Direct Blue (0,0,1) -> Red (1,0,0)
    color.r = t;
    color.g = 0.0f;
    color.b = 1.0f - t;
    color.a = 1.0f;
    return color;
}


visualization_msgs::msg::Marker LoggerNode::create_simple_edge(const int32_t id, const std::string &ns, const geometry_msgs::msg::Pose *from_pose, const geometry_msgs::msg::Pose *to_pose)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.emplace_back();
    marker.points.back().x = from_pose->position.x;
    marker.points.back().y = from_pose->position.y;
    marker.points.back().z = from_pose->position.z;

    marker.points.emplace_back();
    marker.points.back().x = to_pose->position.x;
    marker.points.back().y = to_pose->position.y;
    marker.points.back().z = to_pose->position.z;

    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
}


double LoggerNode::calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}


void LoggerNode::build_poses_map(
    const ViewPoses::SharedPtr msg, 
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
)
{
    map.clear();
    for (auto pose : msg->poses){
        map[pose.id] = pose.pose;
    }
}

void LoggerNode::build_poses_map(
    const ViewPoses msg, 
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
)
{
    map.clear();
    for (auto pose : msg.poses){
        map[pose.id] = pose.pose;
    }
}


void LoggerNode::build_poses_msg(
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





void LoggerNode::write_inspection_summary(){
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





static LoggerNode* logger_node = nullptr;
static void shutdown_handler(int) {
    if (logger_node) {
        logger_node->write_inspection_summary();
    }
}
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggerNode>();
    logger_node = node.get();

    // Register signal handler to write inspection summary on shutdown
    signal(SIGINT, shutdown_handler);

    rclcpp::spin(node);
    if (rclcpp::ok()){
        rclcpp::shutdown();
    }
    return 0;
}