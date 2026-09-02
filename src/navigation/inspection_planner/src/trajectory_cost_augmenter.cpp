#include <inspection_planner/trajectory_cost_augmenter.hpp>

TrajectoryCostAugmenter::TrajectoryCostAugmenter()
: Node("trajectory_cost_augmenter")
{
    // Declare Parameters
    this->declare_parameter<double>("fixed_entry_cost", 1.0);
    this->declare_parameter<double>("cost_per_degree", 0.01111111111);

    fixed_entry_cost_ = this->get_parameter("fixed_entry_cost").as_double();
    cost_per_radian_ = this->get_parameter("cost_per_degree").as_double() / 3.14159265358979323846 * 180.0;

    trajectory_cost_srv_ = this->create_service<InjectTrajectoryCost>(
        "/inject_trajectory_cost", 
        std::bind(&TrajectoryCostAugmenter::trajectory_cost_service_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
}


void TrajectoryCostAugmenter::trajectory_cost_service_callback(const InjectTrajectoryCost::Request::SharedPtr req, InjectTrajectoryCost::Response::SharedPtr res)
{
    RCLCPP_INFO(this->get_logger(), "Received distance matrix, injecting trajectory cost...");
    auto matrix = req->matrix;
    auto poses = req->poses;

    std::unordered_map<uint32_t, geometry_msgs::msg::Pose> poses_map;
    build_poses_map(poses, poses_map);

    std::string log_msg = "";
    for (auto& entry : matrix.entries){
        auto source_pose_itr = poses_map.find(entry.source_id);
        auto target_pose_itr = poses_map.find(entry.target_id);
        if (source_pose_itr == poses_map.end()){
            RCLCPP_WARN(this->get_logger(), "Cannot find pose with id %d! Skipping entry from %d to %d", entry.source_id, entry.source_id, entry.target_id);
            log_msg += "Cannot find pose with id " + std::to_string(entry.source_id) + "! Skipping entry from " + std::to_string(entry.source_id) + " to " + std::to_string(entry.target_id) + "\n";
            continue;
        } else if (target_pose_itr == poses_map.end()){
            RCLCPP_WARN(this->get_logger(), "Cannot find pose with id %d! Skipping entry from %d to %d", entry.target_id, entry.source_id, entry.target_id);
            log_msg += "Cannot find pose with id " + std::to_string(entry.target_id) + "! Skipping entry from " + std::to_string(entry.source_id) + " to " + std::to_string(entry.target_id) + "\n";
            continue;
        }
        entry.distance += calculate_cost(source_pose_itr->second, target_pose_itr->second);
    }

    res->matrix = matrix;
    res->msg = log_msg;
    RCLCPP_INFO(this->get_logger(), "Finished injecting trajectory cost.");
}


double TrajectoryCostAugmenter::calculate_cost(const geometry_msgs::msg::Pose& source, const geometry_msgs::msg::Pose& target){
    double cost = fixed_entry_cost_;
    
    // Calculate the vector from source to target
    double dx = target.position.x - source.position.x;
    double dy = target.position.y - source.position.y;
    // We'll ignore z difference for yaw calculation
    
    // Calculate the yaw angle of the vector from source to target
    double vector_yaw = atan2(dy, dx);
    
    // Extract yaw angles from quaternions
    double source_yaw = tf2::getYaw(source.orientation);
    double target_yaw = tf2::getYaw(target.orientation);
    
    // Calculate the absolute difference between source orientation and vector orientation
    double source_to_vector_diff = std::atan2(
        std::sin(vector_yaw - source_yaw), 
        std::cos(vector_yaw - source_yaw)
    );
    
    // Calculate the absolute difference between vector orientation and target orientation
    double vector_to_target_diff = std::atan2(
        std::sin(target_yaw - vector_yaw), 
        std::cos(target_yaw - vector_yaw)
    );
    
    // Add both rotation costs to the total cost
    double source_cost = std::fabs(source_to_vector_diff) * cost_per_radian_;
    double target_cost = std::fabs(vector_to_target_diff) * cost_per_radian_;
    cost += source_cost + target_cost;

    // RCLCPP_INFO(this->get_logger(), "Added cost for edge: Source Angle Diff: %lf Source Cost: %lf Target Angle Diff: %lf Target Cost: %lf, Cost per radian: %lf", source_to_vector_diff, source_cost, vector_to_target_diff, target_cost, cost_per_radian_);
    
    return cost;
}


void TrajectoryCostAugmenter::build_poses_map(
    const ViewPoses& msg, 
    std::unordered_map<uint32_t, geometry_msgs::msg::Pose>& map
)
{
    map.clear();
    for (auto pose : msg.poses){
        map[pose.id] = pose.pose;
    }
}



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryCostAugmenter>();

    rclcpp::spin(node);
    if (rclcpp::ok()){
        rclcpp::shutdown();
    }
    return 0;
}