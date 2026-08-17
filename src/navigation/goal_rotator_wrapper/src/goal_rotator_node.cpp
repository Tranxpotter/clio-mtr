#include <cstdio>
#include <string>
#include <sstream>
#include <memory>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>

#include "nav_msgs/msg/odometry.hpp"

#include <inspection_planner_interfaces/action/nav_to_pose.hpp>



class GoalRotatorNode : public rclcpp::Node
{
    public: 
        GoalRotatorNode()
        : Node("goal_rotator_node")
        {
            /* Declare parameters */
            auto pose_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            pose_topic_desc.description = "Navigation goal pose topic [geometry_msgs/msg/PoseStamped]";
            this->declare_parameter<std::string>("pose_topic", "/goal_pose", pose_topic_desc);

            auto pose_server_desc = rcl_interfaces::msg::ParameterDescriptor();
            pose_server_desc.description = "Navigation goal pose action server [inspection_planner_interfaces/action/NavToPose]";
            this->declare_parameter<std::string>("pose_server", "/nav_to_pose", pose_server_desc);

            auto goal_point_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            goal_point_topic_desc.description = "FAR Planner goal point topic [geometry_msgs/msg/PoseStamped]";
            this->declare_parameter<std::string>("goal_point_topic", "/goal_point", goal_point_topic_desc);

            auto planner_status_desc = rcl_interfaces::msg::ParameterDescriptor();
            planner_status_desc.description = "FAR Planner navigation status topic [std_msgs/msg/Bool]";
            this->declare_parameter<std::string>("planner_status_topic", "/far_reach_goal_status", planner_status_desc);

            auto planner_cmd_vel_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            planner_cmd_vel_topic_desc.description = "Local planner output cmd_vel topic [geometry_msgs/msg/Twist]";
            this->declare_parameter<std::string>("planner_cmd_vel_topic", "/cmd_vel", planner_cmd_vel_topic_desc);

            auto planner_cmd_vel_stamped_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            planner_cmd_vel_stamped_topic_desc.description = "Local planner output cmd_vel topic [geometry_msgs/msg/TwistStamped]";
            this->declare_parameter<std::string>("planner_cmd_vel_stamped_topic", "/cmd_vel", planner_cmd_vel_stamped_topic_desc);

            auto use_stamped_cmd_vel_desc = rcl_interfaces::msg::ParameterDescriptor();
            use_stamped_cmd_vel_desc.description = "Use stamped cmd_vel topic";
            this->declare_parameter<bool>("use_stamped_cmd_vel", false, use_stamped_cmd_vel_desc);

            auto max_rotation_speed_desc = rcl_interfaces::msg::ParameterDescriptor();
            max_rotation_speed_desc.description = "Maximum turning speed output value";
            this->declare_parameter<double>("max_rotation_speed", 1.0, max_rotation_speed_desc);

            auto min_rotation_speed_desc = rcl_interfaces::msg::ParameterDescriptor();
            min_rotation_speed_desc.description = "Minimum turning speed output value";
            this->declare_parameter<double>("min_rotation_speed", 0.0, min_rotation_speed_desc);

            auto angle_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor();
            angle_tolerance_desc.description = "Angle diff tolerance in degrees";
            this->declare_parameter<double>("angle_tolerance", 20.0, angle_tolerance_desc);

            auto kp_desc = rcl_interfaces::msg::ParameterDescriptor();
            kp_desc.description = "P-Controller multiplier";
            this->declare_parameter<double>("kp", 1.5, kp_desc);

            auto target_distance_threshold_desc = rcl_interfaces::msg::ParameterDescriptor();
            target_distance_threshold_desc.description = "Maximum distance to the target to consider finished adjusting (meters)";
            this->declare_parameter<double>("target_distance_threshold", 0.1, target_distance_threshold_desc);

            auto approach_rate_threshold_desc = rcl_interfaces::msg::ParameterDescriptor();
            approach_rate_threshold_desc.description = "Minimum rate of distance reduction required between robot and target to consider finished adjusting(m/s)";
            this->declare_parameter<double>("approach_rate_threshold", 0.03, kp_desc);

            auto odom_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            odom_topic_desc.description = "Robot odometry topic [nav_msgs/msg/Odometry]";
            this->declare_parameter<std::string>("odom_topic", "/Odometry", odom_topic_desc);

            auto output_topic_desc = rcl_interfaces::msg::ParameterDescriptor();
            output_topic_desc.description = "Output cmd_vel topic name [geometry_msgs/msg/Twist]";
            this->declare_parameter<std::string>("output_topic", "/cmd_vel_out", output_topic_desc);

            auto output_hz_desc = rcl_interfaces::msg::ParameterDescriptor();
            output_hz_desc.description = "Output cmd_vel hz";
            this->declare_parameter<double>("output_hz", 50.0, output_hz_desc);

            auto verbose_desc = rcl_interfaces::msg::ParameterDescriptor();
            verbose_desc.description = "Debug use";
            this->declare_parameter<bool>("verbose", false, verbose_desc);



            /* Get parameter values */
            this->pose_topic_ = this->get_parameter("pose_topic").as_string();
            this->pose_server_name_ = this->get_parameter("pose_server").as_string();
            this->goal_point_topic_ = this->get_parameter("goal_point_topic").as_string();
            this->planner_status_topic_ = this->get_parameter("planner_status_topic").as_string();
            this->planner_cmd_vel_topic_ = this->get_parameter("planner_cmd_vel_topic").as_string();
            this->planner_cmd_vel_stamped_topic_ = this->get_parameter("planner_cmd_vel_stamped_topic").as_string();
            this->use_stamped_cmd_vel_ = this->get_parameter("use_stamped_cmd_vel").as_bool();
            this->max_rotation_speed_ = this->get_parameter("max_rotation_speed").as_double();
            this->min_rotation_speed_ = this->get_parameter("min_rotation_speed").as_double();
            this->angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
            this->kp_ = this->get_parameter("kp").as_double();
            this->target_distance_threshold_ = this->get_parameter("target_distance_threshold").as_double();
            this->approach_rate_threshold_ = this->get_parameter("approach_rate_threshold").as_double();
            this->odom_topic_ = this->get_parameter("odom_topic").as_string();
            this->output_topic_ = this->get_parameter("output_topic").as_string();
            this->output_hz_ = this->get_parameter("output_hz").as_double();
            this->verbose_ = this->get_parameter("verbose").as_bool();

            /* Create subscribers and publishers */
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic_, 1, std::bind(&GoalRotatorNode::on_receive_pose, this, std::placeholders::_1));
            pose_server_ = rclcpp_action::create_server<inspection_planner_interfaces::action::NavToPose>(
                this,  
                pose_server_name_, 
                std::bind(&GoalRotatorNode::server_handle_pose, this, std::placeholders::_1, std::placeholders::_2), 
                std::bind(&GoalRotatorNode::server_handle_cancel, this, std::placeholders::_1), 
                std::bind(&GoalRotatorNode::server_handle_accepted, this, std::placeholders::_1)
            );
            goal_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(goal_point_topic_, 1);
            planner_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(planner_status_topic_, 5, std::bind(&GoalRotatorNode::on_receive_status, this, std::placeholders::_1));
            if (!use_stamped_cmd_vel_) {
                cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(planner_cmd_vel_topic_, 5, std::bind(&GoalRotatorNode::on_receive_cmd_vel, this, std::placeholders::_1));
            }
            else {
                cmd_vel_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(planner_cmd_vel_stamped_topic_, 5, std::bind(&GoalRotatorNode::on_receive_cmd_vel_stamped, this, std::placeholders::_1));
            }
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 5, std::bind(&GoalRotatorNode::odom_handler, this, std::placeholders::_1));
            output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 5);
        
            /* TF Frame listener */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /* Output timer */
            timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000/output_hz_)), std::bind(&GoalRotatorNode::timer_callback, this));


            /* Debug */
            debug_angle_diff_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rotator/angle_diff", 10);
        }
    
    private:
        // Node parameters
        std::string pose_topic_;
        std::string pose_server_name_;
        std::string goal_point_topic_;
        std::string planner_status_topic_;
        std::string planner_cmd_vel_topic_;
        std::string planner_cmd_vel_stamped_topic_;
        bool use_stamped_cmd_vel_;
        double max_rotation_speed_;
        double min_rotation_speed_;
        double angle_tolerance_;
        double kp_;
        double target_distance_threshold_;
        double approach_rate_threshold_;
        std::string odom_topic_;
        std::string output_topic_;
        double output_hz_;
        bool verbose_;

        // Subscribers and publishers
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> pose_sub_;
        rclcpp_action::Server<inspection_planner_interfaces::action::NavToPose>::SharedPtr pose_server_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> goal_point_pub_;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> planner_status_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> cmd_vel_stamped_sub_;
        std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> output_pub_;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> debug_angle_diff_pub_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        std::shared_ptr<rclcpp::TimerBase> timer_;

        // States
        const int IDLE = 0, NAVIGATING = 1, ADJUSTING = 2, ROTATING = 3;
        std::atomic<int> state = 0;
        bool is_pose_from_server = false;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<inspection_planner_interfaces::action::NavToPose>> active_goal_handle_{nullptr};
        std::mutex goal_handle_mutex_;

        geometry_msgs::msg::PoseStamped goal_pose_;
        std::shared_ptr<nav_msgs::msg::Odometry> odom_;
        std::string odom_frame = "";
        std::string robot_frame = "";
        geometry_msgs::msg::Twist curr_output_vel_;
        double prev_distance_ = 0.0;
        rclcpp::Time prev_odom_time_;

        /**
         * @brief Get the goal point from pose object
         * 
         * @param pose [in] goal pose
         * @param point [out] result goal point
         */
        void get_goal_point_from_pose(const geometry_msgs::msg::PoseStamped &pose, geometry_msgs::msg::PointStamped &point){
            point.header = pose.header;
            point.point = pose.pose.position;
        }

        /**
         * @brief Goal Pose callback, aborts old goal, update state and publish goal point
         * 
         * @param msg 
         */
        void on_receive_pose(std::shared_ptr<geometry_msgs::msg::PoseStamped> msg){
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);

            if (active_goal_handle_ && active_goal_handle_->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Topic goal received while action server busy. Aborting old goal...");
                auto result = std::make_shared<inspection_planner_interfaces::action::NavToPose::Result>();
                result->result = false;
                active_goal_handle_->abort(result);
                active_goal_handle_ = nullptr;
            }

            goal_pose_ = *msg;
            is_pose_from_server = false;
            RCLCPP_INFO(this->get_logger(), "Received goal pose from message. Navigating to pose...");

            auto goal_point = geometry_msgs::msg::PointStamped();
            get_goal_point_from_pose(*msg, goal_point);
            goal_point_pub_->publish(goal_point);

            state = NAVIGATING;
        }

        /**
         * @brief Action server callback, aborts old goal, update state and publish goal point
         * 
         * @param uuid 
         * @param goal 
         * @return rclcpp_action::GoalResponse 
         */
        rclcpp_action::GoalResponse server_handle_pose(
            const rclcpp_action::GoalUUID & uuid, 
            inspection_planner_interfaces::action::NavToPose::Goal::ConstSharedPtr goal)
        {
            (void)uuid;
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);

            if (active_goal_handle_ && active_goal_handle_->is_active()){
                RCLCPP_WARN(this->get_logger(), "New goal received while busy, aborting old goal...");
                auto result = std::make_shared<inspection_planner_interfaces::action::NavToPose::Result>();
                result->result = false;
                active_goal_handle_->abort(result);
                active_goal_handle_ = nullptr;
            }

            goal_pose_ = goal->pose;
            RCLCPP_INFO(this->get_logger(), "Received goal pose from server. Accepting goal...");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        /**
         * @brief Cancel goal, send goal_point to robot odom to stop navigation
         * 
         * @param goal_handle 
         * @return rclcpp_action::CancelResponse 
         */
        rclcpp_action::CancelResponse server_handle_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inspection_planner_interfaces::action::NavToPose>> goal_handle
        )
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);

            if (goal_handle == active_goal_handle_){
                RCLCPP_INFO(this->get_logger(), "Received cancel request for the active goal, cancelling goal...");
                auto goal_point = geometry_msgs::msg::PointStamped();
                goal_point.header = goal_pose_.header;
                if (odom_){
                    goal_point.point = odom_->pose.pose.position;
                }   
                goal_point_pub_->publish(goal_point);
                curr_output_vel_ = geometry_msgs::msg::Twist();
            }
            // goal_handle->canceled(result);

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void server_handle_accepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inspection_planner_interfaces::action::NavToPose>> goal_handle
        )
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted, navigating to pose...");
            
            auto feedback = std::make_shared<inspection_planner_interfaces::action::NavToPose::Feedback>();
            feedback->state = NAVIGATING;

            auto goal_point = geometry_msgs::msg::PointStamped();
            {
                std::lock_guard<std::mutex> lock(goal_handle_mutex_);
                active_goal_handle_ = goal_handle;
                is_pose_from_server = true;
                state = NAVIGATING;
                get_goal_point_from_pose(goal_pose_, goal_point);
            }

            goal_point_pub_->publish(goal_point);
            goal_handle->publish_feedback(feedback);
        }





        

        /**
         * @brief Update state, start rotation if reached goal
         * 
         * @param msg True if reached goal
         */
        void on_receive_status(std::shared_ptr<std_msgs::msg::Bool> msg){
            if (!msg->data) return;

            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if (state == NAVIGATING){
                state = ADJUSTING;
                this->prev_distance_ = 0.0;
                this->prev_odom_time_ = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "Done navigation, switching to ADJUSTING. Pose from server: %d", is_pose_from_server);
                if (is_pose_from_server){
                    auto feedback = std::make_shared<inspection_planner_interfaces::action::NavToPose::Feedback>();
                    feedback->state = ADJUSTING;
                    active_goal_handle_->publish_feedback(feedback);
                }
            }
        }

        void on_receive_cmd_vel(std::shared_ptr<geometry_msgs::msg::Twist> msg){
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if (state != ROTATING){
                curr_output_vel_ = *msg;
            }
        }

        void on_receive_cmd_vel_stamped(std::shared_ptr<geometry_msgs::msg::TwistStamped> msg){
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if (state != ROTATING){
                auto output = msg->twist;
                curr_output_vel_ = output;
            }
        }
        
        void odom_handler(std::shared_ptr<nav_msgs::msg::Odometry> msg){
            if (odom_frame == "" || robot_frame == ""){
                odom_frame = msg->header.frame_id;
                robot_frame = msg->child_frame_id;
            }

            odom_ = msg;
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if (active_goal_handle_ && active_goal_handle_->is_canceling()){
                auto result = std::make_shared<inspection_planner_interfaces::action::NavToPose::Result>();
                result->result = false;
                active_goal_handle_->canceled(result);
                state = IDLE;
                RCLCPP_INFO(this->get_logger(), "Goal canceled successfully.");
            }

            if (state == IDLE || state == NAVIGATING) return;

            geometry_msgs::msg::PoseStamped transformed_goal_pose;
            convert_goal_pose_frame(goal_pose_, transformed_goal_pose);

            if (state == ADJUSTING){
                double odom_x = msg->pose.pose.position.x;
                double odom_y = msg->pose.pose.position.y;
                double goal_x = transformed_goal_pose.pose.position.x;
                double goal_y = transformed_goal_pose.pose.position.y;
                double diff_x = odom_x - goal_x;
                double diff_y = odom_y - goal_y;
                double distance = diff_x * diff_x + diff_y * diff_y;
                if (verbose_){
                    RCLCPP_INFO(this->get_logger(), "ADJUSTING, distance to goal: %lf", distance);    
                }
                if (distance <= (target_distance_threshold_*target_distance_threshold_)){
                    state = ROTATING;
                    curr_output_vel_ = geometry_msgs::msg::Twist();
                    RCLCPP_INFO(this->get_logger(), "Done ADJUSTING, now ROTATING, distance to goal < threshold.");
                    return;
                }

                rclcpp::Time now = this->get_clock()->now();
                if (prev_distance_ != 0.0){
                    auto time_diff = now - prev_odom_time_;
                    auto distance_change = prev_distance_ - distance;
                    auto approach_rate = distance_change / time_diff.seconds();
                    if (verbose_){
                        RCLCPP_INFO(this->get_logger(), "ADJUSTING, approach rate: %lf", approach_rate);
                    }
                    if (approach_rate < this->approach_rate_threshold_){
                        state = ROTATING;
                        curr_output_vel_ = geometry_msgs::msg::Twist();
                        RCLCPP_INFO(this->get_logger(), "Done ADJUSTING, now ROTATING, approach rate < threshold.");
                        return;
                    }
                }
                prev_distance_ = distance;
                prev_odom_time_ = now;
            }

            if (state == ROTATING){
                // Convert goal pose to odom frame                
                double target_yaw = tf2::getYaw(transformed_goal_pose.pose.orientation);

                geometry_msgs::msg::Quaternion robot_quaternion = odom_->pose.pose.orientation;
                double robot_yaw = tf2::getYaw(robot_quaternion);

                double yaw_diff = target_yaw - robot_yaw;
                yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));

                if (verbose_){
                    RCLCPP_INFO(this->get_logger(), "ROTATING, angle diff: %lf", yaw_diff);
                }
                std_msgs::msg::Float32 debug_msg;
                debug_msg.data = yaw_diff;
                this->debug_angle_diff_pub_->publish(debug_msg);

                if (abs(yaw_diff) <= (angle_tolerance_ / 180.0 * 3.1415926)){
                    state = IDLE;
                    curr_output_vel_ = geometry_msgs::msg::Twist();
                    RCLCPP_INFO(this->get_logger(), "Done ROTATING, now IDLE. Pose from server: %d", is_pose_from_server);
                    
                    // pub action server result
                    if (is_pose_from_server && active_goal_handle_){
                        if (active_goal_handle_->is_active()){
                            auto result = std::make_shared<inspection_planner_interfaces::action::NavToPose::Result>();
                            result->result = true;
                            active_goal_handle_->succeed(result);
                        }
                        
                        active_goal_handle_ = nullptr;
                        is_pose_from_server = false;
                    }
                    return;
                }
                double cmd_vel_angular_z = kp_*yaw_diff;
                cmd_vel_angular_z = std::max(-max_rotation_speed_, std::min(max_rotation_speed_, cmd_vel_angular_z));
                if (abs(cmd_vel_angular_z) < min_rotation_speed_){
                    if (cmd_vel_angular_z < 0.0) cmd_vel_angular_z = -min_rotation_speed_;
                    else cmd_vel_angular_z = min_rotation_speed_; 
                }
                curr_output_vel_ = geometry_msgs::msg::Twist();
                curr_output_vel_.angular.z = cmd_vel_angular_z;
            }
        }
        
        /**
         * @brief Transform goal pose from pose frame to odom frame
         * 
         * @param pose [in] original goal pose
         * @param transformed_pose [out] transformed goal pose
         */
        void convert_goal_pose_frame(const geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::PoseStamped& transformed_pose){
            auto pose_frame = pose.header.frame_id;
            if (pose_frame == odom_frame || odom_frame == "") return;
            try{
                auto transform = tf_buffer_->lookupTransform(odom_frame, pose_frame, tf2::TimePointZero);
                tf2::doTransform<geometry_msgs::msg::PoseStamped>(pose, transformed_pose, transform);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to transform goal pose from %s to %s: %s", pose_frame.c_str(), odom_frame.c_str(), ex.what());
                transformed_pose = pose;
            }
        }

        void timer_callback(){
            geometry_msgs::msg::Twist output_vel;
            {
                std::lock_guard<std::mutex> lock(goal_handle_mutex_);
                output_vel = curr_output_vel_;
            }
            output_pub_->publish(output_vel);
        }

}; 


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalRotatorNode>());
    if (rclcpp::ok()){
        rclcpp::shutdown();
    }
    return 0;
}
