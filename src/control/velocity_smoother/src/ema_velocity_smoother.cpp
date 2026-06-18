#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace ema_velocity_smoother
{

class EmaVelocitySmoother : public rclcpp::Node
{
    public:
        explicit EmaVelocitySmoother(const rclcpp::NodeOptions & options)
        : Node("ema_velocity_smoother", options), first_msg_(true)
        {
            // Declare parameters with default values
            this->declare_parameter("input_topic", "cmd_vel_raw");
            this->declare_parameter("output_topic", "cmd_vel_smoothed");
            this->declare_parameter("robot_base_frame", "base_link");
            this->declare_parameter("alpha_linear", 0.2);
            this->declare_parameter("alpha_angular", 0.2);

            // Retrieve parameters
            std::string input_topic = this->get_parameter("input_topic").as_string();
            std::string output_topic = this->get_parameter("output_topic").as_string();
            frame_id_ = this->get_parameter("robot_base_frame").as_string();
            alpha_linear_ = this->get_parameter("alpha_linear").as_double();
            alpha_angular_ = this->get_parameter("alpha_angular").as_double();

            // Validate alpha bounds
            alpha_linear_ = std::clamp(alpha_linear_, 0.0, 1.0);
            alpha_angular_ = std::clamp(alpha_angular_, 0.0, 1.0);

            RCLCPP_INFO(this->get_logger(), "Initializing EMA Smoother with Alpha Linear: %.2f, Angular: %.2f", 
                        alpha_linear_, alpha_angular_);

            // Setup QoS (using standard sensor/command data defaults)
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

            // Publishers and Subscribers
            pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, qos);
            sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            input_topic, qos, std::bind(&EmaVelocitySmoother::velocityCallback, this, std::placeholders::_1));
        }

    private:
        void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            auto smoothed_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            smoothed_msg->header.stamp = this->get_clock()->now();
            smoothed_msg->header.frame_id = frame_id_;

            if (first_msg_) {
            // Initialize filter with the very first message received
                current_linear_x_ = msg->twist.linear.x;
                current_linear_y_ = msg->twist.linear.y;
                current_angular_z_ = msg->twist.angular.z;
            first_msg_ = false;
            } else {
            // Apply Exponential Moving Average Formula
                current_linear_x_ = (alpha_linear_ * msg->twist.linear.x) + ((1.0 - alpha_linear_) * current_linear_x_);
                current_linear_y_ = (alpha_linear_ * msg->twist.linear.y) + ((1.0 - alpha_linear_) * current_linear_y_);
                current_angular_z_ = (alpha_angular_ * msg->twist.angular.z) + ((1.0 - alpha_angular_) * current_angular_z_);
            }

            // Populate smoothed message tracking
            smoothed_msg->twist.linear.x = current_linear_x_;
            smoothed_msg->twist.linear.y = current_linear_y_;
            smoothed_msg->twist.angular.z = current_angular_z_;

            pub_->publish(std::move(smoothed_msg));
        }

        // Node Variables
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;

        std::string frame_id_;
        double alpha_linear_;
        double alpha_angular_;
        bool first_msg_;

        // Filter States
        double current_linear_x_{0.0};
        double current_linear_y_{0.0};
        double current_angular_z_{0.0};
};

} // namespace ema_velocity_smoother

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ema_velocity_smoother::EmaVelocitySmoother>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}