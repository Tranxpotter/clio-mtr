#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL includes for merging and voxel filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudAggregator : public rclcpp::Node {
public:
  PointCloudAggregator() : Node("point_cloud_aggregator") {
    // Declare configurable parameters with default values
    this->declare_parameter<std::string>("input_topic", "/cloud_registered");
    this->declare_parameter<std::string>("output_topic", "/scan_cloud_aggregated");
    this->declare_parameter<double>("publish_rate_hz", 5.0);
    this->declare_parameter<double>("accumulation_time_sec", 0.4);
    this->declare_parameter<bool>("use_voxel_filter", true);
    this->declare_parameter<double>("voxel_leaf_size", 0.1);

    // Retrieve parameters
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("accumulation_time_sec", accumulation_time_sec_);
    this->get_parameter("use_voxel_filter", use_voxel_filter_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

    // Initialize subscriber and publisher
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 10,
        std::bind(&PointCloudAggregator::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 5);

    // Calculate timer period from desired Hz rate
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    pub_timer_ = this->create_wall_timer(
        timer_period, std::bind(&PointCloudAggregator::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Aggregator initialized. Listening to: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Accumulating for %.2f seconds. Publishing to %s at %.1f Hz",
                accumulation_time_sec_, output_topic_.c_str(), publish_rate_hz_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // Store message alongside its receive timestamp
    buffer_queue_.push_back({this->now(), msg});
  }

  void timerCallback() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (buffer_queue_.empty()) {
      return;
    }

    rclcpp::Time now = this->now();
    rclcpp::Duration max_age = rclcpp::Duration::from_seconds(accumulation_time_sec_);

    // 1. Evict scans older than the tracking window
    while (!buffer_queue_.empty() && (now - buffer_queue_.front().first) > max_age) {
      buffer_queue_.pop_front();
    }

    if (buffer_queue_.empty()) {
      return;
    }

    // 2. Concatenate the remaining clouds using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    std::string target_frame_id = buffer_queue_.back().second->header.frame_id;

    for (const auto& item : buffer_queue_) {
      pcl::PointCloud<pcl::PointXYZ> temp_pcl;
      pcl::fromROSMsg(*(item.second), temp_pcl);
      *combined_pcl += temp_pcl;
    }

    // 3. Optional: Downsample to keep memory under control for FAR Planner
    if (use_voxel_filter_ && !combined_pcl->empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(combined_pcl);
      voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_grid.filter(*filtered_pcl);
      combined_pcl = filtered_pcl;
    }

    // 4. Convert back to ROS2 Cloud msg and publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*combined_pcl, output_msg);
    output_msg.header.frame_id = target_frame_id;
    output_msg.header.stamp = now;

    cloud_pub_->publish(output_msg);
  }

  // Configurations
  std::string input_topic_;
  std::string output_topic_;
  double publish_rate_hz_;
  double accumulation_time_sec_;
  bool use_voxel_filter_;
  double voxel_leaf_size_;

  // Infrastructure
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  
  std::mutex buffer_mutex_;
  // Holds pairs of [Receive Time, Cloud MessagePointer]
  std::deque<std::pair<rclcpp::Time, sensor_msgs::msg::PointCloud2::SharedPtr>> buffer_queue_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudAggregator>());
  rclcpp::shutdown();
  return 0;
}