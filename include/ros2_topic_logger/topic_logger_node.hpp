#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ros2_topic_logger/csv_writer.hpp"
#include "ros2_topic_logger/event_buffer.hpp"
#include "ros2_topic_logger/retention_manager.hpp"
#include "ros2_topic_logger/topic_config.hpp"

namespace ros2_topic_logger
{

class TopicLoggerNode : public rclcpp::Node
{
public:
  explicit TopicLoggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void load_config(const std::string & config_path);
  void create_subscriptions();
  void create_imu_subscription(const TopicConfig & config);
  void create_odom_subscription(const TopicConfig & config);
  void create_pose_subscription(const TopicConfig & config);
  void create_laserscan_subscription(const TopicConfig & config);

  double now_sec() const;
  bool should_log_periodic(const std::string & topic_name, double current_time);
  void on_retention_timer();

  GlobalConfig global_config_;
  std::vector<TopicConfig> topic_configs_;
  std::unique_ptr<RetentionManager> retention_manager_;
  rclcpp::TimerBase::SharedPtr retention_timer_;

  std::unordered_map<std::string, double> last_log_time_sec_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, std::shared_ptr<CsvWriter>> writers_;

  // LaserScan event mode state.
  std::unordered_map<std::string, std::shared_ptr<CsvWriter>> event_writers_;
  std::unordered_map<std::string, EventBuffer<sensor_msgs::msg::LaserScan>> scan_buffers_;
  std::unordered_map<std::string, bool> event_active_;
  std::unordered_map<std::string, double> event_end_time_sec_;
  std::unordered_map<std::string, std::string> scan_headers_;
  std::unordered_map<std::string, std::size_t> scan_max_file_sizes_;
};

}  // namespace ros2_topic_logger
