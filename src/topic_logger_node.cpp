#include "ros2_topic_logger/topic_logger_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "rclcpp/qos.hpp"

#include "ros2_topic_logger/message_serializers.hpp"

namespace ros2_topic_logger
{

namespace
{

LogMode parse_mode(const std::string & mode)
{
  if (mode == "always") {
    return LogMode::ALWAYS;
  }
  if (mode == "periodic") {
    return LogMode::PERIODIC;
  }
  if (mode == "event_window") {
    return LogMode::EVENT_WINDOW;
  }
  throw std::runtime_error("Unsupported log mode: " + mode);
}

}  // namespace

TopicLoggerNode::TopicLoggerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("topic_logger_node", options)
{
  this->declare_parameter<std::string>("config_path", "config/logger.yaml");
  const auto config_path = this->get_parameter("config_path").as_string();

  load_config(config_path);
  retention_manager_ = std::make_unique<RetentionManager>(
    global_config_.root_dir,
    global_config_.max_total_size_bytes,
    global_config_.max_age_days);

  create_subscriptions();

  retention_timer_ = this->create_wall_timer(
    std::chrono::minutes(5),
    std::bind(&TopicLoggerNode::on_retention_timer, this));

  RCLCPP_INFO(get_logger(), "Topic logger is ready. config_path=%s", config_path.c_str());
}

void TopicLoggerNode::load_config(const std::string & config_path)
{
  const YAML::Node root = YAML::LoadFile(config_path);

  global_config_.root_dir = root["root_dir"].as<std::string>("./logs");
  global_config_.max_total_size_bytes =
    root["max_total_size_mb"].as<std::uintmax_t>(5120) * 1024ull * 1024ull;
  global_config_.max_age_days = root["max_age_days"].as<int>(7);

  const auto topics = root["topics"];
  if (!topics || !topics.IsSequence()) {
    throw std::runtime_error("'topics' must exist and be a YAML sequence.");
  }

  for (const auto & topic_node : topics) {
    TopicConfig config;
    config.topic_name = topic_node["topic"].as<std::string>();
    config.msg_type = topic_node["type"].as<std::string>();
    config.mode = parse_mode(topic_node["mode"].as<std::string>("always"));
    config.min_period_sec = topic_node["min_period_ms"].as<double>(0.0) / 1000.0;
    config.trigger_distance_threshold = topic_node["trigger_distance_threshold"].as<double>(0.0);
    config.pre_buffer_sec = topic_node["pre_buffer_sec"].as<double>(0.0);
    config.post_buffer_sec = topic_node["post_buffer_sec"].as<double>(0.0);
    config.is_large_data = topic_node["is_large_data"].as<bool>(false);
    config.max_file_size_bytes =
      topic_node["max_file_size_mb"].as<std::size_t>(100) * 1024ull * 1024ull;

    topic_configs_.push_back(config);
  }
}

double TopicLoggerNode::now_sec() const
{
  return this->get_clock()->now().seconds();
}

bool TopicLoggerNode::should_log_periodic(const std::string & topic_name, double current_time)
{
  const auto it = last_log_time_sec_.find(topic_name);
  if (it == last_log_time_sec_.end()) {
    last_log_time_sec_[topic_name] = current_time;
    return true;
  }

  const auto & config = *std::find_if(
    topic_configs_.begin(), topic_configs_.end(),
    [&topic_name](const TopicConfig & cfg) { return cfg.topic_name == topic_name; });

  if ((current_time - it->second) >= config.min_period_sec) {
    it->second = current_time;
    return true;
  }
  return false;
}

void TopicLoggerNode::create_subscriptions()
{
  for (const auto & config : topic_configs_) {
    if (config.msg_type == "sensor_msgs/msg/Imu") {
      create_imu_subscription(config);
    } else if (config.msg_type == "nav_msgs/msg/Odometry") {
      create_odom_subscription(config);
    } else if (config.msg_type == "geometry_msgs/msg/PoseStamped") {
      create_pose_subscription(config);
    } else if (config.msg_type == "sensor_msgs/msg/LaserScan") {
      create_laserscan_subscription(config);
    } else {
      RCLCPP_WARN(
        get_logger(), "Unsupported type '%s' for topic '%s'. Skipped.",
        config.msg_type.c_str(), config.topic_name.c_str());
    }
  }
}

void TopicLoggerNode::create_imu_subscription(const TopicConfig & config)
{
  const auto topic_key = sanitize_topic_name(config.topic_name);
  writers_[topic_key] = std::make_shared<CsvWriter>(
    global_config_.root_dir, topic_key, imu_header(), config.max_file_size_bytes);

  auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
    config.topic_name,
    rclcpp::SensorDataQoS(),
    [this, config, topic_key](const sensor_msgs::msg::Imu::SharedPtr msg) {
      const double recv_time = now_sec();
      if (config.mode == LogMode::PERIODIC && !should_log_periodic(config.topic_name, recv_time)) {
        return;
      }
      try {
        writers_.at(topic_key)->write_row(recv_time, imu_row(*msg, recv_time));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "IMU write failed for %s: %s", config.topic_name.c_str(), e.what());
      }
    });

  subscriptions_.push_back(sub);
}

void TopicLoggerNode::create_odom_subscription(const TopicConfig & config)
{
  const auto topic_key = sanitize_topic_name(config.topic_name);
  writers_[topic_key] = std::make_shared<CsvWriter>(
    global_config_.root_dir, topic_key, odom_header(), config.max_file_size_bytes);

  auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
    config.topic_name,
    rclcpp::QoS(50),
    [this, config, topic_key](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const double recv_time = now_sec();
      if (config.mode == LogMode::PERIODIC && !should_log_periodic(config.topic_name, recv_time)) {
        return;
      }
      try {
        writers_.at(topic_key)->write_row(recv_time, odom_row(*msg, recv_time));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Odom write failed for %s: %s", config.topic_name.c_str(), e.what());
      }
    });

  subscriptions_.push_back(sub);
}

void TopicLoggerNode::create_pose_subscription(const TopicConfig & config)
{
  const auto topic_key = sanitize_topic_name(config.topic_name);
  writers_[topic_key] = std::make_shared<CsvWriter>(
    global_config_.root_dir, topic_key, pose_header(), config.max_file_size_bytes);

  auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    config.topic_name,
    rclcpp::QoS(50),
    [this, config, topic_key](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      const double recv_time = now_sec();
      if (config.mode == LogMode::PERIODIC && !should_log_periodic(config.topic_name, recv_time)) {
        return;
      }
      try {
        writers_.at(topic_key)->write_row(recv_time, pose_row(*msg, recv_time));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Pose write failed for %s: %s", config.topic_name.c_str(), e.what());
      }
    });

  subscriptions_.push_back(sub);
}

void TopicLoggerNode::create_laserscan_subscription(const TopicConfig & config)
{
  const auto topic_key = sanitize_topic_name(config.topic_name);
  scan_headers_[topic_key] = laserscan_header();
  scan_max_file_sizes_[topic_key] = config.max_file_size_bytes;
  event_active_[topic_key] = false;
  event_end_time_sec_[topic_key] = 0.0;

  auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    config.topic_name,
    rclcpp::SensorDataQoS(),
    [this, config, topic_key](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      const double recv_time = now_sec();
      auto & buffer = scan_buffers_[topic_key];
      buffer.push(recv_time, *msg);
      buffer.prune_older_than(recv_time - config.pre_buffer_sec);

      const double current_min_range = min_valid_range(*msg);
      const bool triggered = std::isfinite(current_min_range) &&
        current_min_range <= config.trigger_distance_threshold;

      if (triggered && !event_active_[topic_key]) {
        event_active_[topic_key] = true;
        event_end_time_sec_[topic_key] = recv_time + config.post_buffer_sec;
        event_writers_[topic_key] = std::make_shared<CsvWriter>(
          global_config_.root_dir,
          topic_key,
          scan_headers_[topic_key],
          scan_max_file_sizes_[topic_key],
          true);

        try {
          for (const auto & item : buffer.snapshot()) {
            event_writers_.at(topic_key)->write_row(item.first, laserscan_row(item.second, item.first));
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "LaserScan pre-buffer dump failed for %s: %s", config.topic_name.c_str(), e.what());
        }
      }

      if (event_active_[topic_key]) {
        if (triggered) {
          event_end_time_sec_[topic_key] = recv_time + config.post_buffer_sec;
        }

        try {
          event_writers_.at(topic_key)->write_row(recv_time, laserscan_row(*msg, recv_time));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "LaserScan event write failed for %s: %s", config.topic_name.c_str(), e.what());
        }

        if (recv_time > event_end_time_sec_[topic_key]) {
          event_active_[topic_key] = false;
          event_writers_.erase(topic_key);
          buffer.clear();
        }
      }
    });

  subscriptions_.push_back(sub);
}

void TopicLoggerNode::on_retention_timer()
{
  try {
    retention_manager_->enforce();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Retention enforcement failed: %s", e.what());
  }
}

}  // namespace ros2_topic_logger
