#include "ros2_topic_logger/topic_logger_node.hpp"

#include <fstream>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

#include "rclcpp/qos.hpp"

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

// ── 템플릿 구현 ───────────────────────────────────────────────────────────────
//
// 모든 인스턴스화가 이 파일 안에서 일어나므로 .cpp 에 정의해도 무방합니다.

template<typename MsgT>
void TopicLoggerNode::create_subscription_impl(
  const TopicConfig & config,
  const rclcpp::QoS & qos,
  std::function<bool(const MsgT &, const TopicConfig &)> trigger)
{
  using S = MessageSerializer<MsgT>;
  const auto topic_key = sanitize_topic_name(config.topic_name);

  min_period_sec_[topic_key] = config.min_period_sec;

  if (config.mode == LogMode::EVENT_WINDOW) {
    event_states_[topic_key] = EventState{};
    auto buffer = std::make_shared<EventBuffer<MsgT>>();

    auto sub = this->create_subscription<MsgT>(
      config.topic_name, qos,
      [this, config, topic_key, buffer, trigger](const typename MsgT::SharedPtr msg) {
        const double recv_time = now_sec();

        // 현재 메시지를 버퍼에 추가하기 전에 먼저 오래된 항목을 제거합니다.
        buffer->prune_older_than(recv_time - config.pre_buffer_sec);

        const bool triggered = trigger && trigger(*msg, config);
        auto & state = event_states_.at(topic_key);

        if (triggered && !state.active) {
          state.active = true;
          state.end_time_sec = recv_time + config.post_buffer_sec;
          state.writer = std::make_shared<CsvWriter>(
            global_config_.root_dir, topic_key, S::header(),
            config.max_file_size_bytes, /*event_file=*/true);

          // pre-buffer 덤프: 현재 메시지는 아직 버퍼에 없으므로 중복 없음.
          try {
            for (const auto & [t, m] : buffer->snapshot()) {
              state.writer->write_row(t, S::row(m, t));
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Pre-buffer dump failed for %s: %s",
              config.topic_name.c_str(), e.what());
          }
        }

        // 현재 메시지를 버퍼에 추가 (pre-buffer 덤프 이후에 추가).
        buffer->push(recv_time, *msg);

        if (state.active) {
          if (triggered) {
            state.end_time_sec = recv_time + config.post_buffer_sec;
          }
          try {
            state.writer->write_row(recv_time, S::row(*msg, recv_time));
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Event write failed for %s: %s",
              config.topic_name.c_str(), e.what());
          }

          if (recv_time > state.end_time_sec) {
            state.active = false;
            state.writer.reset();
            buffer->clear();
          }
        }
      });

    subscriptions_.push_back(sub);

  } else {
    // ALWAYS / PERIODIC
    writers_[topic_key] = std::make_shared<CsvWriter>(
      global_config_.root_dir, topic_key, S::header(), config.max_file_size_bytes);

    auto sub = this->create_subscription<MsgT>(
      config.topic_name, qos,
      [this, config, topic_key](const typename MsgT::SharedPtr msg) {
        const double recv_time = now_sec();
        if (config.mode == LogMode::PERIODIC &&
            !should_log_periodic(topic_key, recv_time)) {
          return;
        }
        try {
          writers_.at(topic_key)->write_row(recv_time, S::row(*msg, recv_time));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Write failed for %s: %s",
            config.topic_name.c_str(), e.what());
        }
      });

    subscriptions_.push_back(sub);
  }
}

// ── 타입 등록 ─────────────────────────────────────────────────────────────────
//
// 새 메시지 타입 추가 시 이 함수에 한 줄만 추가하면 됩니다.

void TopicLoggerNode::register_types()
{
  type_registry_["sensor_msgs/msg/Imu"] = [this](const TopicConfig & cfg) {
    create_subscription_impl<sensor_msgs::msg::Imu>(cfg, rclcpp::SensorDataQoS());
  };
  type_registry_["nav_msgs/msg/Odometry"] = [this](const TopicConfig & cfg) {
    create_subscription_impl<nav_msgs::msg::Odometry>(cfg, rclcpp::QoS(50));
  };
  type_registry_["geometry_msgs/msg/PoseStamped"] = [this](const TopicConfig & cfg) {
    create_subscription_impl<geometry_msgs::msg::PoseStamped>(cfg, rclcpp::QoS(50));
  };
  type_registry_["sensor_msgs/msg/LaserScan"] = [this](const TopicConfig & cfg) {
    create_subscription_impl<sensor_msgs::msg::LaserScan>(
      cfg, rclcpp::SensorDataQoS(),
      &MessageSerializer<sensor_msgs::msg::LaserScan>::event_trigger);
  };
}

// ── 노드 초기화 ───────────────────────────────────────────────────────────────

TopicLoggerNode::TopicLoggerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("topic_logger_node", options)
{
  this->declare_parameter<std::string>("config_path", "config/logger.yaml");
  const auto config_path = this->get_parameter("config_path").as_string();

  load_config(config_path);
  register_types();

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

void TopicLoggerNode::create_subscriptions()
{
  for (const auto & config : topic_configs_) {
    const auto it = type_registry_.find(config.msg_type);
    if (it == type_registry_.end()) {
      RCLCPP_WARN(get_logger(), "Unsupported type '%s' for topic '%s'. Skipped.",
        config.msg_type.c_str(), config.topic_name.c_str());
      continue;
    }
    it->second(config);
  }
}

double TopicLoggerNode::now_sec()
{
  return this->get_clock()->now().seconds();
}

bool TopicLoggerNode::should_log_periodic(const std::string & topic_key, double current_time)
{
  auto [it, inserted] = last_log_time_sec_.emplace(topic_key, current_time);
  if (inserted) {
    return true;
  }
  if ((current_time - it->second) >= min_period_sec_.at(topic_key)) {
    it->second = current_time;
    return true;
  }
  return false;
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
