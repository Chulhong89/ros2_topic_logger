#pragma once

#include <functional>
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
#include "ros2_topic_logger/message_serializers.hpp"
#include "ros2_topic_logger/retention_manager.hpp"
#include "ros2_topic_logger/topic_config.hpp"

namespace ros2_topic_logger
{

class TopicLoggerNode : public rclcpp::Node
{
public:
  explicit TopicLoggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // EVENT_WINDOW 모드의 런타임 상태. 메시지 타입과 무관하게 공용으로 사용.
  struct EventState
  {
    bool active{false};
    double end_time_sec{0.0};
    std::shared_ptr<CsvWriter> writer;
  };

  void load_config(const std::string & config_path);
  void register_types();
  void create_subscriptions();

  // 새 메시지 타입을 추가할 때 register_types() 에서 이 함수를 호출합니다.
  // trigger: EVENT_WINDOW 트리거 조건. ALWAYS/PERIODIC 타입은 nullptr로 전달.
  template<typename MsgT>
  void create_subscription_impl(
    const TopicConfig & config,
    const rclcpp::QoS & qos,
    std::function<bool(const MsgT &, const TopicConfig &)> trigger = nullptr);

  double now_sec();
  bool should_log_periodic(const std::string & topic_key, double current_time);
  void on_retention_timer();

  GlobalConfig global_config_;
  std::vector<TopicConfig> topic_configs_;
  std::unique_ptr<RetentionManager> retention_manager_;
  rclcpp::TimerBase::SharedPtr retention_timer_;

  std::unordered_map<std::string, double> last_log_time_sec_;
  std::unordered_map<std::string, double> min_period_sec_;   // PERIODIC 판단용 캐시
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, std::shared_ptr<CsvWriter>> writers_;
  std::unordered_map<std::string, EventState> event_states_;

  using SubscriptionFactory = std::function<void(const TopicConfig &)>;
  std::unordered_map<std::string, SubscriptionFactory> type_registry_;
};

}  // namespace ros2_topic_logger
