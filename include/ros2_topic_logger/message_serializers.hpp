#pragma once

#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_topic_logger/topic_config.hpp"

namespace ros2_topic_logger
{

// ── 유틸리티 ──────────────────────────────────────────────────────────────────

inline double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline std::string join_vector(const std::vector<float> & values)
{
  std::ostringstream oss;
  oss.precision(9);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ';';
    }
    oss << values[i];
  }
  return oss.str();
}

inline double min_valid_range(const sensor_msgs::msg::LaserScan & msg)
{
  double min_range = std::numeric_limits<double>::infinity();
  for (const auto & r : msg.ranges) {
    if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max) {
      min_range = std::min(min_range, static_cast<double>(r));
    }
  }
  return min_range;
}

// ── Serializer 트레이트 ────────────────────────────────────────────────────────
//
// 새 메시지 타입 추가 방법:
//   1. MessageSerializer<YourMsgT> 특수화를 이 파일에 추가
//   2. topic_logger_node.cpp 의 register_types() 에 한 줄 등록
//   3. CMakeLists.txt / package.xml 의존성 추가
//
// 모든 특수화는 아래 두 정적 멤버를 구현해야 합니다:
//   static std::string header()
//   static std::string row(const MsgT &, double recv_time)
//
// EVENT_WINDOW 트리거가 필요한 타입은 추가로:
//   static bool event_trigger(const MsgT &, const TopicConfig &)
//
// CSV 컬럼 공통 포맷: recv_time, msg_time, frame_id, <타입별 필드...>

template<typename MsgT>
struct MessageSerializer;

// ── sensor_msgs/msg/Imu ───────────────────────────────────────────────────────

template<>
struct MessageSerializer<sensor_msgs::msg::Imu>
{
  static std::string header()
  {
    return "recv_time,msg_time,frame_id,"
           "orientation_x,orientation_y,orientation_z,orientation_w,"
           "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
           "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z";
  }

  static std::string row(const sensor_msgs::msg::Imu & msg, double recv_time)
  {
    std::ostringstream oss;
    oss.precision(15);
    oss << recv_time << ','
        << stamp_to_sec(msg.header.stamp) << ','
        << '"' << msg.header.frame_id << '"' << ','
        << msg.orientation.x << ',' << msg.orientation.y << ','
        << msg.orientation.z << ',' << msg.orientation.w << ','
        << msg.angular_velocity.x << ',' << msg.angular_velocity.y << ','
        << msg.angular_velocity.z << ','
        << msg.linear_acceleration.x << ',' << msg.linear_acceleration.y << ','
        << msg.linear_acceleration.z;
    return oss.str();
  }
};

// ── nav_msgs/msg/Odometry ─────────────────────────────────────────────────────

template<>
struct MessageSerializer<nav_msgs::msg::Odometry>
{
  static std::string header()
  {
    return "recv_time,msg_time,frame_id,child_frame_id,"
           "pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,"
           "twist_linear_x,twist_linear_y,twist_linear_z,"
           "twist_angular_x,twist_angular_y,twist_angular_z";
  }

  static std::string row(const nav_msgs::msg::Odometry & msg, double recv_time)
  {
    std::ostringstream oss;
    oss.precision(15);
    oss << recv_time << ','
        << stamp_to_sec(msg.header.stamp) << ','
        << '"' << msg.header.frame_id << '"' << ','
        << '"' << msg.child_frame_id << '"' << ','
        << msg.pose.pose.position.x << ',' << msg.pose.pose.position.y << ','
        << msg.pose.pose.position.z << ','
        << msg.pose.pose.orientation.x << ',' << msg.pose.pose.orientation.y << ','
        << msg.pose.pose.orientation.z << ',' << msg.pose.pose.orientation.w << ','
        << msg.twist.twist.linear.x << ',' << msg.twist.twist.linear.y << ','
        << msg.twist.twist.linear.z << ','
        << msg.twist.twist.angular.x << ',' << msg.twist.twist.angular.y << ','
        << msg.twist.twist.angular.z;
    return oss.str();
  }
};

// ── geometry_msgs/msg/PoseStamped ─────────────────────────────────────────────

template<>
struct MessageSerializer<geometry_msgs::msg::PoseStamped>
{
  static std::string header()
  {
    return "recv_time,msg_time,frame_id,"
           "pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w";
  }

  static std::string row(const geometry_msgs::msg::PoseStamped & msg, double recv_time)
  {
    std::ostringstream oss;
    oss.precision(15);
    oss << recv_time << ','
        << stamp_to_sec(msg.header.stamp) << ','
        << '"' << msg.header.frame_id << '"' << ','
        << msg.pose.position.x << ',' << msg.pose.position.y << ','
        << msg.pose.position.z << ','
        << msg.pose.orientation.x << ',' << msg.pose.orientation.y << ','
        << msg.pose.orientation.z << ',' << msg.pose.orientation.w;
    return oss.str();
  }
};

// ── sensor_msgs/msg/LaserScan ─────────────────────────────────────────────────

template<>
struct MessageSerializer<sensor_msgs::msg::LaserScan>
{
  static std::string header()
  {
    return "recv_time,msg_time,frame_id,"
           "angle_min,angle_max,angle_increment,time_increment,scan_time,"
           "range_min,range_max,ranges,intensities";
  }

  static std::string row(const sensor_msgs::msg::LaserScan & msg, double recv_time)
  {
    std::ostringstream oss;
    oss.precision(15);
    oss << recv_time << ','
        << stamp_to_sec(msg.header.stamp) << ','
        << '"' << msg.header.frame_id << '"' << ','
        << msg.angle_min << ',' << msg.angle_max << ',' << msg.angle_increment << ','
        << msg.time_increment << ',' << msg.scan_time << ','
        << msg.range_min << ',' << msg.range_max << ','
        << '"' << join_vector(msg.ranges) << '"' << ','
        << '"' << join_vector(msg.intensities) << '"';
    return oss.str();
  }

  static bool event_trigger(const sensor_msgs::msg::LaserScan & msg, const TopicConfig & cfg)
  {
    const double r = min_valid_range(msg);
    return std::isfinite(r) && r <= cfg.trigger_distance_threshold;
  }
};

}  // namespace ros2_topic_logger
