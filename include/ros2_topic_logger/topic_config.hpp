#pragma once

#include <cstddef>
#include <string>

namespace ros2_topic_logger
{

enum class LogMode
{
  ALWAYS,
  PERIODIC,
  EVENT_WINDOW
};

struct TopicConfig
{
  std::string topic_name;
  std::string msg_type;
  LogMode mode{LogMode::ALWAYS};

  // PERIODIC mode
  double min_period_sec{0.0};

  // EVENT_WINDOW mode
  double trigger_distance_threshold{0.0};
  double pre_buffer_sec{0.0};
  double post_buffer_sec{0.0};

  // Writer behavior
  bool is_large_data{false};
  std::size_t max_file_size_bytes{100 * 1024 * 1024};
};

struct GlobalConfig
{
  std::string root_dir{"./logs"};
  std::uintmax_t max_total_size_bytes{5ull * 1024ull * 1024ull * 1024ull};
  int max_age_days{7};
};

inline std::string sanitize_topic_name(const std::string & topic_name)
{
  std::string out;
  out.reserve(topic_name.size());

  for (char c : topic_name) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      out.push_back(c);
    } else if (c == '/') {
      if (!out.empty() && out.back() != '_') {
        out.push_back('_');
      }
    } else {
      out.push_back('_');
    }
  }

  while (!out.empty() && out.front() == '_') {
    out.erase(out.begin());
  }
  while (!out.empty() && out.back() == '_') {
    out.pop_back();
  }

  return out.empty() ? "topic" : out;
}

}  // namespace ros2_topic_logger
