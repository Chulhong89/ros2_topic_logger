#pragma once

#include <deque>
#include <utility>
#include <vector>

namespace ros2_topic_logger
{

template<typename MessageT>
class EventBuffer
{
public:
  void push(double recv_time, const MessageT & msg)
  {
    buffer_.emplace_back(recv_time, msg);
  }

  void prune_older_than(double min_time)
  {
    while (!buffer_.empty() && buffer_.front().first < min_time) {
      buffer_.pop_front();
    }
  }

  std::vector<std::pair<double, MessageT>> snapshot() const
  {
    return std::vector<std::pair<double, MessageT>>(buffer_.begin(), buffer_.end());
  }

  void clear()
  {
    buffer_.clear();
  }

private:
  std::deque<std::pair<double, MessageT>> buffer_;
};

}  // namespace ros2_topic_logger
