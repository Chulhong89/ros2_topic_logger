#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace ros2_topic_logger
{

class CsvWriter
{
public:
  CsvWriter(
    std::string root_dir,
    std::string topic_name,
    std::string header,
    std::size_t max_file_size_bytes,
    bool event_file = false);

  void write_row(double recv_time, const std::string & row);

private:
  std::filesystem::path build_directory(double recv_time) const;
  std::filesystem::path build_file_path(double recv_time);
  void rotate_if_needed(double recv_time);
  void open_if_needed(double recv_time);
  static std::tm localtime_safe(std::time_t t);

  std::string root_dir_;
  std::string topic_name_;
  std::string header_;
  std::size_t max_file_size_bytes_;
  bool event_file_{false};

  std::ofstream stream_;
  std::filesystem::path current_path_;
  int current_part_index_{0};
  int current_year_{-1};
  int current_month_{-1};
  int current_day_{-1};
  int current_hour_{-1};
};

}  // namespace ros2_topic_logger
