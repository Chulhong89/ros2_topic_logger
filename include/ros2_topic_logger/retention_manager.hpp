#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace ros2_topic_logger
{

class RetentionManager
{
public:
  RetentionManager(std::string root_dir, std::uintmax_t max_total_size_bytes, int max_age_days);

  void enforce();

private:
  std::uintmax_t calculate_total_size() const;
  void delete_old_files_by_age() const;
  void delete_oldest_until_size_ok() const;

  std::filesystem::path root_dir_;
  std::uintmax_t max_total_size_bytes_;
  int max_age_days_;
};

}  // namespace ros2_topic_logger
