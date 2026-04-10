#include "ros2_topic_logger/retention_manager.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace ros2_topic_logger
{

RetentionManager::RetentionManager(
  std::string root_dir,
  std::uintmax_t max_total_size_bytes,
  int max_age_days)
: root_dir_(std::move(root_dir)),
  max_total_size_bytes_(max_total_size_bytes),
  max_age_days_(max_age_days)
{
}

std::uintmax_t RetentionManager::calculate_total_size() const
{
  if (!std::filesystem::exists(root_dir_)) {
    return 0;
  }

  std::uintmax_t total = 0;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(root_dir_)) {
    if (entry.is_regular_file()) {
      total += entry.file_size();
    }
  }
  return total;
}

void RetentionManager::delete_old_files_by_age() const
{
  if (!std::filesystem::exists(root_dir_) || max_age_days_ <= 0) {
    return;
  }

  const auto now = std::filesystem::file_time_type::clock::now();
  const auto max_age = std::chrono::hours(24 * max_age_days_);

  for (const auto & entry : std::filesystem::recursive_directory_iterator(root_dir_)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto age = now - std::filesystem::last_write_time(entry.path());
    if (age > max_age) {
      std::filesystem::remove(entry.path());
    }
  }
}

void RetentionManager::delete_oldest_until_size_ok() const
{
  if (!std::filesystem::exists(root_dir_)) {
    return;
  }

  struct FileInfo
  {
    std::filesystem::path path;
    std::filesystem::file_time_type time;
    std::uintmax_t size;
  };

  std::vector<FileInfo> files;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(root_dir_)) {
    if (entry.is_regular_file()) {
      files.push_back({entry.path(), std::filesystem::last_write_time(entry.path()), entry.file_size()});
    }
  }

  std::sort(files.begin(), files.end(), [](const auto & a, const auto & b) {
    return a.time < b.time;
  });

  std::uintmax_t total_size = calculate_total_size();
  for (const auto & file : files) {
    if (total_size <= max_total_size_bytes_) {
      break;
    }
    std::filesystem::remove(file.path);
    if (total_size >= file.size) {
      total_size -= file.size;
    } else {
      total_size = 0;
    }
  }
}

void RetentionManager::enforce()
{
  delete_old_files_by_age();
  delete_oldest_until_size_ok();
}

}  // namespace ros2_topic_logger
