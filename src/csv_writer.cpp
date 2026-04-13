#include "ros2_topic_logger/csv_writer.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ros2_topic_logger
{

CsvWriter::CsvWriter(
  std::string root_dir,
  std::string topic_name,
  std::string header,
  std::size_t max_file_size_bytes,
  bool event_file)
: root_dir_(std::move(root_dir)),
  topic_name_(std::move(topic_name)),
  header_(std::move(header)),
  max_file_size_bytes_(max_file_size_bytes),
  event_file_(event_file)
{
}

std::tm CsvWriter::localtime_safe(std::time_t t)
{
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  return tm;
}

std::filesystem::path CsvWriter::build_directory(double recv_time) const
{
  const auto tt = static_cast<std::time_t>(recv_time);
  const auto tm = localtime_safe(tt);

  std::ostringstream year;
  std::ostringstream month;
  std::ostringstream day;
  std::ostringstream hour;

  year << std::setw(4) << std::setfill('0') << (tm.tm_year + 1900);
  month << std::setw(2) << std::setfill('0') << (tm.tm_mon + 1);
  day << std::setw(2) << std::setfill('0') << tm.tm_mday;
  hour << std::setw(2) << std::setfill('0') << tm.tm_hour;

  return std::filesystem::path(root_dir_) / year.str() / month.str() / day.str() / hour.str();
}

std::filesystem::path CsvWriter::build_file_path(double recv_time)
{
  const auto tt = static_cast<std::time_t>(recv_time);
  const auto tm = localtime_safe(tt);

  current_year_ = tm.tm_year + 1900;
  current_month_ = tm.tm_mon + 1;
  current_day_ = tm.tm_mday;
  current_hour_ = tm.tm_hour;

  std::ostringstream filename;
  filename << topic_name_;
  if (event_file_) {
    filename << "_event";
  }
  filename << '_' << std::setw(3) << std::setfill('0') << current_part_index_ << ".csv";

  return build_directory(recv_time) / filename.str();
}

void CsvWriter::open_if_needed(double recv_time)
{
  const auto desired_dir = build_directory(recv_time);
  std::filesystem::create_directories(desired_dir);

  const auto tt = static_cast<std::time_t>(recv_time);
  const auto tm = localtime_safe(tt);
  const bool hour_changed =
    (current_year_ != tm.tm_year + 1900) ||
    (current_month_ != tm.tm_mon + 1) ||
    (current_day_ != tm.tm_mday) ||
    (current_hour_ != tm.tm_hour);

  if (!stream_.is_open() || hour_changed) {
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    current_part_index_ = 0;
    current_path_ = build_file_path(recv_time);
    stream_.open(current_path_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
      throw std::runtime_error("Failed to open log file: " + current_path_.string());
    }
    stream_ << header_ << '\n';
    stream_.flush();
  }
}

void CsvWriter::rotate_if_needed(double recv_time)
{
  open_if_needed(recv_time);

  if (std::filesystem::exists(current_path_) &&
      std::filesystem::file_size(current_path_) >= max_file_size_bytes_)
  {
    stream_.flush();
    stream_.close();
    ++current_part_index_;
    current_path_ = build_file_path(recv_time);
    stream_.open(current_path_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
      throw std::runtime_error("Failed to rotate log file: " + current_path_.string());
    }
    stream_ << header_ << '\n';
    stream_.flush();
  }
}

void CsvWriter::write_row(double file_time, const std::string & row)
{
  rotate_if_needed(file_time);
  stream_ << row << '\n';
  stream_.flush();
}

}  // namespace ros2_topic_logger
