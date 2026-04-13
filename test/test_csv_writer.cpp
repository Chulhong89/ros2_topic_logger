#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_topic_logger/csv_writer.hpp"

namespace fs = std::filesystem;
using ros2_topic_logger::CsvWriter;

// ── fixture ───────────────────────────────────────────────────────────────────

class CsvWriterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / "ros2_logger_csv_test";
    fs::remove_all(root_);
    fs::create_directories(root_);
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  // Collect all .csv files under root_
  std::vector<fs::path> csv_files() const
  {
    std::vector<fs::path> result;
    for (const auto & entry : fs::recursive_directory_iterator(root_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        result.push_back(entry.path());
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  // Read all lines from a file
  static std::vector<std::string> read_lines(const fs::path & p)
  {
    std::vector<std::string> lines;
    std::ifstream in(p);
    std::string line;
    while (std::getline(in, line)) {
      lines.push_back(line);
    }
    return lines;
  }

  fs::path root_;
};

// ── tests ─────────────────────────────────────────────────────────────────────

TEST_F(CsvWriterTest, WritingOneRowCreatesFile)
{
  const std::string header = "recv_time,value";
  CsvWriter writer(root_.string(), "test_topic", header, 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1.0,42");

  EXPECT_FALSE(csv_files().empty());
}

TEST_F(CsvWriterTest, FirstLineIsHeader)
{
  const std::string header = "recv_time,value";
  CsvWriter writer(root_.string(), "my_topic", header, 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1.0,99");

  const auto files = csv_files();
  ASSERT_FALSE(files.empty());

  const auto lines = read_lines(files.front());
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(lines[0], header);
}

TEST_F(CsvWriterTest, RowDataAppearsAfterHeader)
{
  const std::string header = "recv_time,value";
  CsvWriter writer(root_.string(), "my_topic", header, 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1.0,99");

  const auto files = csv_files();
  ASSERT_FALSE(files.empty());

  const auto lines = read_lines(files.front());
  ASSERT_GE(lines.size(), 2u);
  EXPECT_EQ(lines[1], "1.0,99");
}

TEST_F(CsvWriterTest, FileNameContainsTopicName)
{
  CsvWriter writer(root_.string(), "odom", "h", 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "row");

  const auto files = csv_files();
  ASSERT_FALSE(files.empty());
  EXPECT_NE(files[0].filename().string().find("odom"), std::string::npos);
}

TEST_F(CsvWriterTest, EventFileNameContainsEventSuffix)
{
  CsvWriter writer(root_.string(), "scan", "h", 100 * 1024 * 1024, /*event_file=*/true);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "row");

  const auto files = csv_files();
  ASSERT_FALSE(files.empty());
  EXPECT_NE(files[0].filename().string().find("_event"), std::string::npos);
}

TEST_F(CsvWriterTest, DirectoryStructureIsYYYYMMDDHH)
{
  CsvWriter writer(root_.string(), "test", "h", 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "row");

  const auto files = csv_files();
  ASSERT_FALSE(files.empty());

  // path structure: root / YYYY / MM / DD / HH / filename.csv
  // That means the csv file should be at depth 5 below root (4 directories + file).
  const auto rel = fs::relative(files[0], root_);
  // rel has 5 components: YYYY, MM, DD, HH, filename
  EXPECT_EQ(std::distance(rel.begin(), rel.end()), 5);
}

TEST_F(CsvWriterTest, FileRotatesWhenSizeLimitReached)
{
  // header "a,b\n" = 4 bytes; row "X,Y\n" = 4 bytes.
  // After the first write the file has 8 bytes on disk (header + row1).
  // max_bytes = 5 means the second write sees 8 >= 5 and rotates.
  const std::size_t max_bytes = 5;
  const std::string header = "a,b";
  CsvWriter writer(root_.string(), "rot", header, max_bytes);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1,2");
  writer.write_row(now, "3,4");
  writer.write_row(now, "5,6");

  // After rotation there should be more than one file
  EXPECT_GT(csv_files().size(), 1u);
}

TEST_F(CsvWriterTest, EachRotatedFileStartsWithHeader)
{
  const std::size_t max_bytes = 5;
  const std::string header = "a,b";
  CsvWriter writer(root_.string(), "rot", header, max_bytes);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1,2");
  writer.write_row(now, "3,4");
  writer.write_row(now, "5,6");

  for (const auto & f : csv_files()) {
    const auto lines = read_lines(f);
    ASSERT_FALSE(lines.empty()) << "Empty file: " << f;
    EXPECT_EQ(lines[0], header) << "Missing header in: " << f;
  }
}

TEST_F(CsvWriterTest, MultipleRowsWrittenCorrectly)
{
  const std::string header = "t,v";
  CsvWriter writer(root_.string(), "multi", header, 100 * 1024 * 1024);

  const double now = static_cast<double>(std::time(nullptr));
  writer.write_row(now, "1.0,a");
  writer.write_row(now, "2.0,b");
  writer.write_row(now, "3.0,c");

  const auto files = csv_files();
  ASSERT_EQ(files.size(), 1u);

  const auto lines = read_lines(files[0]);
  // header + 3 data rows
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[1], "1.0,a");
  EXPECT_EQ(lines[2], "2.0,b");
  EXPECT_EQ(lines[3], "3.0,c");
}
