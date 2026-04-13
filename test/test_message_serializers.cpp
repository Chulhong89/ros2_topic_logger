#include <gtest/gtest.h>
#include <sstream>
#include <string>

#include "ros2_topic_logger/message_serializers.hpp"

using namespace ros2_topic_logger;

// ── helpers ──────────────────────────────────────────────────────────────────

static std::size_t count_commas(const std::string & s)
{
  return static_cast<std::size_t>(std::count(s.begin(), s.end(), ','));
}

static std::size_t header_column_count(const std::string & header)
{
  return count_commas(header) + 1;
}

static std::size_t row_column_count(const std::string & row)
{
  // Quoted fields may contain commas; count only top-level commas.
  std::size_t cols = 1;
  bool in_quotes = false;
  for (char c : row) {
    if (c == '"') {
      in_quotes = !in_quotes;
    } else if (c == ',' && !in_quotes) {
      ++cols;
    }
  }
  return cols;
}

// ── stamp_to_sec ─────────────────────────────────────────────────────────────

TEST(StampToSecTest, ZeroStamp)
{
  builtin_interfaces::msg::Time t;
  t.sec = 0;
  t.nanosec = 0;
  EXPECT_DOUBLE_EQ(stamp_to_sec(t), 0.0);
}

TEST(StampToSecTest, OneSecond)
{
  builtin_interfaces::msg::Time t;
  t.sec = 1;
  t.nanosec = 0;
  EXPECT_DOUBLE_EQ(stamp_to_sec(t), 1.0);
}

TEST(StampToSecTest, NanosecPart)
{
  builtin_interfaces::msg::Time t;
  t.sec = 0;
  t.nanosec = 500000000;  // 0.5 s
  EXPECT_NEAR(stamp_to_sec(t), 0.5, 1e-9);
}

// ── join_vector ───────────────────────────────────────────────────────────────

TEST(JoinVectorTest, EmptyVector)
{
  EXPECT_EQ(join_vector({}), "");
}

TEST(JoinVectorTest, SingleElement)
{
  EXPECT_EQ(join_vector({1.5f}), "1.5");
}

TEST(JoinVectorTest, MultipleElementsSemicolonSeparated)
{
  const std::string result = join_vector({1.0f, 2.0f, 3.0f});
  EXPECT_EQ(result, "1;2;3");
}

// ── IMU serializer ────────────────────────────────────────────────────────────

TEST(ImuSerializerTest, HeaderColumnCount)
{
  const auto hdr = MessageSerializer<sensor_msgs::msg::Imu>::header();
  EXPECT_EQ(header_column_count(hdr), 13u);
}

TEST(ImuSerializerTest, RowColumnCountMatchesHeader)
{
  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = "imu_link";
  msg.orientation.w = 1.0;
  const auto hdr = MessageSerializer<sensor_msgs::msg::Imu>::header();
  const auto row = MessageSerializer<sensor_msgs::msg::Imu>::row(msg, 100.0);
  EXPECT_EQ(row_column_count(row), header_column_count(hdr));
}

TEST(ImuSerializerTest, RowContainsRecvTime)
{
  sensor_msgs::msg::Imu msg;
  const auto row = MessageSerializer<sensor_msgs::msg::Imu>::row(msg, 123.456);
  EXPECT_NE(row.find("123.456"), std::string::npos);
}

// ── Odometry serializer ───────────────────────────────────────────────────────

TEST(OdometrySerializerTest, HeaderColumnCount)
{
  const auto hdr = MessageSerializer<nav_msgs::msg::Odometry>::header();
  EXPECT_EQ(header_column_count(hdr), 17u);
}

TEST(OdometrySerializerTest, RowColumnCountMatchesHeader)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  const auto hdr = MessageSerializer<nav_msgs::msg::Odometry>::header();
  const auto row = MessageSerializer<nav_msgs::msg::Odometry>::row(msg, 1.0);
  EXPECT_EQ(row_column_count(row), header_column_count(hdr));
}

// ── PoseStamped serializer ────────────────────────────────────────────────────

TEST(PoseStampedSerializerTest, HeaderColumnCount)
{
  const auto hdr = MessageSerializer<geometry_msgs::msg::PoseStamped>::header();
  EXPECT_EQ(header_column_count(hdr), 10u);
}

TEST(PoseStampedSerializerTest, RowColumnCountMatchesHeader)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  const auto hdr = MessageSerializer<geometry_msgs::msg::PoseStamped>::header();
  const auto row = MessageSerializer<geometry_msgs::msg::PoseStamped>::row(msg, 1.0);
  EXPECT_EQ(row_column_count(row), header_column_count(hdr));
}

// ── LaserScan serializer ──────────────────────────────────────────────────────

TEST(LaserScanSerializerTest, HeaderColumnCount)
{
  const auto hdr = MessageSerializer<sensor_msgs::msg::LaserScan>::header();
  EXPECT_EQ(header_column_count(hdr), 12u);
}

TEST(LaserScanSerializerTest, RowColumnCountMatchesHeader)
{
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = "laser";
  msg.ranges = {1.0f, 2.0f};
  const auto hdr = MessageSerializer<sensor_msgs::msg::LaserScan>::header();
  const auto row = MessageSerializer<sensor_msgs::msg::LaserScan>::row(msg, 1.0);
  EXPECT_EQ(row_column_count(row), header_column_count(hdr));
}

TEST(LaserScanSerializerTest, EventTriggerFalseWhenRangeAboveThreshold)
{
  sensor_msgs::msg::LaserScan msg;
  msg.range_min = 0.1f;
  msg.range_max = 10.0f;
  msg.ranges = {5.0f, 6.0f};

  TopicConfig cfg;
  cfg.trigger_distance_threshold = 1.0;

  EXPECT_FALSE(
    MessageSerializer<sensor_msgs::msg::LaserScan>::event_trigger(msg, cfg));
}

TEST(LaserScanSerializerTest, EventTriggerTrueWhenRangeBelowThreshold)
{
  sensor_msgs::msg::LaserScan msg;
  msg.range_min = 0.1f;
  msg.range_max = 10.0f;
  msg.ranges = {0.5f, 6.0f};

  TopicConfig cfg;
  cfg.trigger_distance_threshold = 1.0;

  EXPECT_TRUE(
    MessageSerializer<sensor_msgs::msg::LaserScan>::event_trigger(msg, cfg));
}

TEST(LaserScanSerializerTest, EventTriggerFalseWhenAllRangesInvalid)
{
  sensor_msgs::msg::LaserScan msg;
  msg.range_min = 0.1f;
  msg.range_max = 10.0f;
  msg.ranges = {std::numeric_limits<float>::infinity()};

  TopicConfig cfg;
  cfg.trigger_distance_threshold = 1.0;

  EXPECT_FALSE(
    MessageSerializer<sensor_msgs::msg::LaserScan>::event_trigger(msg, cfg));
}
