#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_topic_logger/topic_logger_node.hpp"

namespace fs = std::filesystem;

// ── helpers ───────────────────────────────────────────────────────────────────

static const std::string kLogRoot = "/tmp/ros2_logger_integration_test";

static std::string test_config_path()
{
  // The yaml is installed next to the test binary via CMake
  // Fall back to the source tree path if installed path not found.
  const char * env = std::getenv("TEST_CONFIG_PATH");
  if (env) {
    return std::string(env);
  }
  return std::string(TEST_CONFIG_PATH);  // injected by CMake
}

static std::vector<fs::path> csv_files_under(const fs::path & root)
{
  std::vector<fs::path> files;
  if (!fs::exists(root)) {
    return files;
  }
  for (const auto & e : fs::recursive_directory_iterator(root)) {
    if (e.is_regular_file() && e.path().extension() == ".csv") {
      files.push_back(e.path());
    }
  }
  return files;
}

// ── fixture ───────────────────────────────────────────────────────────────────

class TopicLoggerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fs::remove_all(kLogRoot);

    // use_intra_process_comms bypasses DDS so messages are delivered
    // synchronously within the same process — no discovery wait needed.
    auto options = rclcpp::NodeOptions()
      .append_parameter_override("config_path", test_config_path())
      .use_intra_process_comms(true);

    node_ = std::make_shared<ros2_topic_logger::TopicLoggerNode>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    node_.reset();
    fs::remove_all(kLogRoot);
  }

  // Spin for the given duration to let callbacks fire
  void spin_for(std::chrono::milliseconds ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + ms;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<ros2_topic_logger::TopicLoggerNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// ── tests ─────────────────────────────────────────────────────────────────────

TEST_F(TopicLoggerNodeTest, NodeInitializesWithoutError)
{
  // If node construction threw, SetUp() would have failed.
  EXPECT_NE(node_, nullptr);
}

// Publish n_pubs messages on the given topic, spinning after each publish
// so the subscriber callback runs immediately (intra-process delivery).
template<typename MsgT>
static void publish_and_spin(
  std::shared_ptr<ros2_topic_logger::TopicLoggerNode> & node,
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> & executor,
  const std::string & topic,
  const MsgT & msg,
  int n_pubs = 5)
{
  auto pub = node->create_publisher<MsgT>(topic, rclcpp::QoS(10));

  for (int i = 0; i < n_pubs; ++i) {
    pub->publish(msg);
    executor->spin_some(std::chrono::milliseconds(10));
  }
}

TEST_F(TopicLoggerNodeTest, ImuMessageIsWrittenToCsv)
{
  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = "imu_link";
  msg.orientation.w = 1.0;
  msg.linear_acceleration.x = 0.1;

  publish_and_spin(node_, executor_,"/test/imu", msg);

  const auto files = csv_files_under(kLogRoot);
  ASSERT_FALSE(files.empty()) << "No CSV files written to " << kLogRoot;

  bool found = false;
  for (const auto & f : files) {
    std::ifstream in(f);
    std::string first_line;
    if (std::getline(in, first_line) &&
        first_line.find("orientation_x") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "No CSV file with IMU header found";
}

TEST_F(TopicLoggerNodeTest, OdometryMessageIsWrittenToCsv)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = 1.0;

  publish_and_spin(node_, executor_,"/test/odom", msg);

  const auto files = csv_files_under(kLogRoot);
  bool found = false;
  for (const auto & f : files) {
    std::ifstream in(f);
    std::string first_line;
    if (std::getline(in, first_line) &&
        first_line.find("child_frame_id") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "No CSV file with Odometry header found";
}

TEST_F(TopicLoggerNodeTest, PoseStampedMessageIsWrittenToCsv)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.pose.position.x = 2.0;

  publish_and_spin(node_, executor_, "/test/pose", msg);

  const auto files = csv_files_under(kLogRoot);
  bool found = false;
  for (const auto & f : files) {
    std::ifstream in(f);
    std::string first_line;
    if (std::getline(in, first_line) &&
        first_line.find("ori_x") != std::string::npos &&
        first_line.find("child_frame_id") == std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "No CSV file with PoseStamped header found";
}

TEST_F(TopicLoggerNodeTest, LaserScanEventWindowWritesCsvOnTrigger)
{
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = "laser";
  msg.range_min = 0.1f;
  msg.range_max = 10.0f;
  msg.ranges = {0.5f};  // below trigger_distance_threshold = 1.0

  publish_and_spin(node_, executor_, "/test/scan", msg, 15);

  const auto files = csv_files_under(kLogRoot);
  bool found = false;
  for (const auto & f : files) {
    if (f.filename().string().find("event") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "No event CSV file for LaserScan found";
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
