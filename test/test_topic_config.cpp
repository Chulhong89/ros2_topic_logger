#include <gtest/gtest.h>
#include "ros2_topic_logger/topic_config.hpp"

using ros2_topic_logger::sanitize_topic_name;

TEST(TopicConfigTest, LeadingSlashStripped)
{
  EXPECT_EQ(sanitize_topic_name("/odom"), "odom");
}

TEST(TopicConfigTest, NestedTopicUsesUnderscoreSeparator)
{
  EXPECT_EQ(sanitize_topic_name("/imu/data"), "imu_data");
  EXPECT_EQ(sanitize_topic_name("/a/b/c"), "a_b_c");
}

TEST(TopicConfigTest, LeadingAndTrailingUnderscoresRemoved)
{
  EXPECT_EQ(sanitize_topic_name("/_hidden"), "hidden");
  EXPECT_EQ(sanitize_topic_name("/topic_"), "topic");
}

TEST(TopicConfigTest, ConsecutiveSlashesProduceSingleUnderscore)
{
  // double slash should not produce double underscore
  const std::string result = sanitize_topic_name("//double");
  EXPECT_EQ(result.find("__"), std::string::npos);
}

TEST(TopicConfigTest, SpecialCharactersReplacedByUnderscore)
{
  EXPECT_EQ(sanitize_topic_name("/a-b"), "a_b");
  EXPECT_EQ(sanitize_topic_name("/a.b"), "a_b");
}

TEST(TopicConfigTest, EmptyStringReturnsFallback)
{
  EXPECT_EQ(sanitize_topic_name(""), "topic");
}

TEST(TopicConfigTest, AlphanumericOnlyPassesThrough)
{
  EXPECT_EQ(sanitize_topic_name("myTopic123"), "myTopic123");
}
