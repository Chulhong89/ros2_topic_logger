#include <gtest/gtest.h>
#include "ros2_topic_logger/event_buffer.hpp"
#include "sensor_msgs/msg/imu.hpp"

using ros2_topic_logger::EventBuffer;
using sensor_msgs::msg::Imu;

TEST(EventBufferTest, EmptyBufferSnapshotIsEmpty)
{
  EventBuffer<Imu> buf;
  EXPECT_TRUE(buf.snapshot().empty());
}

TEST(EventBufferTest, PushedMessagesAppearInSnapshot)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);
  buf.push(2.0, msg);

  auto snap = buf.snapshot();
  ASSERT_EQ(snap.size(), 2u);
  EXPECT_DOUBLE_EQ(snap[0].first, 1.0);
  EXPECT_DOUBLE_EQ(snap[1].first, 2.0);
}

TEST(EventBufferTest, PruneRemovesOlderEntries)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);
  buf.push(2.0, msg);
  buf.push(3.0, msg);

  // keep entries with recv_time >= 2.0
  buf.prune_older_than(2.0);

  auto snap = buf.snapshot();
  ASSERT_EQ(snap.size(), 2u);
  EXPECT_DOUBLE_EQ(snap[0].first, 2.0);
  EXPECT_DOUBLE_EQ(snap[1].first, 3.0);
}

TEST(EventBufferTest, PruneAllLeavesEmptyBuffer)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);
  buf.push(2.0, msg);

  buf.prune_older_than(999.0);
  EXPECT_TRUE(buf.snapshot().empty());
}

TEST(EventBufferTest, PruneNoneKeepsAll)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);
  buf.push(2.0, msg);

  buf.prune_older_than(0.0);
  EXPECT_EQ(buf.snapshot().size(), 2u);
}

TEST(EventBufferTest, ClearEmptiesBuffer)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);
  buf.push(2.0, msg);

  buf.clear();
  EXPECT_TRUE(buf.snapshot().empty());
}

TEST(EventBufferTest, SnapshotIsACopy)
{
  EventBuffer<Imu> buf;
  Imu msg;
  buf.push(1.0, msg);

  auto snap1 = buf.snapshot();
  buf.push(2.0, msg);
  auto snap2 = buf.snapshot();

  // snap1 was taken before the second push, so it must still have 1 element
  EXPECT_EQ(snap1.size(), 1u);
  EXPECT_EQ(snap2.size(), 2u);
}
