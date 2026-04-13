#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "ros2_topic_logger/retention_manager.hpp"

namespace fs = std::filesystem;
using ros2_topic_logger::RetentionManager;

// ── helpers ───────────────────────────────────────────────────────────────────

static fs::path create_file(const fs::path & dir, const std::string & name,
  std::size_t size_bytes)
{
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream f(p, std::ios::binary);
  f << std::string(size_bytes, 'x');
  return p;
}

static void set_age(const fs::path & p, int days_old)
{
  const auto old_time =
    fs::file_time_type::clock::now() - std::chrono::hours(24 * days_old);
  fs::last_write_time(p, old_time);
}

static std::uintmax_t total_size(const fs::path & root)
{
  if (!fs::exists(root)) {
    return 0;
  }
  std::uintmax_t s = 0;
  for (const auto & e : fs::recursive_directory_iterator(root)) {
    if (e.is_regular_file()) {
      s += e.file_size();
    }
  }
  return s;
}

// ── fixture ───────────────────────────────────────────────────────────────────

class RetentionManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / "ros2_logger_retention_test";
    fs::remove_all(root_);
    fs::create_directories(root_);
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  fs::path root_;
};

// ── tests ─────────────────────────────────────────────────────────────────────

TEST_F(RetentionManagerTest, EnforceOnEmptyDirectoryDoesNotCrash)
{
  RetentionManager rm(root_.string(), 1024ull * 1024 * 1024, 7);
  EXPECT_NO_THROW(rm.enforce());
}

TEST_F(RetentionManagerTest, EnforceOnNonExistentDirectoryDoesNotCrash)
{
  RetentionManager rm("/tmp/does_not_exist_ros2_logger_xyz", 1024ull * 1024 * 1024, 7);
  EXPECT_NO_THROW(rm.enforce());
}

TEST_F(RetentionManagerTest, RecentFilesAreNotDeleted)
{
  create_file(root_ / "subdir", "recent.csv", 100);

  RetentionManager rm(root_.string(), 1024ull * 1024 * 1024, 7);
  rm.enforce();

  EXPECT_TRUE(fs::exists(root_ / "subdir" / "recent.csv"));
}

TEST_F(RetentionManagerTest, OldFilesAreDeletedByAge)
{
  const auto old_file = create_file(root_ / "subdir", "old.csv", 100);
  set_age(old_file, 8);  // 8 days old, max_age_days = 7

  RetentionManager rm(root_.string(), 1024ull * 1024 * 1024, 7);
  rm.enforce();

  EXPECT_FALSE(fs::exists(old_file));
}

TEST_F(RetentionManagerTest, FilesWithinAgeAreKept)
{
  const auto young_file = create_file(root_ / "subdir", "young.csv", 100);
  set_age(young_file, 3);  // 3 days old, max_age_days = 7

  RetentionManager rm(root_.string(), 1024ull * 1024 * 1024, 7);
  rm.enforce();

  EXPECT_TRUE(fs::exists(young_file));
}

TEST_F(RetentionManagerTest, SizeEnforcementDeletesOldestFirst)
{
  // Create two files: older one and newer one; total exceeds limit
  const auto older = create_file(root_ / "subdir", "older.csv", 600);
  const auto newer = create_file(root_ / "subdir", "newer.csv", 600);
  set_age(older, 2);
  set_age(newer, 1);

  // Limit is 800 bytes — after enforcement the oldest file should be gone
  RetentionManager rm(root_.string(), 800, 30);
  rm.enforce();

  EXPECT_FALSE(fs::exists(older));
  // Total size should now be within the limit
  EXPECT_LE(total_size(root_), static_cast<std::uintmax_t>(800));
}

TEST_F(RetentionManagerTest, SizeEnforcementDoesNothingIfUnderLimit)
{
  const auto f1 = create_file(root_ / "subdir", "a.csv", 100);
  const auto f2 = create_file(root_ / "subdir", "b.csv", 100);

  RetentionManager rm(root_.string(), 1024ull * 1024, 7);
  rm.enforce();

  EXPECT_TRUE(fs::exists(f1));
  EXPECT_TRUE(fs::exists(f2));
}

TEST_F(RetentionManagerTest, ZeroMaxAgeDaysDisablesAgeDeletion)
{
  const auto old_file = create_file(root_ / "subdir", "old.csv", 100);
  set_age(old_file, 100);

  // max_age_days = 0 disables age-based deletion
  RetentionManager rm(root_.string(), 1024ull * 1024 * 1024, 0);
  rm.enforce();

  EXPECT_TRUE(fs::exists(old_file));
}
