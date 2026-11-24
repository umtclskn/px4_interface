#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "px4_interface/path_tracker.hpp"

namespace
{

using px4_interface::LookaheadResult;
using px4_interface::PathTracker;
using px4_interface::Vec3;
using px4_interface::distance;

constexpr double kEps = 1e-6;

}  // namespace

TEST(PathTrackerTest, StraightLookahead)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{10, 0, 0}});
  ASSERT_TRUE(tracker.hasPath());
  const LookaheadResult r = tracker.update(Vec3{1.0, 0.1, 0.0}, 2.0);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.target.x, 3.0, 0.05);
  EXPECT_NEAR(r.target.y, 0.0, kEps);
  EXPECT_FALSE(r.at_end);
  EXPECT_GT(r.remaining_length_m, 0.0);
  EXPECT_NEAR(r.cross_track_error_m, 0.1, 0.05);
}

TEST(PathTrackerTest, CornerPathTurns)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{5, 0, 0}, Vec3{5, 5, 0}});
  // Near the corner with large lookahead → target past the corner.
  LookaheadResult r = tracker.update(Vec3{4.5, 0.0, 0.0}, 2.0);
  ASSERT_TRUE(r.valid);
  EXPECT_GT(r.target.y, 0.5);
  EXPECT_NEAR(r.target.x, 5.0, 0.2);
}

TEST(PathTrackerTest, ClampsAtEnd)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{2, 0, 0}});
  LookaheadResult r = tracker.update(Vec3{1.9, 0.0, 0.0}, 5.0);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.at_end);
  EXPECT_NEAR(r.target.x, 2.0, kEps);
}

TEST(PathTrackerTest, ProgressMonotonicWhenMovedBackwards)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{10, 0, 0}});
  LookaheadResult a = tracker.update(Vec3{4.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(a.valid);
  const double target_x_a = a.target.x;
  LookaheadResult b = tracker.update(Vec3{1.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(b.valid);
  EXPECT_GE(b.target.x, target_x_a - kEps);
}

TEST(PathTrackerTest, DuplicateConsecutivePoints)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{0, 0, 0}, Vec3{4, 0, 0}});
  LookaheadResult r = tracker.update(Vec3{0.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.target.x, 1.0, 0.1);
}

TEST(PathTrackerTest, SinglePoint)
{
  PathTracker tracker;
  tracker.setPath({Vec3{3, 4, 5}});
  LookaheadResult r = tracker.update(Vec3{0, 0, 0}, 1.0);
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.at_end);
  EXPECT_NEAR(r.target.x, 3.0, kEps);
  EXPECT_NEAR(r.target.y, 4.0, kEps);
  EXPECT_NEAR(r.target.z, 5.0, kEps);
  EXPECT_NEAR(r.remaining_length_m, 0.0, kEps);
}

TEST(PathTrackerTest, EmptyPath)
{
  PathTracker tracker;
  tracker.setPath({});
  EXPECT_FALSE(tracker.hasPath());
  LookaheadResult r = tracker.update(Vec3{0, 0, 0}, 1.0);
  EXPECT_FALSE(r.valid);
}

TEST(PathTrackerTest, InvalidInputsLeaveProgress)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{5, 0, 0}});
  ASSERT_TRUE(tracker.update(Vec3{1, 0, 0}, 1.0).valid);
  LookaheadResult bad = tracker.update(
    Vec3{std::numeric_limits<double>::quiet_NaN(), 0, 0}, 1.0);
  EXPECT_FALSE(bad.valid);
  LookaheadResult zero_la = tracker.update(Vec3{1, 0, 0}, 0.0);
  EXPECT_FALSE(zero_la.valid);
  LookaheadResult again = tracker.update(Vec3{1, 0, 0}, 1.0);
  ASSERT_TRUE(again.valid);
  EXPECT_GE(again.target.x, 2.0 - 0.1);
}

TEST(PathTrackerTest, ClearResets)
{
  PathTracker tracker;
  tracker.setPath({Vec3{0, 0, 0}, Vec3{1, 0, 0}});
  tracker.clear();
  EXPECT_FALSE(tracker.hasPath());
}
