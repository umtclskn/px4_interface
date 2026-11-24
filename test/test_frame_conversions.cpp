#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "px4_interface/frame_conversions.hpp"

namespace
{

using px4_interface::PathLimits;
using px4_interface::PathRejection;
using px4_interface::Vec3;
using px4_interface::WorldNedAlignment;
using px4_interface::distance;
using px4_interface::enuToNed;
using px4_interface::isFinite;
using px4_interface::nedToEnu;
using px4_interface::nedToWorld;
using px4_interface::validatePathNed;
using px4_interface::wrapPi;
using px4_interface::worldToNed;
using px4_interface::yawEnuToNed;
using px4_interface::yawNedToEnu;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

}  // namespace

TEST(FrameConversionsTest, EnuToNedUnitAxes)
{
  const Vec3 east = enuToNed(Vec3{1.0, 0.0, 0.0});
  EXPECT_NEAR(east.x, 0.0, kEps);
  EXPECT_NEAR(east.y, 1.0, kEps);
  EXPECT_NEAR(east.z, 0.0, kEps);

  const Vec3 north = enuToNed(Vec3{0.0, 1.0, 0.0});
  EXPECT_NEAR(north.x, 1.0, kEps);
  EXPECT_NEAR(north.y, 0.0, kEps);
  EXPECT_NEAR(north.z, 0.0, kEps);

  const Vec3 up = enuToNed(Vec3{0.0, 0.0, 1.0});
  EXPECT_NEAR(up.x, 0.0, kEps);
  EXPECT_NEAR(up.y, 0.0, kEps);
  EXPECT_NEAR(up.z, -1.0, kEps);
}

TEST(FrameConversionsTest, EnuNedRoundTrip)
{
  const Vec3 v{1.2, -3.4, 5.6};
  const Vec3 back = nedToEnu(enuToNed(v));
  EXPECT_NEAR(back.x, v.x, kEps);
  EXPECT_NEAR(back.y, v.y, kEps);
  EXPECT_NEAR(back.z, v.z, kEps);
}

TEST(FrameConversionsTest, YawRoundTrip)
{
  for (double yaw : {-kPi, -2.0, -0.5, 0.0, 0.5, 2.0, kPi}) {
    EXPECT_NEAR(yawNedToEnu(yawEnuToNed(yaw)), wrapPi(yaw), 1e-9);
  }
}

TEST(FrameConversionsTest, WrapPiBoundaries)
{
  EXPECT_NEAR(wrapPi(kPi), kPi, kEps);
  EXPECT_NEAR(wrapPi(-kPi), kPi, kEps);
  EXPECT_NEAR(wrapPi(3.0 * kPi), kPi, kEps);
  EXPECT_NEAR(wrapPi(-3.0 * kPi), kPi, kEps);
  EXPECT_NEAR(wrapPi(0.0), 0.0, kEps);
}

TEST(FrameConversionsTest, WorldToNedOriginOffsetOnly)
{
  WorldNedAlignment a;
  a.world_home = Vec3{10.0, 20.0, 0.0};
  a.ned_home = Vec3{1.0, 2.0, -0.5};
  a.world_yaw_offset_rad = 0.0;
  a.valid = true;

  const auto ned = worldToNed(Vec3{10.0, 21.0, 1.0}, a);
  ASSERT_TRUE(ned.has_value());
  // ENU delta (0, 1, 1) -> NED (1, 0, -1) + home
  EXPECT_NEAR(ned->x, 2.0, kEps);
  EXPECT_NEAR(ned->y, 2.0, kEps);
  EXPECT_NEAR(ned->z, -1.5, kEps);

  const auto world = nedToWorld(*ned, a);
  ASSERT_TRUE(world.has_value());
  EXPECT_NEAR(world->x, 10.0, kEps);
  EXPECT_NEAR(world->y, 21.0, kEps);
  EXPECT_NEAR(world->z, 1.0, kEps);
}

TEST(FrameConversionsTest, WorldToNedYawOffset90)
{
  WorldNedAlignment a;
  a.world_home = Vec3{0.0, 0.0, 0.0};
  a.ned_home = Vec3{0.0, 0.0, 0.0};
  a.world_yaw_offset_rad = kPi / 2.0;
  a.valid = true;

  // World +x, rotate by -90° about z -> (0, -1, 0) ENU -> NED (-1, 0, 0)
  const auto ned = worldToNed(Vec3{1.0, 0.0, 0.0}, a);
  ASSERT_TRUE(ned.has_value());
  EXPECT_NEAR(ned->x, -1.0, kEps);
  EXPECT_NEAR(ned->y, 0.0, kEps);
  EXPECT_NEAR(ned->z, 0.0, kEps);

  const auto world = nedToWorld(*ned, a);
  ASSERT_TRUE(world.has_value());
  EXPECT_NEAR(world->x, 1.0, kEps);
  EXPECT_NEAR(world->y, 0.0, kEps);
  EXPECT_NEAR(world->z, 0.0, kEps);
}

TEST(FrameConversionsTest, OneMetreAboveHomeIsMoreNegativeNedZ)
{
  WorldNedAlignment a;
  a.world_home = Vec3{0.0, 0.0, 1.0};
  a.ned_home = Vec3{0.0, 0.0, -1.0};
  a.valid = true;
  const auto ned = worldToNed(Vec3{0.0, 0.0, 2.0}, a);
  ASSERT_TRUE(ned.has_value());
  EXPECT_NEAR(ned->z, -2.0, kEps);
}

TEST(FrameConversionsTest, InvalidAlignmentAndNanReturnNullopt)
{
  WorldNedAlignment a;
  a.valid = false;
  EXPECT_FALSE(worldToNed(Vec3{0.0, 0.0, 0.0}, a).has_value());
  a.valid = true;
  a.world_home = Vec3{0.0, 0.0, 0.0};
  a.ned_home = Vec3{0.0, 0.0, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(worldToNed(Vec3{nan, 0.0, 0.0}, a).has_value());
  EXPECT_FALSE(nedToWorld(Vec3{0.0, nan, 0.0}, a).has_value());
}

TEST(FrameConversionsTest, IsFiniteAndDistance)
{
  EXPECT_TRUE(isFinite(Vec3{1.0, 2.0, 3.0}));
  EXPECT_FALSE(isFinite(Vec3{std::numeric_limits<double>::infinity(), 0, 0}));
  EXPECT_NEAR(distance(Vec3{0, 0, 0}, Vec3{3, 4, 0}), 5.0, kEps);
}

TEST(FrameConversionsTest, ValidatePathRejectionOrder)
{
  PathLimits limits;
  limits.max_points = 3;
  limits.max_start_distance_m = 1.0;
  limits.max_horizontal_distance_m = 10.0;
  limits.min_altitude_m = 0.5;
  limits.max_altitude_m = 5.0;
  const Vec3 vehicle{0.0, 0.0, -1.0};
  const Vec3 home{0.0, 0.0, 0.0};

  EXPECT_EQ(
    validatePathNed({}, vehicle, home, limits),
    PathRejection::kEmpty);

  std::vector<Vec3> too_many = {
    Vec3{0, 0, -1}, Vec3{0, 0, -1}, Vec3{0, 0, -1}, Vec3{0, 0, -1}};
  EXPECT_EQ(
    validatePathNed(too_many, vehicle, home, limits),
    PathRejection::kTooManyPoints);

  std::vector<Vec3> non_finite = {
    Vec3{0, 0, -1},
    Vec3{std::numeric_limits<double>::quiet_NaN(), 0, -1}};
  EXPECT_EQ(
    validatePathNed(non_finite, vehicle, home, limits),
    PathRejection::kNonFinite);

  std::vector<Vec3> start_far = {Vec3{5.0, 0.0, -1.0}};
  EXPECT_EQ(
    validatePathNed(start_far, vehicle, home, limits),
    PathRejection::kStartTooFar);

  std::vector<Vec3> outside = {Vec3{0.0, 0.0, -1.0}, Vec3{20.0, 0.0, -1.0}};
  EXPECT_EQ(
    validatePathNed(outside, vehicle, home, limits),
    PathRejection::kOutsideGeofence);

  std::vector<Vec3> ok = {Vec3{0.0, 0.0, -1.0}, Vec3{1.0, 0.0, -1.5}};
  EXPECT_EQ(
    validatePathNed(ok, vehicle, home, limits),
    PathRejection::kNone);
}
