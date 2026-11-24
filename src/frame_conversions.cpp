#include "px4_interface/frame_conversions.hpp"

#include <cmath>

namespace px4_interface
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Vec3 rotateZ(const Vec3 & v, const double yaw_rad) noexcept
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Vec3 out;
  out.x = c * v.x - s * v.y;
  out.y = s * v.x + c * v.y;
  out.z = v.z;
  return out;
}

}  // namespace

bool isFinite(const Vec3 & v) noexcept
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double norm(const Vec3 & v) noexcept
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double distance(const Vec3 & a, const Vec3 & b) noexcept
{
  return norm(Vec3{a.x - b.x, a.y - b.y, a.z - b.z});
}

Vec3 enuToNed(const Vec3 & enu) noexcept
{
  return Vec3{enu.y, enu.x, -enu.z};
}

Vec3 nedToEnu(const Vec3 & ned) noexcept
{
  return Vec3{ned.y, ned.x, -ned.z};
}

double wrapPi(double angle_rad) noexcept
{
  if (!std::isfinite(angle_rad)) {
    return angle_rad;
  }
  // Map to (-pi, pi].
  angle_rad = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (angle_rad <= 0.0) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad - kPi;
}

double yawEnuToNed(const double yaw_enu_rad) noexcept
{
  return wrapPi(kPi / 2.0 - yaw_enu_rad);
}

double yawNedToEnu(const double yaw_ned_rad) noexcept
{
  return wrapPi(kPi / 2.0 - yaw_ned_rad);
}

std::optional<Vec3> worldToNed(
  const Vec3 & world, const WorldNedAlignment & a)
{
  if (!a.valid || !isFinite(world) || !isFinite(a.world_home) ||
    !isFinite(a.ned_home) || !std::isfinite(a.world_yaw_offset_rad))
  {
    return std::nullopt;
  }
  Vec3 delta;
  delta.x = world.x - a.world_home.x;
  delta.y = world.y - a.world_home.y;
  delta.z = world.z - a.world_home.z;
  const Vec3 rotated = rotateZ(delta, -a.world_yaw_offset_rad);
  const Vec3 ned_delta = enuToNed(rotated);
  return Vec3{
    ned_delta.x + a.ned_home.x,
    ned_delta.y + a.ned_home.y,
    ned_delta.z + a.ned_home.z};
}

std::optional<Vec3> nedToWorld(
  const Vec3 & ned, const WorldNedAlignment & a)
{
  if (!a.valid || !isFinite(ned) || !isFinite(a.world_home) ||
    !isFinite(a.ned_home) || !std::isfinite(a.world_yaw_offset_rad))
  {
    return std::nullopt;
  }
  Vec3 ned_delta;
  ned_delta.x = ned.x - a.ned_home.x;
  ned_delta.y = ned.y - a.ned_home.y;
  ned_delta.z = ned.z - a.ned_home.z;
  const Vec3 enu_delta = nedToEnu(ned_delta);
  const Vec3 rotated = rotateZ(enu_delta, a.world_yaw_offset_rad);
  return Vec3{
    rotated.x + a.world_home.x,
    rotated.y + a.world_home.y,
    rotated.z + a.world_home.z};
}

PathRejection validatePathNed(
  const std::vector<Vec3> & path_ned,
  const Vec3 & vehicle_ned,
  const Vec3 & home_ned,
  const PathLimits & limits)
{
  if (path_ned.empty()) {
    return PathRejection::kEmpty;
  }
  if (path_ned.size() > limits.max_points) {
    return PathRejection::kTooManyPoints;
  }
  if (!isFinite(vehicle_ned) || !isFinite(home_ned)) {
    return PathRejection::kNonFinite;
  }
  for (const Vec3 & p : path_ned) {
    if (!isFinite(p)) {
      return PathRejection::kNonFinite;
    }
  }
  if (distance(path_ned.front(), vehicle_ned) > limits.max_start_distance_m) {
    return PathRejection::kStartTooFar;
  }
  for (const Vec3 & p : path_ned) {
    const double dx = p.x - home_ned.x;
    const double dy = p.y - home_ned.y;
    const double horiz = std::sqrt(dx * dx + dy * dy);
    if (horiz > limits.max_horizontal_distance_m) {
      return PathRejection::kOutsideGeofence;
    }
    // NED z negative up: altitude above home = -(p.z - home.z) = home.z - p.z
    const double altitude_m = home_ned.z - p.z;
    if (altitude_m < limits.min_altitude_m ||
      altitude_m > limits.max_altitude_m)
    {
      return PathRejection::kOutsideGeofence;
    }
  }
  return PathRejection::kNone;
}

}  // namespace px4_interface
