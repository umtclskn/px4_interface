#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace px4_interface
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool isFinite(const Vec3 & v) noexcept;
double norm(const Vec3 & v) noexcept;
double distance(const Vec3 & a, const Vec3 & b) noexcept;

/// ENU vector -> NED vector: (x, y, z) -> (y, x, -z). Its own inverse.
Vec3 enuToNed(const Vec3 & enu) noexcept;
Vec3 nedToEnu(const Vec3 & ned) noexcept;

/// Wrap to (-pi, pi].
double wrapPi(double angle_rad) noexcept;
/// ENU yaw (CCW from east) -> NED yaw (CW from north): pi/2 - yaw.
double yawEnuToNed(double yaw_enu_rad) noexcept;
double yawNedToEnu(double yaw_ned_rad) noexcept;

/// One simultaneous pair of positions of the vehicle in both frames.
struct WorldNedAlignment
{
  Vec3 world_home;
  Vec3 ned_home;
  /// Extra rotation about +z from world axes to true ENU, normally 0.
  double world_yaw_offset_rad{0.0};
  bool valid{false};
};

/// nullopt when the alignment is invalid or any input is non-finite.
std::optional<Vec3> worldToNed(const Vec3 & world, const WorldNedAlignment & a);
std::optional<Vec3> nedToWorld(const Vec3 & ned, const WorldNedAlignment & a);

struct PathLimits
{
  std::size_t max_points{10000};
  /// Reject a path whose first point is farther than this from the vehicle.
  double max_start_distance_m{3.0};
  /// Reject points farther than this (horizontal) from home, or outside the
  /// altitude band relative to home (NED z is negative up).
  double max_horizontal_distance_m{50.0};
  double min_altitude_m{0.5};
  double max_altitude_m{5.0};
};

enum class PathRejection
{
  kNone,
  kEmpty,
  kTooManyPoints,
  kNonFinite,
  kStartTooFar,
  kOutsideGeofence,
};

/// Validate a path already expressed in NED.
PathRejection validatePathNed(
  const std::vector<Vec3> & path_ned,
  const Vec3 & vehicle_ned,
  const Vec3 & home_ned,
  const PathLimits & limits);

}  // namespace px4_interface
