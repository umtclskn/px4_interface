#pragma once

#include <cstddef>
#include <vector>

#include "px4_interface/frame_conversions.hpp"

namespace px4_interface
{

struct LookaheadResult
{
  bool valid{false};
  Vec3 target;
  double remaining_length_m{0.0};
  double cross_track_error_m{0.0};
  bool at_end{false};
};

/// Follows a polyline with monotonic progress.
class PathTracker
{
public:
  /// Replaces the path and resets progress. An empty path clears the tracker.
  void setPath(const std::vector<Vec3> & path);
  bool hasPath() const noexcept;
  /// Projects `position` onto the path at or after the current progress,
  /// advances progress, and returns the point `lookahead_m` further along,
  /// clamped to the last point.
  LookaheadResult update(const Vec3 & position, double lookahead_m);
  void clear() noexcept;

private:
  std::vector<Vec3> path_;
  /// Cumulative arc length at each vertex (size == path_.size()).
  std::vector<double> cumulative_;
  /// Progress along the path in meters from the first vertex.
  double progress_m_{0.0};
};

}  // namespace px4_interface
