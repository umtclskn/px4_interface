#include "px4_interface/path_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace px4_interface
{
namespace
{

Vec3 interpolate(const Vec3 & a, const Vec3 & b, const double t) noexcept
{
  Vec3 out;
  out.x = a.x + t * (b.x - a.x);
  out.y = a.y + t * (b.y - a.y);
  out.z = a.z + t * (b.z - a.z);
  return out;
}

/// Closest point on segment a→b to p; returns (point, arc fraction in [0,1]).
void projectOnSegment(
  const Vec3 & a, const Vec3 & b, const Vec3 & p,
  Vec3 & closest, double & t) noexcept
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double len2 = dx * dx + dy * dy + dz * dz;
  if (len2 <= 0.0) {
    closest = a;
    t = 0.0;
    return;
  }
  const double px = p.x - a.x;
  const double py = p.y - a.y;
  const double pz = p.z - a.z;
  t = (px * dx + py * dy + pz * dz) / len2;
  if (t < 0.0) {
    t = 0.0;
  } else if (t > 1.0) {
    t = 1.0;
  }
  closest = interpolate(a, b, t);
}

}  // namespace

void PathTracker::setPath(const std::vector<Vec3> & path)
{
  path_ = path;
  cumulative_.clear();
  progress_m_ = 0.0;
  if (path_.empty()) {
    return;
  }
  cumulative_.resize(path_.size(), 0.0);
  for (std::size_t i = 1; i < path_.size(); ++i) {
    const double seg = distance(path_[i - 1u], path_[i]);
    cumulative_[i] = cumulative_[i - 1u] + seg;
  }
}

bool PathTracker::hasPath() const noexcept
{
  return !path_.empty();
}

void PathTracker::clear() noexcept
{
  path_.clear();
  cumulative_.clear();
  progress_m_ = 0.0;
}

LookaheadResult PathTracker::update(
  const Vec3 & position, const double lookahead_m)
{
  LookaheadResult result;
  if (path_.empty() || !isFinite(position) ||
    !std::isfinite(lookahead_m) || !(lookahead_m > 0.0))
  {
    return result;
  }

  if (path_.size() == 1u) {
    result.valid = true;
    result.target = path_.front();
    result.remaining_length_m = 0.0;
    result.cross_track_error_m = distance(position, path_.front());
    result.at_end = true;
    progress_m_ = 0.0;
    return result;
  }

  const double total = cumulative_.back();
  // Find projection at or after current progress.
  double best_s = progress_m_;
  Vec3 best_point = path_.front();
  double best_cte = distance(position, path_.front());
  bool found = false;

  for (std::size_t i = 0; i + 1u < path_.size(); ++i) {
    const double s0 = cumulative_[i];
    const double s1 = cumulative_[i + 1u];
    if (s1 < progress_m_ - 1e-12) {
      continue;
    }
    const double seg_len = s1 - s0;
    if (seg_len <= 0.0) {
      continue;
    }
    Vec3 closest;
    double t = 0.0;
    projectOnSegment(path_[i], path_[i + 1u], position, closest, t);
    double s = s0 + t * seg_len;
    if (s < progress_m_) {
      // Clamp to current progress on this segment (monotonic).
      const double t_prog = (progress_m_ - s0) / seg_len;
      if (t_prog >= 0.0 && t_prog <= 1.0) {
        closest = interpolate(path_[i], path_[i + 1u], t_prog);
        s = progress_m_;
      } else {
        continue;
      }
    }
    const double cte = distance(position, closest);
    if (!found || cte < best_cte - 1e-12 ||
      (std::abs(cte - best_cte) <= 1e-12 && s < best_s))
    {
      found = true;
      best_cte = cte;
      best_s = s;
      best_point = closest;
    }
  }

  if (!found) {
    best_s = total;
    best_point = path_.back();
    best_cte = distance(position, best_point);
  }

  progress_m_ = std::max(progress_m_, best_s);
  if (progress_m_ > total) {
    progress_m_ = total;
  }

  const double target_s = std::min(progress_m_ + lookahead_m, total);
  Vec3 target = path_.back();
  for (std::size_t i = 0; i + 1u < path_.size(); ++i) {
    const double s0 = cumulative_[i];
    const double s1 = cumulative_[i + 1u];
    if (target_s <= s1 || i + 2u == path_.size()) {
      const double seg_len = s1 - s0;
      if (seg_len <= 0.0) {
        target = path_[i + 1u];
      } else {
        const double t = (target_s - s0) / seg_len;
        target = interpolate(path_[i], path_[i + 1u], std::clamp(t, 0.0, 1.0));
      }
      break;
    }
  }

  result.valid = true;
  result.target = target;
  result.remaining_length_m = total - progress_m_;
  result.cross_track_error_m = best_cte;
  result.at_end = (progress_m_ >= total - 1e-9) ||
    (target_s >= total - 1e-9 && lookahead_m > 0.0 &&
    distance(target, path_.back()) <= 1e-9);
  (void)best_point;
  return result;
}

}  // namespace px4_interface
