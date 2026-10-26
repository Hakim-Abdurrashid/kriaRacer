#include "race_core/follow_gap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_core
{

namespace
{
double sanitize(float r, double max_range)
{
  if (std::isnan(r) || (std::isinf(r) && r > 0)) {return max_range;}
  if (std::isinf(r)) {return 0.0;}
  return std::clamp(static_cast<double>(r), 0.0, max_range);
}
}  // namespace

FollowGap::FollowGap(const FollowGapParams & params)
: params_(params) {}

std::vector<double> FollowGap::preprocess(const std::vector<float> & ranges) const
{
  const std::size_t n = ranges.size();
  std::vector<double> clean(n);
  for (std::size_t i = 0; i < n; ++i) {clean[i] = sanitize(ranges[i], params_.max_lidar_range);}
  if (params_.preprocess_conv_size <= 1 || n == 0) {return clean;}

  // Same window placement and zero padding as numpy.convolve(..., mode='same').
  const long w = std::min<long>(params_.preprocess_conv_size, static_cast<long>(n));
  const long lo_off = w / 2;
  const long hi_off = (w - 1) / 2;
  std::vector<double> out(n);
  for (long i = 0; i < static_cast<long>(n); ++i) {
    double sum = 0.0;
    for (long j = i - lo_off; j <= i + hi_off; ++j) {
      if (j >= 0 && j < static_cast<long>(n)) {sum += clean[j];}
    }
    out[i] = sum / static_cast<double>(w);
  }
  return out;
}

std::vector<double> FollowGap::extendDisparities(
  const std::vector<double> & processed, const std::vector<float> & raw,
  std::size_t offset, double angle_increment) const
{
  std::vector<double> extended = processed;
  const std::size_t n = processed.size();
  const double clearance = std::max(0.0, params_.vehicle_width / 2.0 + params_.disparity_margin);
  if (n < 2 || clearance == 0.0) {return extended;}

  std::vector<double> d(n);
  for (std::size_t i = 0; i < n; ++i) {d[i] = sanitize(raw[offset + i], params_.max_lidar_range);}

  for (std::size_t e = 0; e + 1 < n; ++e) {
    if (std::abs(d[e + 1] - d[e]) <= params_.disparity_threshold) {continue;}
    const bool near_is_left = d[e] < d[e + 1];
    const std::size_t near = near_is_left ? e : e + 1;
    const double near_d = d[near];
    const double half_angle = std::asin(std::min(1.0, clearance / std::max(near_d, 1e-6)));
    const std::size_t beams = std::min(
      n, static_cast<std::size_t>(std::ceil(half_angle / angle_increment)));
    const std::size_t first = near_is_left ? near : (near >= beams ? near - beams : 0);
    const std::size_t last = near_is_left ? std::min(n - 1, near + beams) : near;
    for (std::size_t k = first; k <= last; ++k) {extended[k] = std::min(extended[k], near_d);}
  }
  return extended;
}

std::optional<DriveCommand> FollowGap::compute(
  const std::vector<float> & ranges, double angle_min, double angle_increment)
{
  if (ranges.size() < 2 || !std::isfinite(angle_increment) || angle_increment <= 0.0) {
    return std::nullopt;
  }
  bool any_finite = false;
  for (float r : ranges) {
    if (std::isfinite(r)) {any_finite = true; break;}
  }
  if (!any_finite) {return std::nullopt;}

  const std::vector<double> proc = preprocess(ranges);
  const std::size_t n = proc.size();
  const std::size_t crop_start = n / 5;
  const std::size_t crop_end = 4 * n / 5;
  const std::vector<double> front(proc.begin() + crop_start, proc.begin() + crop_end);
  const std::size_t m = front.size();
  if (m == 0) {return std::nullopt;}

  // Closest point: a very close obstacle in the centre window takes priority.
  double closest_dist = std::numeric_limits<double>::infinity();
  std::size_t closest_idx = 0;
  bool have_nonzero = false;
  for (double r : front) {
    if (r > 0.0) {have_nonzero = true; break;}
  }
  if (!have_nonzero) {return std::nullopt;}

  const std::size_t mid = m / 2;
  const std::size_t c_lo = mid > static_cast<std::size_t>(params_.center_half_width) ?
    mid - params_.center_half_width : 0;
  const std::size_t c_hi = std::min(m, mid + params_.center_half_width);
  double center_dist = std::numeric_limits<double>::infinity();
  std::size_t center_idx = 0;
  for (std::size_t i = c_lo; i < c_hi; ++i) {
    if (front[i] < center_dist) {center_dist = front[i]; center_idx = i;}
  }
  if (center_dist < params_.center_priority_dist) {
    closest_dist = center_dist;
    closest_idx = center_idx;
  } else {
    for (std::size_t i = 0; i < m; ++i) {
      if (front[i] > 0.0 && front[i] < closest_dist) {closest_dist = front[i]; closest_idx = i;}
    }
  }

  std::vector<double> free_space = extendDisparities(front, ranges, crop_start, angle_increment);

  // Zero the safety bubble around the closest point.
  const double bubble_d = closest_dist <= 0.0 ? 0.1 : closest_dist;
  const long bubble = static_cast<long>(std::ceil(params_.bubble_radius / bubble_d / angle_increment));
  const long b_lo = std::max(0L, static_cast<long>(closest_idx) - bubble);
  const long b_hi = std::min(static_cast<long>(m) - 1, static_cast<long>(closest_idx) + bubble);
  for (long i = b_lo; i <= b_hi; ++i) {free_space[i] = 0.0;}

  // Longest run of free beams (first one wins ties, like numpy.argmax).
  long gap_start = -1, gap_end = -1, best_len = 0;
  for (std::size_t i = 0; i < m; ) {
    if (free_space[i] <= 0.0) {++i; continue;}
    std::size_t j = i;
    while (j < m && free_space[j] > 0.0) {++j;}
    if (static_cast<long>(j - i) > best_len) {
      best_len = static_cast<long>(j - i);
      gap_start = static_cast<long>(i);
      gap_end = static_cast<long>(j) - 1;
    }
    i = j;
  }
  if (gap_start < 0) {
    return DriveCommand{prev_steering_, std::min(params_.no_gap_speed, params_.speed_max)};
  }

  // Aim 80% at the middle of the longest run of maximum range, 20% at the gap centre.
  double gap_max = 0.0;
  for (long i = gap_start; i <= gap_end; ++i) {gap_max = std::max(gap_max, free_space[i]);}
  long run_first = -1, run_last = -1, run_best = 0;
  for (long i = gap_start; i <= gap_end; ) {
    if (free_space[i] != gap_max) {++i; continue;}
    long j = i;
    while (j <= gap_end && free_space[j] == gap_max) {++j;}
    if (j - i > run_best) {run_best = j - i; run_first = i - gap_start; run_last = j - 1 - gap_start;}
    i = j;
  }
  const long furthest = gap_start + (run_first + run_last) / 2;
  const long centred = (gap_start + gap_end) / 2;
  const long best = static_cast<long>(0.8 * furthest + 0.2 * centred);

  const double bearing = angle_min + static_cast<double>(crop_start + best) * angle_increment;
  double steer = std::clamp(params_.steering_gain * bearing, -params_.max_steer, params_.max_steer);
  const double blend = std::abs(steer) <= 0.139626 ? 0.58 : 0.20;  // 8 deg
  steer = std::clamp(
    blend * prev_steering_ + (1.0 - blend) * steer, -params_.max_steer, params_.max_steer);
  prev_steering_ = steer;

  const double abs_steer = std::abs(steer);
  double speed;
  if (abs_steer < params_.small_turn) {
    speed = params_.speed_max;
  } else if (abs_steer < params_.medium_turn) {
    speed = params_.medium_speed;
  } else if (abs_steer < params_.sharp_turn) {
    speed = params_.sharp_speed;
  } else {
    speed = params_.speed_min;
  }
  const long gap_len = gap_end - gap_start + 1;
  if (gap_len < params_.narrow_gap_beams) {speed = params_.narrow_gap_speed;}
  if (gap_len < params_.min_safe_gap) {speed = params_.speed_min;}
  speed = std::min(speed, params_.speed_max);
  return DriveCommand{steer, speed};
}

}  // namespace race_core
