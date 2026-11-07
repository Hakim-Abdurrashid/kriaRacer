// Line-by-line port of blaze_racer iv2026 follow_the_gap/planner.py (see follow_gap_iv.hpp).
// numpy semantics kept where they decide something: np.convolve(..., 'same') window placement and
// zero padding, argmin/argmax returning the first extreme (and the first NaN, as numpy does),
// Python's round-half-to-even, and the first longest gap winning ties.
#include "race_core/follow_gap_iv.hpp"

#include <algorithm>
#include <cmath>

namespace race_core
{

namespace
{
// numpy.convolve(a, ones(w), 'same') for len(a) >= w: out[k] = sum(a[k - w/2 .. k + (w-1)/2]),
// zero outside a.
void boxSame(const double * a, int n, int w, std::vector<double> & out)
{
  out.assign(n, 0.0);
  const int lo = w / 2, hi = (w - 1) / 2;
  for (int k = 0; k < n; ++k) {
    double s = 0.0;
    const int j0 = std::max(0, k - lo), j1 = std::min(n - 1, k + hi);
    for (int j = j0; j <= j1; ++j) {s += a[j];}
    out[k] = s;
  }
}

int argMin(const std::vector<double> & v)
{
  int best = 0;
  for (int i = 0; i < static_cast<int>(v.size()); ++i) {
    if (std::isnan(v[i])) {return i;}
    if (v[i] < v[best]) {best = i;}
  }
  return best;
}

int argMax(const double * v, int n)
{
  int best = 0;
  for (int i = 0; i < n; ++i) {
    if (std::isnan(v[i])) {return i;}
    if (v[i] > v[best]) {best = i;}
  }
  return best;
}

double clip(double x, double lo, double hi)
{
  if (std::isnan(x)) {return x;}   // np.clip keeps NaN
  return std::min(std::max(x, lo), hi);
}
}  // namespace

FollowGapIV::FollowGapIV(const FollowGapIVParams & params)
: p_(params) {}

std::optional<DriveCommand> FollowGapIV::compute(
  const std::vector<float> & ranges, double angle_min, double angle_increment)
{
  // BaseDriveNode: no finite beam at all -> stop
  if (std::none_of(ranges.begin(), ranges.end(), [](float r) {return std::isfinite(r);}) ||
    angle_increment <= 0.0)
  {
    return std::nullopt;
  }
  // 1. clamp, crop to the forward field of view (lidar_utils.crop_to_fov)
  const double half_fov = p_.fov_degrees * (M_PI / 180.0) / 2.0;
  int start = static_cast<int>(std::ceil((-half_fov - angle_min) / angle_increment));
  int stop = static_cast<int>(std::floor((half_fov - angle_min) / angle_increment)) + 1;
  start = std::max(start, 0);
  stop = std::min(stop, static_cast<int>(ranges.size()));
  if (stop <= start) {return std::nullopt;}
  const double angle_of_first = angle_min + start * angle_increment;
  const int n = stop - start;
  tmp_.resize(n);
  for (int i = 0; i < n; ++i) {
    tmp_[i] = clip(static_cast<double>(ranges[start + i]), 0.0, p_.max_lidar_dist);
  }

  // 2. smooth, re-clamp
  if (p_.preprocess_conv_size <= 1 || n < p_.preprocess_conv_size) {
    buf_ = tmp_;
  } else {
    boxSame(tmp_.data(), n, p_.preprocess_conv_size, buf_);
    for (double & r : buf_) {r /= p_.preprocess_conv_size;}
  }
  for (double & r : buf_) {r = clip(r, 0.0, p_.max_lidar_dist);}

  // 3. bubble around the closest point; front clearance measured before the bubble
  const int closest = argMin(buf_);
  const int min_index = std::max(closest - p_.bubble_radius, 0);
  const int max_index = std::min(closest + p_.bubble_radius, n - 1);
  {
    const double half = p_.front_angle_width / 2.0;
    const int i_center = static_cast<int>(std::nearbyint(-angle_of_first / angle_increment));
    const int i_half = std::max(1, static_cast<int>(std::nearbyint(half / angle_increment)));
    const int i0 = std::max(0, i_center - i_half), i1 = std::min(n, i_center + i_half + 1);
    if (i1 <= i0) {
      front_dist_ = p_.max_lidar_dist;
    } else {
      double s = 0.0;
      for (int i = i0; i < i1; ++i) {s += buf_[i];}
      front_dist_ = s / (i1 - i0);
    }
  }
  for (int i = min_index; i < max_index; ++i) {buf_[i] = 0.0;}

  // 4. first longest run of non-zero beams, best point inside it
  int g0 = -1, g1 = -1, best_len = 0;
  for (int i = 0; i < n; ) {
    if (buf_[i] == 0.0) {++i; continue;}
    int j = i;
    while (j < n && buf_[j] != 0.0) {++j;}
    if (j - i > best_len) {best_len = j - i; g0 = i; g1 = j;}
    i = j;
  }
  int best_idx;
  if (g0 < 0) {
    best_idx = n / 2;   // everything masked: straight ahead
  } else if (g1 - g0 < p_.best_point_conv_size) {
    best_idx = g0 + (g1 - g0) / 2;
  } else {
    boxSame(buf_.data() + g0, g1 - g0, p_.best_point_conv_size, tmp_);
    for (double & r : tmp_) {r /= p_.best_point_conv_size;}   // same rounding as the Python
    best_idx = argMax(tmp_.data(), g1 - g0) + g0;
  }

  // 5. steering (lidar_utils.index_to_steering) and speed
  const double bearing = angle_of_first + best_idx * angle_increment;
  const double steering = std::clamp(bearing * p_.steering_gain, -p_.max_steer, p_.max_steer);
  double speed;
  if (p_.use_distance_speed) {
    const double t = std::clamp((front_dist_ - p_.dist_min_speed) /
        std::max(p_.dist_max_speed - p_.dist_min_speed, 1e-6), 0.0, 1.0);
    speed = std::isnan(t) ? p_.distance_min_speed :
      p_.distance_min_speed + t * (p_.distance_max_speed - p_.distance_min_speed);
  } else if (std::fabs(steering) > p_.straights_steering_angle) {
    speed = p_.corners_speed;
  } else if (std::fabs(steering) > p_.fast_steering_angle) {
    speed = p_.straights_speed;
  } else {
    speed = p_.fast_speed;
  }
  return DriveCommand{steering, speed};
}

}  // namespace race_core
