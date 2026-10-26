// Follow-the-gap / disparity-extender planner (REACTIVE mode of race_core).
//
// ROS-free C++ port of blaze_racer's follow_the_gap_2 (Python) so it can run
// inside race_core without a Python node on /scan. Behaviour matches the
// Python version beam for beam; see test/test_follow_gap.cpp and
// tools/ftg_parity.py for the parity check against the original.
#pragma once

#include <optional>
#include <vector>

namespace race_core
{

struct FollowGapParams
{
  double bubble_radius = 0.4;         // m, safety bubble around the closest point
  int preprocess_conv_size = 3;       // beams, moving-average window
  double max_lidar_range = 5.0;       // m, ranges are clamped to this
  double speed_min = 1.0;             // m/s
  double speed_max = 2.8;             // m/s, hard cap on every speed branch
  double steering_gain = 0.7;         // bearing -> steering
  double max_steer = 0.60;            // rad
  double disparity_threshold = 0.7;   // m, adjacent-beam jump treated as an edge
  double vehicle_width = 0.31;        // m
  double disparity_margin = 0.30;     // m, extra clearance on each side
  int center_half_width = 100;        // beams, window checked first for close obstacles
  double center_priority_dist = 0.4;  // m, a centre point closer than this wins
  double small_turn = 0.0872665;      // rad (5 deg)  -> speed_max below this
  double medium_turn = 0.261799;      // rad (15 deg) -> medium_speed below this
  double sharp_turn = 0.349066;       // rad (20 deg) -> sharp_speed below this
  double medium_speed = 2.2;          // m/s
  double sharp_speed = 1.5;           // m/s
  int narrow_gap_beams = 100;         // gap narrower than this -> narrow_gap_speed
  double narrow_gap_speed = 1.5;      // m/s
  int min_safe_gap = 60;              // gap narrower than this -> speed_min
  double no_gap_speed = 1.0;          // m/s, crawl when no gap is found
};

struct DriveCommand
{
  double steering;  // rad, positive = left
  double speed;     // m/s
};

class FollowGap
{
public:
  explicit FollowGap(const FollowGapParams & params = FollowGapParams());

  // Returns nullopt when the scan is unusable (caller should stop the car).
  std::optional<DriveCommand> compute(
    const std::vector<float> & ranges, double angle_min, double angle_increment);

  void reset() {prev_steering_ = 0.0;}
  const FollowGapParams & params() const {return params_;}

  // Exposed for unit tests.
  std::vector<double> preprocess(const std::vector<float> & ranges) const;
  std::vector<double> extendDisparities(
    const std::vector<double> & processed, const std::vector<float> & raw,
    std::size_t offset, double angle_increment) const;

private:
  FollowGapParams params_;
  double prev_steering_ = 0.0;
};

}  // namespace race_core
