// Follow-the-gap from blaze_racer branch iv2026 (autonomy_ws/src/follow_the_gap/planner.py and
// autonomy_core/lidar_utils.py), ported line by line so race_core's REACTIVE mode drives exactly
// like the team's tested node, with race_core's deadman, watchdogs and lap timer around it.
//
// Index-based tunables (bubble_radius, safe_threshold, best_point_conv_size) are in scan beams after
// the field-of-view crop, as in the Python. Parity with the Python planner on recorded scans:
// tools/ftg_iv_parity.py.
#pragma once

#include <optional>
#include <vector>

#include "race_core/follow_gap.hpp"   // DriveCommand

namespace race_core
{

// Defaults = blaze_racer iv2026 follow_the_gap/config/params.yaml as run on the car on 2026-11-06.
struct FollowGapIVParams
{
  double fov_degrees = 270.0;
  int bubble_radius = 175;            // beams
  int preprocess_conv_size = 3;       // beams
  int safe_threshold = 30;            // beams
  int best_point_conv_size = 200;     // beams
  double max_lidar_dist = 10.0;       // m
  double steering_gain = 0.5;
  double max_steer = 0.349066;        // rad
  double straights_steering_angle = 0.1396;
  double fast_steering_angle = 0.0698;
  double corners_speed = 1.5;
  double straights_speed = 7.0;
  double fast_speed = 5.0;
  bool use_distance_speed = true;
  double front_angle_width = 0.1745;  // rad
  double dist_min_speed = 1.0;        // m of front clearance -> distance_min_speed
  double dist_max_speed = 5.0;        // m of front clearance -> distance_max_speed
  double distance_min_speed = 2.0;    // m/s
  double distance_max_speed = 4.5;    // m/s
};

class FollowGapIV
{
public:
  explicit FollowGapIV(const FollowGapIVParams & params = FollowGapIVParams());

  // nullopt = no usable field of view (the Python node then publishes a stop).
  std::optional<DriveCommand> compute(
    const std::vector<float> & ranges, double angle_min, double angle_increment);

  const FollowGapIVParams & params() const {return p_;}
  double frontClearance() const {return front_dist_;}   // of the last compute(), m

private:
  FollowGapIVParams p_;
  double front_dist_ = 0.0;
  std::vector<double> buf_, tmp_;
};

}  // namespace race_core
