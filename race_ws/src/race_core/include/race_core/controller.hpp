// Raceline tracking (pure pursuit with actuation-delay compensation) and emergency braking.
#pragma once

#include <vector>

#include "race_core/geometry.hpp"
#include "race_core/raceline.hpp"

namespace race_core
{

struct PurePursuitParams
{
  double wheelbase = 0.33;          // m
  double lookahead_base = 0.25;     // m, lookahead = base + time * v, clamped to [min, max]
  double lookahead_time = 0.15;     // s
  double lookahead_min = 0.5;       // m
  double lookahead_max = 2.0;       // m
  double delay = 0.07;              // s, actuation + pipeline delay compensated by forward prediction
  double max_steer_left = 0.40;     // rad
  double max_steer_right = 0.40;    // rad (magnitude)
  double speed_preview = 0.25;      // s, read the speed profile this far ahead (brake early)
  double lat_err_slow_start = 0.15; // m, lateral error where slowing down begins
  double lat_err_slow_gain = 2.0;   // speed fraction lost per metre of error beyond the start
  double min_speed_factor = 0.4;    // never slow below this fraction of the profile for errors
  double min_speed = 0.6;           // m/s, floor on the commanded speed (sensorless motors stall)
};

struct PurePursuitOutput
{
  double steering = 0.0;   // rad
  double speed = 0.0;      // m/s (already scaled)
  double v_ref = 0.0;      // m/s, scaled profile speed before error slowdowns
  Projection proj;         // of the delay-compensated pose
  double lookahead = 0.0;  // m
  double target_x = 0.0;   // lookahead point (map frame)
  double target_y = 0.0;
};

// pose: current base_link pose in the map frame. v, yaw_rate: from odometry.
// hint: previous nearest index (or -1). speed_scale: global scaler on the raceline profile.
PurePursuitOutput purePursuit(const Raceline & line, const Pose2D & pose, double v, double yaw_rate,
  int hint, double speed_scale, const PurePursuitParams & p);

struct AebParams
{
  double ttc = 0.30;          // s, brake when the time to collision drops below this
  double min_speed = 0.8;     // m/s, inactive below this speed
  double half_width = 0.16;   // m, half the car width
  double margin = 0.01;       // m, extra lateral clearance in the swept corridor
  double front_offset = 0.45; // m, base_link (rear axle) to the front bumper
  double max_arc = 4.0;       // m, ignore points farther along the path
  double wheelbase = 0.33;
};

// Minimum time to collision over points (base_link frame, x0,y0,x1,y1,...) that fall inside the
// corridor swept by the car if it keeps its current steering. Returns a large value if none.
double minTimeToCollision(const std::vector<float> & pts_xy, double v, double steering,
  const AebParams & p);

}  // namespace race_core
