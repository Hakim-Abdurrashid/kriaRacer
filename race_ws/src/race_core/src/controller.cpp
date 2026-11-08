#include "race_core/controller.hpp"

#include <algorithm>
#include <cmath>

namespace race_core
{

PurePursuitOutput purePursuit(const Raceline & line, const Pose2D & pose, double v, double yaw_rate,
  int hint, double speed_scale, const PurePursuitParams & p)
{
  PurePursuitOutput out;
  // 1. predict where the car will be when this command takes effect
  Pose2D q = pose;
  const double dt = p.delay;
  if (std::fabs(yaw_rate) > 1e-3) {
    const double r = v / yaw_rate;
    q.x += r * (std::sin(pose.yaw + yaw_rate * dt) - std::sin(pose.yaw));
    q.y += r * (-std::cos(pose.yaw + yaw_rate * dt) + std::cos(pose.yaw));
  } else {
    q.x += v * dt * std::cos(pose.yaw);
    q.y += v * dt * std::sin(pose.yaw);
  }
  q.yaw = normalizeAngle(pose.yaw + yaw_rate * dt);

  // 2. project onto the line and pick the lookahead point
  out.proj = line.project(q.x, q.y, hint);
  const double vv = std::max(0.0, v);
  out.lookahead = std::clamp(p.lookahead_base + p.lookahead_time * vv, p.lookahead_min, p.lookahead_max);
  const RacelinePoint tgt = line.at(out.proj.s + out.lookahead);
  out.target_x = tgt.x;
  out.target_y = tgt.y;

  // 3. pure pursuit steering towards it
  const double dx = tgt.x - q.x, dy = tgt.y - q.y;
  const double c = std::cos(q.yaw), s = std::sin(q.yaw);
  const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
  const double l2 = std::max(lx * lx + ly * ly, 1e-4);
  double steer = std::atan(2.0 * p.wheelbase * ly / l2);
  out.steering = std::clamp(steer, -p.max_steer_right, p.max_steer_left);

  // 4. speed from the profile, a little ahead, scaled; slow down when off the line
  const RacelinePoint here = line.at(out.proj.s + vv * p.speed_preview);
  out.v_ref = here.v * speed_scale;
  double factor = 1.0;
  const double e = std::fabs(out.proj.d);
  if (e > p.lat_err_slow_start) {factor -= p.lat_err_slow_gain * (e - p.lat_err_slow_start);}
  const double heading_err = std::fabs(normalizeAngle(q.yaw - out.proj.psi));
  if (heading_err > 0.35) {factor -= 0.8 * (heading_err - 0.35);}
  factor = std::max(factor, p.min_speed_factor);
  out.speed = std::max(out.v_ref * factor, p.min_speed);
  return out;
}

double minTimeToCollision(const std::vector<float> & pts, double v, double steering,
  const AebParams & p)
{
  if (v < p.min_speed) {return 1e9;}
  const double kappa = std::tan(steering) / p.wheelbase;
  const double band = p.half_width + p.margin;
  double best_arc = 1e9;
  const size_t n = pts.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    double x = pts[2 * i], y = pts[2 * i + 1];
    if (x < 0.0) {continue;}               // behind the rear axle
    double arc, lateral;
    if (std::fabs(kappa) < 1e-3) {
      arc = x;
      lateral = y;
    } else {
      double k = kappa;
      if (k < 0) {   // mirror right turns onto left turns
        k = -k;
        y = -y;
      }
      const double R = 1.0 / k;
      const double rho = std::hypot(x, y - R);
      lateral = R - rho;                    // > 0: inside of the turn
      const double phi = std::atan2(x, R - y);  // angle travelled around the centre
      if (phi < 0.0 || phi > 0.5 * M_PI) {continue;}   // only the next quarter turn is predictable
      arc = R * phi;
    }
    if (std::fabs(lateral) > band || arc > p.max_arc) {continue;}
    // Only points ahead of the front bumper are frontal collisions; walls beside the car body are
    // scrapes that braking cannot prevent (and chassis returns live there too).
    if (arc < p.front_offset) {continue;}
    best_arc = std::min(best_arc, arc);
  }
  if (best_arc >= 1e9) {return 1e9;}
  return std::max(0.0, best_arc - p.front_offset) / v;
}

}  // namespace race_core
