// Small 2D pose helpers shared by the race_core modules.
#pragma once

#include <cmath>

namespace race_core
{

inline double normalizeAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) {a += 2.0 * M_PI;}
  return a - M_PI;
}

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// a * b: b expressed in a's frame, result in a's parent frame.
inline Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.yaw + b.yaw)};
}

inline Pose2D inverse(const Pose2D & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-c * a.x - s * a.y, s * a.x - c * a.y, normalizeAngle(-a.yaw)};
}

// Motion from a to b expressed in a's frame (inverse(a) * b).
inline Pose2D between(const Pose2D & a, const Pose2D & b) {return compose(inverse(a), b);}

}  // namespace race_core
