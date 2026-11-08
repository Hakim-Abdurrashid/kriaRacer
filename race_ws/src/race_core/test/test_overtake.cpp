// Unit tests for the spline overtake planner.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "race_core/overtake.hpp"

using namespace race_core;

namespace
{
// counter-clockwise circle of radius R with the given wall distances on both sides
Raceline circle(double R, double w_left, double w_right, int n = 600)
{
  std::vector<RacelinePoint> pts(n);
  for (int i = 0; i < n; ++i) {
    const double th = 2 * M_PI * i / n;
    pts[i].x = R * std::cos(th);
    pts[i].y = R * std::sin(th);
    pts[i].psi = th + M_PI / 2;
    pts[i].kappa = 1.0 / R;
    pts[i].v = 4.0;
    pts[i].w_left = w_left;
    pts[i].w_right = w_right;
  }
  Raceline r;
  r.setPoints(pts);
  return r;
}

OpponentTrack opponent(double s, double d, double vs)
{
  OpponentTrack t;
  t.valid = true;
  t.s = s;
  t.d = d;
  t.vs = vs;
  t.hits = 10;
  return t;
}
}  // namespace

TEST(Overtake, FeasibleOnWideTrackWithSmoothProfile)
{
  const auto line = circle(10.0, 1.0, 1.0);
  OvertakeParams p;
  const auto plan = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.0, 2.0), 0, p);
  ASSERT_TRUE(plan.feasible);
  const double off = p.opp_half_width + p.clearance + p.car_half_width;
  EXPECT_NEAR(std::fabs(plan.d_target), off, 1e-9);
  // zero before the start, full offset alongside the opponent, zero again after the end
  EXPECT_NEAR(overtakeOffset(line, plan, line.wrapS(plan.s_start - 0.5)), 0.0, 1e-9);
  EXPECT_NEAR(overtakeOffset(line, plan, 3.0), plan.d_target, 1e-9);
  EXPECT_NEAR(overtakeOffset(line, plan, line.wrapS(plan.s_end + 0.5)), 0.0, 1e-9);
  // the ramp is monotonic
  double prev = 0.0;
  for (double s = plan.s_start; s <= plan.s_full; s += 0.05) {
    const double d = std::fabs(overtakeOffset(line, plan, s));
    EXPECT_GE(d + 1e-9, prev);
    prev = d;
  }
}

TEST(Overtake, InfeasibleOnNarrowTrack)
{
  const auto line = circle(10.0, 0.4, 0.4);   // 0.8 m wide: no room beside a 0.3 m car
  const auto plan = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.0, 2.0), 0, OvertakeParams());
  EXPECT_FALSE(plan.feasible);
}

TEST(Overtake, PicksTheSideWithMoreRoom)
{
  const auto line = circle(10.0, 1.0, 1.0);
  const auto plan = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.3, 2.0), 0, OvertakeParams());
  ASSERT_TRUE(plan.feasible);
  EXPECT_EQ(plan.side, -1);   // opponent is left of the line -> pass on the right
  const auto forced = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.3, 2.0), +1, OvertakeParams());
  EXPECT_EQ(forced.side, +1);
}

TEST(Overtake, KeepsStartWhenReplanning)
{
  const auto line = circle(10.0, 1.0, 1.0);
  OvertakeParams p;
  const auto first = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.0, 2.0), 0, p);
  const auto again = planOvertake(line, 1.0, 4.0, opponent(3.4, 0.0, 2.0), first.side, p, &first);
  EXPECT_DOUBLE_EQ(again.s_start, first.s_start);
  EXPECT_DOUBLE_EQ(again.s_full, first.s_full);
  // release = where we are hold_after past the opponent at constant speeds: we gained 0.6 m on it,
  // so the pass now ends 0.2 m earlier (opponent at 2 m/s, us at 4 m/s)
  EXPECT_NEAR(line.ds(first.s_release, again.s_release), -0.2, 1e-6);
}

TEST(Overtake, ShiftedLineIsOffsetAlongTheNormal)
{
  const auto line = circle(10.0, 1.0, 1.0);
  const auto plan = planOvertake(line, 0.0, 4.0, opponent(3.0, 0.0, 2.0), +1, OvertakeParams());
  const auto shifted = shiftedLine(line, plan);
  ASSERT_EQ(shifted.size(), line.size());
  // a point alongside the opponent: left of a CCW circle is inwards, so the radius shrinks
  const auto pr = line.project(10.0 * std::cos(3.0 / 10.0), 10.0 * std::sin(3.0 / 10.0));
  const auto & q = shifted[pr.index];
  EXPECT_NEAR(std::hypot(q.x, q.y), 10.0 - plan.d_target, 0.01);
}
