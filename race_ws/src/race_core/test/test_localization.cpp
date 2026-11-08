// Unit tests for the race_core v1 modules: distance field, raceline projection, particle filter,
// pure pursuit and emergency braking.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "race_core/controller.hpp"
#include "race_core/occupancy_map.hpp"
#include "race_core/opponent.hpp"
#include "race_core/particle_filter.hpp"
#include "race_core/raceline.hpp"

using namespace race_core;

namespace
{

// 12 m x 8 m room (0.05 m cells) with walls on the border and an asymmetric box inside.
OccupancyMap makeRoom()
{
  const int W = 240, H = 160;
  std::vector<int8_t> c(W * H, OccupancyMap::kFree);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const bool border = x == 0 || y == 0 || x == W - 1 || y == H - 1;
      const bool box = x >= 150 && x < 170 && y >= 40 && y < 100;
      const bool post = x >= 60 && x < 64 && y >= 110 && y < 114;
      if (border || box || post) {c[y * W + x] = OccupancyMap::kOccupied;}
    }
  }
  OccupancyMap m;
  m.fromCells(W, H, 0.05, 0.0, 0.0, c);
  return m;
}

// Simulated scan endpoints (base frame) by brute-force ray marching on the map.
std::vector<float> scanFrom(const OccupancyMap & m, const Pose2D & p, int beams)
{
  std::vector<float> ep;
  for (int i = 0; i < beams; ++i) {
    const double a = -2.35 + 4.7 * i / (beams - 1);
    double r = 0.0;
    while (r < 10.0) {
      const double x = p.x + r * std::cos(p.yaw + a), y = p.y + r * std::sin(p.yaw + a);
      if (m.cellAt(x, y) == OccupancyMap::kOccupied) {break;}
      r += 0.01;
    }
    if (r < 10.0) {
      ep.push_back(static_cast<float>(r * std::cos(a)));
      ep.push_back(static_cast<float>(r * std::sin(a)));
    }
  }
  return ep;
}

Raceline makeCircle(double R, int n)
{
  std::vector<RacelinePoint> pts(n);
  for (int i = 0; i < n; ++i) {
    const double th = 2 * M_PI * i / n;
    pts[i].x = R * std::cos(th);
    pts[i].y = R * std::sin(th);
    pts[i].psi = normalizeAngle(th + M_PI / 2);   // counter-clockwise
    pts[i].kappa = 1.0 / R;
    pts[i].v = 3.0;
    pts[i].w_left = pts[i].w_right = 0.5;
  }
  Raceline r;
  r.setPoints(pts);
  return r;
}

}  // namespace

TEST(OccupancyMap, DistanceField)
{
  const auto m = makeRoom();
  EXPECT_NEAR(m.distanceAt(0.025, 0.025), 0.0, 1e-6);           // a border cell
  EXPECT_NEAR(m.distanceAt(1.025, 4.025), 1.0, 0.051);          // 20 cells from the left wall
  EXPECT_NEAR(m.distanceAt(6.0, 0.5), 0.475, 0.051);            // near the bottom wall
  EXPECT_EQ(m.cellAt(-1.0, 1.0), OccupancyMap::kUnknown);       // outside the map
}

TEST(Raceline, ProjectAndInterpolate)
{
  const auto r = makeCircle(2.0, 400);
  EXPECT_NEAR(r.length(), 2 * M_PI * 2.0, 0.01);
  const auto pr = r.project(0.0, 2.3);   // outside the CCW circle = right of the line
  EXPECT_NEAR(pr.s, r.length() / 4, 0.03);
  EXPECT_NEAR(pr.d, -0.3, 0.01);
  const auto pin = r.project(0.0, -1.8, pr.index, 400);   // inside = left
  EXPECT_NEAR(pin.d, 0.2, 0.01);
  const auto p = r.at(r.length() + r.length() / 2);   // wraps
  EXPECT_NEAR(p.x, -2.0, 0.01);
  EXPECT_NEAR(r.ds(r.length() - 0.5, 0.5), 1.0, 1e-6);
}

TEST(ParticleFilter, ConvergesFromOffsetGuess)
{
  const auto m = makeRoom();
  PFParams pp;
  pp.particles = 2000;
  pp.threads = 1;
  ParticleFilter pf(m, pp);
  const Pose2D truth{4.0, 3.0, 0.3};
  pf.initGaussian({4.25, 2.8, 0.15}, 0.3, 0.2);
  const auto ep = scanFrom(m, truth, 108);
  for (int i = 0; i < 30; ++i) {     // 0.75 s of scans at standstill
    pf.predict({0, 0, 0});
    ASSERT_TRUE(pf.update(ep));
  }
  const auto e = pf.estimate();
  EXPECT_NEAR(e.x, truth.x, 0.08);
  EXPECT_NEAR(e.y, truth.y, 0.08);
  EXPECT_NEAR(e.yaw, truth.yaw, 0.05);
  EXPECT_GT(pf.matchScore(), 0.9);
}

TEST(ParticleFilter, TracksMotion)
{
  const auto m = makeRoom();
  PFParams pp;
  pp.threads = 1;
  ParticleFilter pf(m, pp);
  Pose2D truth{3.0, 2.0, 0.0};
  pf.initGaussian(truth, 0.1, 0.05);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int i = 0; i < 80; ++i) {          // drive 0.1 m per step with a slight left curve
    const Pose2D step{0.1, 0.0, 0.01};
    truth = compose(truth, step);
    const Pose2D odom{0.1 * 1.05 + 0.005 * n(rng), 0.0, 0.01 + 0.005 * n(rng)};   // biased odometry
    pf.predict(odom);
    ASSERT_TRUE(pf.update(scanFrom(m, truth, 108)));
  }
  const auto e = pf.estimate();
  EXPECT_NEAR(e.x, truth.x, 0.10);
  EXPECT_NEAR(e.y, truth.y, 0.10);
  EXPECT_NEAR(normalizeAngle(e.yaw - truth.yaw), 0.0, 0.05);
}

TEST(Controller, PurePursuitOnCircle)
{
  const auto r = makeCircle(2.0, 400);
  PurePursuitParams p;
  p.delay = 0.0;
  p.max_steer_left = p.max_steer_right = 0.6;
  const Pose2D on_line{2.0, 0.0, M_PI / 2};
  const auto out = purePursuit(r, on_line, 2.0, 1.0, -1, 0.5, p);
  // pure pursuit on a circle of radius R with lookahead chord: steering ~ atan(L / R)
  EXPECT_NEAR(out.steering, std::atan(p.wheelbase / 2.0), 0.02);
  EXPECT_NEAR(out.v_ref, 1.5, 1e-6);   // 3 m/s profile x 0.5
  EXPECT_NEAR(out.speed, 1.5, 1e-6);
  const Pose2D outside{2.4, 0.0, M_PI / 2};   // 0.4 m right of the line -> steer harder left
  const auto out2 = purePursuit(r, outside, 2.0, 1.0, -1, 0.5, p);
  EXPECT_GT(out2.steering, out.steering + 0.05);
  EXPECT_LT(out2.speed, out.speed);    // slowed down for the lateral error
}

TEST(Controller, EmergencyBrakeCorridor)
{
  AebParams a;
  std::vector<float> wall_ahead, wall_side;
  for (int i = -10; i <= 10; ++i) {
    wall_ahead.push_back(1.5f);
    wall_ahead.push_back(0.05f * i);
    wall_side.push_back(0.1f * (i + 11));
    wall_side.push_back(0.35f);
  }
  EXPECT_NEAR(minTimeToCollision(wall_ahead, 3.0, 0.0, a), (1.5 - a.front_offset) / 3.0, 1e-3);
  EXPECT_GT(minTimeToCollision(wall_side, 3.0, 0.0, a), 100.0);   // beside us: no brake
  EXPECT_GT(minTimeToCollision(wall_ahead, 0.3, 0.0, a), 100.0);  // too slow to matter
  // turning hard left: the wall straight ahead at 1.5 m is no longer on the swept arc
  EXPECT_GT(minTimeToCollision(wall_ahead, 3.0, 0.4, a), 0.6);
}

TEST(Opponent, DetectsBoxOnTrack)
{
  const auto m = makeRoom();
  // straight raceline along y = 2 from x = 1 to x = 11 and back on y = 6 (closed loop)
  std::vector<RacelinePoint> pts;
  for (double x = 1.0; x < 11.0; x += 0.1) {pts.push_back({0, x, 2.0, 0, 0, 3, 0, 1.5, 1.5});}
  for (double x = 11.0; x > 1.0; x -= 0.1) {pts.push_back({0, x, 6.0, M_PI, 0, 3, 0, 1.5, 1.5});}
  Raceline line;
  line.setPoints(pts);
  // points of a 0.3 m wide object 2 m ahead of a car at (3, 2), plus a wall segment
  std::vector<float> scan;
  for (int i = 0; i < 7; ++i) {
    scan.push_back(5.0f);
    scan.push_back(1.85f + 0.05f * i);
  }
  for (int i = 0; i < 10; ++i) {
    scan.push_back(0.03f + 0.1f * i);
    scan.push_back(0.03f);
  }
  OpponentParams op;
  const auto obs = detectObstacles(scan, 3.0, 2.0, m, line, -1, op);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].x, 5.0, 1e-3);
  EXPECT_NEAR(obs[0].d, 0.0, 0.05);
  OpponentTracker tr(op);
  for (int k = 0; k < 10; ++k) {
    std::vector<Obstacle> o = obs;
    o[0].s += 0.05 * k;          // moving 0.05 m per 0.025 s = 2 m/s
    tr.update(o, 0.025 * k, 2.0, line);
  }
  EXPECT_TRUE(tr.track().valid);
  EXPECT_NEAR(tr.track().vs, 2.0, 0.5);
}
