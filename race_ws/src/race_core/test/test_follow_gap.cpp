#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "race_core/follow_gap.hpp"

namespace
{
constexpr double kAngleMin = -2.35619;
constexpr double kInc = 0.00436332;
constexpr int kBeams = 1081;

double beamAngle(int i) {return kAngleMin + i * kInc;}

// Ranges seen from the middle of a straight corridor of the given half-width.
std::vector<float> corridor(double left, double right)
{
  std::vector<float> r(kBeams);
  for (int i = 0; i < kBeams; ++i) {
    const double a = beamAngle(i);
    const double s = std::sin(a);
    double d = 30.0;
    if (s > 1e-6) {d = left / s;} else if (s < -1e-6) {d = right / -s;}
    r[i] = static_cast<float>(std::min(d, 30.0));
  }
  return r;
}
}  // namespace

TEST(FollowGap, CentredInStraightCorridorDrivesStraightAtFullSpeed)
{
  race_core::FollowGap planner;
  const auto cmd = planner.compute(corridor(0.8, 0.8), kAngleMin, kInc);
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->steering, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(cmd->speed, planner.params().speed_max);
}

TEST(FollowGap, ObstacleOnTheRightSteersLeft)
{
  race_core::FollowGap planner;
  auto ranges = corridor(1.5, 1.5);
  for (int i = 0; i < kBeams; ++i) {
    const double a = beamAngle(i);
    if (a > -0.45 && a < 0.05) {ranges[i] = 0.9f;}  // box ahead, slightly right
  }
  const auto cmd = planner.compute(ranges, kAngleMin, kInc);
  ASSERT_TRUE(cmd.has_value());
  EXPECT_GT(cmd->steering, 0.05);
}

TEST(FollowGap, DeadScanRequestsStop)
{
  race_core::FollowGap planner;
  std::vector<float> nan_scan(kBeams, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(planner.compute(nan_scan, kAngleMin, kInc).has_value());
  EXPECT_FALSE(planner.compute({1.0f}, kAngleMin, kInc).has_value());
  EXPECT_FALSE(planner.compute(corridor(1, 1), kAngleMin, 0.0).has_value());
}

TEST(FollowGap, SpeedNeverExceedsCap)
{
  race_core::FollowGapParams p;
  p.speed_max = 1.2;
  race_core::FollowGap planner(p);
  for (double half : {0.5, 0.8, 1.5, 3.0}) {
    const auto cmd = planner.compute(corridor(half, half), kAngleMin, kInc);
    ASSERT_TRUE(cmd.has_value());
    EXPECT_LE(cmd->speed, 1.2);
  }
}

TEST(FollowGap, PreprocessMatchesNumpyConvolveSame)
{
  race_core::FollowGap planner;  // window 3, max range 5
  const std::vector<float> raw = {1.f, 2.f, std::numeric_limits<float>::infinity(), 4.f, 7.f};
  const auto out = planner.preprocess(raw);
  // numpy: clip -> [1,2,5,4,5]; convolve 'same' with [1/3]*3 (zero padded)
  const std::vector<double> expected = {1.0, 8.0 / 3, 11.0 / 3, 14.0 / 3, 3.0};
  ASSERT_EQ(out.size(), expected.size());
  for (std::size_t i = 0; i < out.size(); ++i) {EXPECT_NEAR(out[i], expected[i], 1e-9);}
}

TEST(FollowGap, DisparityExtensionCoversFarSideOfEdge)
{
  race_core::FollowGap planner;  // clearance = 0.31/2 + 0.30 = 0.455 m
  std::vector<float> raw(400, 4.0f);  // extension spans ~109 beams past the edge
  for (int i = 0; i < 100; ++i) {raw[i] = 1.0f;}  // near wall left of beam 100
  std::vector<double> proc(raw.begin(), raw.end());
  const auto ext = planner.extendDisparities(proc, raw, 0, kInc);
  const int expected_beams = static_cast<int>(std::ceil(std::asin(0.455 / 1.0) / kInc));
  EXPECT_DOUBLE_EQ(ext[99 + expected_beams], 1.0);
  EXPECT_DOUBLE_EQ(ext[99 + expected_beams + 1], 4.0);
  EXPECT_DOUBLE_EQ(ext[50], 1.0);
}
