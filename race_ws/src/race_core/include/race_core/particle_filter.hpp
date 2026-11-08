// Likelihood-field particle filter (RACE_PLAN.md §5.2): map -> base_link from LiDAR + odometry deltas.
//
// Motion model: odometry delta (from the EKF) applied per particle with noise proportional to the
// motion, inflated by a slip factor. Sensor model: per-beam log-likelihood looked up in a precomputed
// field over the map's distance transform (Thrun's likelihood field: z_hit Gaussian + z_rand floor).
// The update runs over ~108 downsampled beams; 2000 particles cost ~8.5 ms on one A53 core
// (research/pf_bench.cpp), ~4.6 ms on two (OpenMP).
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "race_core/geometry.hpp"
#include "race_core/occupancy_map.hpp"

namespace race_core
{

struct PFParams
{
  int particles = 1000;
  double sigma_hit = 0.08;        // m, std-dev of the hit model
  double z_hit = 0.85;            // weight of the hit Gaussian
  double z_rand = 0.15;           // weight of the uniform floor (robust to unmapped objects)
  double max_dist = 1.0;          // m, distance field clamp
  double squash = 0.4;            // multiplies the summed log-likelihood (beams are correlated)
  double alpha_trans = 0.15;      // translation noise per metre travelled
  double alpha_rot_trans = 0.10;  // heading noise per metre travelled (rad/m)
  double alpha_rot = 0.15;        // heading noise per radian turned
  double min_trans_noise = 0.008; // m per update (keeps diversity when stopped)
  double min_rot_noise = 0.004;   // rad per update
  double resample_neff = 0.5;     // resample when Neff < this * N
  double good_dist = 0.15;        // m, a beam endpoint closer than this to a wall counts as matched
  int threads = 2;
};

struct Particle
{
  float x, y, yaw, w;
};

class ParticleFilter
{
public:
  ParticleFilter(const OccupancyMap & map, const PFParams & params, uint32_t seed = 42);

  void initGaussian(const Pose2D & mean, double sigma_xy, double sigma_yaw);
  // Uniform over free cells whose distance to the nearest wall is at least min_wall_dist.
  void initGlobal(int count, double min_wall_dist = 0.15);

  // delta: odometry motion since the last call, in the robot frame at the previous pose.
  // slip >= 1 inflates the noise (e.g. when gyro and Ackermann yaw rates disagree).
  void predict(const Pose2D & delta, double slip = 1.0);

  // endpoints: beam end points in base_link (x0, y0, x1, y1, ...). Returns false if too few beams.
  bool update(const std::vector<float> & endpoints_xy);

  Pose2D estimate() const {return estimate_;}
  double neffRatio() const {return neff_ratio_;}
  // Fraction of beams whose endpoint (seen from the estimate) lies within good_dist of a wall.
  double matchScore() const {return match_score_;}
  // Weighted position spread (m) and heading spread (rad) of the cloud.
  double spreadXY() const {return spread_xy_;}
  double spreadYaw() const {return spread_yaw_;}
  const std::vector<Particle> & particles() const {return ps_;}
  int size() const {return static_cast<int>(ps_.size());}
  // Shrink the cloud back to params.particles after a global init has converged.
  void shrinkTo(int count);
  // Score any pose against the scan (same matched-fraction metric as matchScore()).
  double scorePose(const Pose2D & pose, const std::vector<float> & endpoints_xy) const;

private:
  void computeEstimate();
  void resample(int count);
  float logLik(float x, float y) const;

  const OccupancyMap & map_;
  PFParams p_;
  std::vector<float> loglik_;     // per map cell
  std::vector<Particle> ps_, tmp_;
  std::mt19937 rng_;
  Pose2D estimate_;
  double neff_ratio_ = 0.0;
  double match_score_ = 0.0;
  double spread_xy_ = 0.0;
  double spread_yaw_ = 0.0;
  float inv_res_;
  float ox_, oy_;
  int W_, H_;
  float outside_loglik_;
};

}  // namespace race_core
