#include "race_core/particle_filter.hpp"

#include <algorithm>
#include <cmath>

namespace race_core
{

ParticleFilter::ParticleFilter(const OccupancyMap & map, const PFParams & params, uint32_t seed)
: map_(map), p_(params), rng_(seed)
{
  W_ = map.width();
  H_ = map.height();
  inv_res_ = static_cast<float>(1.0 / map.resolution());
  ox_ = static_cast<float>(map.originX());
  oy_ = static_cast<float>(map.originY());
  const auto & dist = map.distanceField();
  loglik_.resize(dist.size());
  const double inv2s2 = 1.0 / (2.0 * p_.sigma_hit * p_.sigma_hit);
  for (size_t i = 0; i < dist.size(); ++i) {
    const double d = std::min<double>(dist[i], p_.max_dist);
    loglik_[i] = static_cast<float>(std::log(p_.z_hit * std::exp(-d * d * inv2s2) + p_.z_rand));
  }
  outside_loglik_ = static_cast<float>(std::log(p_.z_rand));
}

float ParticleFilter::logLik(float x, float y) const
{
  const int cx = static_cast<int>((x - ox_) * inv_res_);
  const int cy = static_cast<int>((y - oy_) * inv_res_);
  if (x < ox_ || y < oy_ || cx >= W_ || cy >= H_) {return outside_loglik_;}
  return loglik_[cy * W_ + cx];
}

void ParticleFilter::initGaussian(const Pose2D & mean, double sigma_xy, double sigma_yaw)
{
  std::normal_distribution<float> nxy(0.0f, static_cast<float>(sigma_xy));
  std::normal_distribution<float> nyaw(0.0f, static_cast<float>(sigma_yaw));
  ps_.resize(p_.particles);
  const float w = 1.0f / p_.particles;
  for (auto & q : ps_) {
    q = {static_cast<float>(mean.x) + nxy(rng_), static_cast<float>(mean.y) + nxy(rng_),
      static_cast<float>(normalizeAngle(mean.yaw + nyaw(rng_))), w};
  }
  estimate_ = mean;
  neff_ratio_ = 1.0;
  spread_xy_ = sigma_xy;
  spread_yaw_ = sigma_yaw;
}

void ParticleFilter::initGlobal(int count, double min_wall_dist)
{
  std::vector<int> cells;
  const auto & dist = map_.distanceField();
  for (int i = 0; i < W_ * H_; ++i) {
    if (map_.cells()[i] == OccupancyMap::kFree && dist[i] >= min_wall_dist) {cells.push_back(i);}
  }
  if (cells.empty()) {return;}
  std::uniform_int_distribution<size_t> pick(0, cells.size() - 1);
  std::uniform_real_distribution<float> u01(0.0f, 1.0f);
  const float res = static_cast<float>(map_.resolution());
  ps_.resize(count);
  for (auto & q : ps_) {
    const int i = cells[pick(rng_)];
    q = {ox_ + (i % W_ + u01(rng_)) * res, oy_ + (i / W_ + u01(rng_)) * res,
      static_cast<float>(-M_PI + 2.0 * M_PI * u01(rng_)), 1.0f / count};
  }
  neff_ratio_ = 1.0;
  spread_xy_ = 1e3;
  spread_yaw_ = M_PI;
}

void ParticleFilter::predict(const Pose2D & delta, double slip)
{
  const double trans = std::hypot(delta.x, delta.y);
  const double rot = std::fabs(delta.yaw);
  const float s_long = static_cast<float>((p_.alpha_trans * trans + p_.min_trans_noise) * slip);
  const float s_lat = static_cast<float>((0.5 * p_.alpha_trans * trans + p_.min_trans_noise) * slip);
  const float s_yaw = static_cast<float>(
    (p_.alpha_rot * rot + p_.alpha_rot_trans * trans + p_.min_rot_noise) * slip);
  std::normal_distribution<float> n01(0.0f, 1.0f);
  const float dx = static_cast<float>(delta.x), dy = static_cast<float>(delta.y);
  const float dyaw = static_cast<float>(delta.yaw);
  for (auto & q : ps_) {
    const float mx = dx + s_long * n01(rng_);
    const float my = dy + s_lat * n01(rng_);
    const float c = std::cos(q.yaw), s = std::sin(q.yaw);
    q.x += c * mx - s * my;
    q.y += s * mx + c * my;
    q.yaw += dyaw + s_yaw * n01(rng_);
    if (q.yaw > static_cast<float>(M_PI)) {q.yaw -= static_cast<float>(2.0 * M_PI);}
    if (q.yaw < static_cast<float>(-M_PI)) {q.yaw += static_cast<float>(2.0 * M_PI);}
  }
}

bool ParticleFilter::update(const std::vector<float> & ep)
{
  const int B = static_cast<int>(ep.size() / 2);
  const int N = static_cast<int>(ps_.size());
  if (B < 10 || N == 0) {return false;}
  float maxl = -1e30f;
  #pragma omp parallel for num_threads(p_.threads) schedule(static) reduction(max:maxl)
  for (int k = 0; k < N; ++k) {
    Particle & q = ps_[k];
    const float c = std::cos(q.yaw), s = std::sin(q.yaw);
    float acc = 0.0f;
    for (int i = 0; i < B; ++i) {
      const float bx = ep[2 * i], by = ep[2 * i + 1];
      acc += logLik(q.x + c * bx - s * by, q.y + s * bx + c * by);
    }
    q.w = acc;
    maxl = std::max(maxl, acc);
  }
  double sum = 0.0;
  const float sq = static_cast<float>(p_.squash);
  for (auto & q : ps_) {
    q.w = std::exp(sq * (q.w - maxl));
    sum += q.w;
  }
  double sum2 = 0.0;
  for (auto & q : ps_) {
    q.w = static_cast<float>(q.w / sum);
    sum2 += static_cast<double>(q.w) * q.w;
  }
  neff_ratio_ = 1.0 / (sum2 * N);
  computeEstimate();
  match_score_ = scorePose(estimate_, ep);
  if (neff_ratio_ < p_.resample_neff) {resample(N);}
  return true;
}

void ParticleFilter::computeEstimate()
{
  // Weighted mean around the best particle (robust to a multimodal cloud after a global init).
  const auto best = std::max_element(ps_.begin(), ps_.end(),
      [](const Particle & a, const Particle & b) {return a.w < b.w;});
  double sw = 0, sx = 0, sy = 0, sc = 0, ss = 0;
  for (const auto & q : ps_) {
    if (std::fabs(q.x - best->x) > 0.5f || std::fabs(q.y - best->y) > 0.5f ||
      std::fabs(normalizeAngle(q.yaw - best->yaw)) > 0.6) {continue;}
    sw += q.w;
    sx += q.w * q.x;
    sy += q.w * q.y;
    sc += q.w * std::cos(q.yaw);
    ss += q.w * std::sin(q.yaw);
  }
  estimate_ = {sx / sw, sy / sw, std::atan2(ss, sc)};
  // spread over the whole cloud
  double vxy = 0, vyaw = 0, tw = 0;
  for (const auto & q : ps_) {
    const double dx = q.x - estimate_.x, dy = q.y - estimate_.y;
    const double dyaw = normalizeAngle(q.yaw - estimate_.yaw);
    vxy += q.w * (dx * dx + dy * dy);
    vyaw += q.w * dyaw * dyaw;
    tw += q.w;
  }
  spread_xy_ = std::sqrt(vxy / tw);
  spread_yaw_ = std::sqrt(vyaw / tw);
}

void ParticleFilter::resample(int count)
{
  const int N = static_cast<int>(ps_.size());
  tmp_.resize(count);
  double total = 0.0;
  for (const auto & q : ps_) {total += q.w;}
  const double step = total / count;
  std::uniform_real_distribution<double> u(0.0, step);
  double target = u(rng_), cum = ps_[0].w;
  int j = 0;
  for (int k = 0; k < count; ++k) {
    while (target > cum && j < N - 1) {cum += ps_[++j].w;}
    tmp_[k] = ps_[j];
    tmp_[k].w = 1.0f / count;
    target += step;
  }
  ps_.swap(tmp_);
}

void ParticleFilter::shrinkTo(int count)
{
  if (count < size()) {resample(count);}
}

double ParticleFilter::scorePose(const Pose2D & pose, const std::vector<float> & ep) const
{
  const int B = static_cast<int>(ep.size() / 2);
  if (B == 0) {return 0.0;}
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  int good = 0;
  for (int i = 0; i < B; ++i) {
    const double x = pose.x + c * ep[2 * i] - s * ep[2 * i + 1];
    const double y = pose.y + s * ep[2 * i] + c * ep[2 * i + 1];
    int cx, cy;   // endpoints outside the map never count as matched
    if (map_.worldToCell(x, y, cx, cy) && map_.distanceField()[cy * W_ + cx] < p_.good_dist) {++good;}
  }
  return static_cast<double>(good) / B;
}

}  // namespace race_core
