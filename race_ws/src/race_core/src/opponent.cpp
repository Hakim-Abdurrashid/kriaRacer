#include "race_core/opponent.hpp"

#include <cmath>

namespace race_core
{

std::vector<Obstacle> detectObstacles(const std::vector<float> & pts, double car_x, double car_y,
  const OccupancyMap & map, const Raceline & line, int hint, const OpponentParams & p)
{
  std::vector<Obstacle> out;
  const size_t n = pts.size() / 2;
  std::vector<int> cluster;   // indices of the current cluster
  auto flush = [&]() {
      if (static_cast<int>(cluster.size()) >= p.min_points) {
        const double x0 = pts[2 * cluster.front()], y0 = pts[2 * cluster.front() + 1];
        const double x1 = pts[2 * cluster.back()], y1 = pts[2 * cluster.back() + 1];
        const double size = std::hypot(x1 - x0, y1 - y0);
        if (size >= p.min_size && size <= p.max_size) {
          Obstacle o;
          for (int i : cluster) {
            o.x += pts[2 * i];
            o.y += pts[2 * i + 1];
          }
          o.x /= cluster.size();
          o.y /= cluster.size();
          o.size = size;
          o.points = static_cast<int>(cluster.size());
          const Projection pr = line.project(o.x, o.y, hint, 80);
          o.s = pr.s;
          o.d = pr.d;
          out.push_back(o);
        }
      }
      cluster.clear();
    };
  for (size_t i = 0; i < n; ++i) {
    const double x = pts[2 * i], y = pts[2 * i + 1];
    bool foreign = std::hypot(x - car_x, y - car_y) <= p.max_range &&
      map.cellAt(x, y) == OccupancyMap::kFree && map.distanceAt(x, y) >= p.min_wall_dist;
    if (foreign && !line.empty()) {   // inside the track corridor of the raceline
      const Projection pr = line.project(x, y, hint, 80);
      const RacelinePoint rp = line.at(pr.s);
      foreign = pr.d < rp.w_left - p.corridor_shrink && -pr.d < rp.w_right - p.corridor_shrink;
    }
    if (!foreign) {
      flush();
      continue;
    }
    if (!cluster.empty()) {
      const int j = cluster.back();
      if (std::hypot(x - pts[2 * j], y - pts[2 * j + 1]) > p.cluster_gap) {flush();}
    }
    cluster.push_back(static_cast<int>(i));
  }
  flush();
  return out;
}

void OpponentTracker::update(const std::vector<Obstacle> & obs, double t, double ego_s,
  const Raceline & line, double lookahead_s, double behind_s)
{
  // prediction
  if (tr_.valid) {
    const double dt = t - tr_.last_t;
    if (dt > p_.timeout) {
      tr_ = OpponentTrack();
    }
  }
  const Obstacle * best = nullptr;
  double best_cost = 1e9;
  for (const auto & o : obs) {
    const double ahead = line.ds(ego_s, o.s);
    if (ahead < -behind_s || ahead > lookahead_s) {continue;}
    double cost = ahead;
    if (tr_.valid) {
      const double pred = line.wrapS(tr_.s + tr_.vs * (t - tr_.last_t));
      const double innov = line.ds(pred, o.s);
      if (std::fabs(innov) > p_.assoc_gate) {continue;}
      cost = std::fabs(innov);
    }
    if (cost < best_cost) {
      best_cost = cost;
      best = &o;
    }
  }
  if (!best) {return;}
  if (!tr_.valid) {
    tr_.valid = true;
    tr_.s = best->s;
    tr_.d = best->d;
    tr_.vs = 0.0;
    tr_.hits = 1;
  } else {
    const double dt = std::max(1e-3, t - tr_.last_t);
    const double pred = line.wrapS(tr_.s + tr_.vs * dt);
    const double r = line.ds(pred, best->s);
    tr_.s = line.wrapS(pred + p_.alpha * r);
    tr_.vs += p_.beta * r / dt;
    tr_.d += p_.alpha * (best->d - tr_.d);
    ++tr_.hits;
  }
  tr_.last_t = t;
  tr_.last = *best;
}

}  // namespace race_core
