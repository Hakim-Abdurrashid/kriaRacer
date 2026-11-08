// LiDAR opponent detection + tracking on the raceline (RACE_PLAN.md §5.2, simplified v1).
//
// Detection: scan points (map frame) that are far from every mapped wall but inside the track
// corridor are "foreign"; consecutive foreign beams are clustered and size-filtered.
// Tracking: alpha-beta filter on the Frenet position (s, d) of the nearest cluster ahead.
// Camera detections (dpu_detector) can confirm a cluster later; racing never depends on them.
#pragma once

#include <vector>

#include "race_core/occupancy_map.hpp"
#include "race_core/raceline.hpp"

namespace race_core
{

struct OpponentParams
{
  double min_wall_dist = 0.25;   // m, a point closer than this to a mapped wall is "wall"
  double corridor_shrink = 0.10; // m, keep points at least this far inside the track edges
  double cluster_gap = 0.20;     // m, max distance between consecutive points of one cluster
  int min_points = 3;
  double min_size = 0.08;        // m, cluster extent limits (an F1Tenth car seen from behind ~0.3 m)
  double max_size = 0.80;
  double max_range = 6.0;        // m from the car
  double assoc_gate = 1.0;       // m in s, track association gate
  double alpha = 0.5;            // alpha-beta gains
  double beta = 0.15;
  double timeout = 0.5;          // s without a detection -> track dropped
};

struct Obstacle
{
  double x = 0, y = 0;   // map frame centroid
  double s = 0, d = 0;   // Frenet on the raceline
  double size = 0;       // m, cluster extent
  int points = 0;
};

struct OpponentTrack
{
  bool valid = false;
  double s = 0, d = 0;   // Frenet position
  double vs = 0;         // m/s along the line
  double last_t = 0;
  int hits = 0;
  Obstacle last;
};

// pts_xy: scan points in the map frame (x0, y0, x1, y1, ...) in beam order; car_x/y in the map frame.
std::vector<Obstacle> detectObstacles(const std::vector<float> & pts_xy, double car_x, double car_y,
  const OccupancyMap & map, const Raceline & line, int hint, const OpponentParams & p);

class OpponentTracker
{
public:
  explicit OpponentTracker(const OpponentParams & p = OpponentParams()) : p_(p) {}
  // ego_s: our arc length. Tracks the nearest obstacle from behind_s behind us to lookahead_s ahead.
  void update(const std::vector<Obstacle> & obs, double t, double ego_s, const Raceline & line,
    double lookahead_s = 6.0, double behind_s = 0.5);
  const OpponentTrack & track() const {return tr_;}
  void reset() {tr_ = OpponentTrack();}

private:
  OpponentParams p_;
  OpponentTrack tr_;
};

}  // namespace race_core
