// Occupancy grid loaded straight from a map_server YAML + image, with a Euclidean distance field.
//
// race_core loads the race-day map itself (no map_server in the loop). The distance field
// (metres to the nearest occupied cell) feeds the particle filter's likelihood field, the
// opponent detector ("is this LiDAR point far from every known wall?") and the simulator.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace race_core
{

class OccupancyMap
{
public:
  enum Cell : int8_t {kUnknown = -1, kFree = 0, kOccupied = 100};

  // Reads a map_server YAML (image, resolution, origin, negate, occupied_thresh, free_thresh).
  // A missing absolute image path falls back to the YAML's directory (maps copied between machines).
  bool load(const std::string & yaml_path, std::string * error);
  // Builds a map from raw cells (row 0 = bottom, ROS convention); used by tests.
  void fromCells(int width, int height, double resolution, double origin_x, double origin_y,
    std::vector<int8_t> cells);

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return res_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}
  bool loaded() const {return width_ > 0;}

  bool worldToCell(double x, double y, int & cx, int & cy) const
  {
    cx = static_cast<int>((x - ox_) * inv_res_);
    cy = static_cast<int>((y - oy_) * inv_res_);
    return x >= ox_ && y >= oy_ && cx < width_ && cy < height_;
  }
  int8_t cell(int cx, int cy) const {return cells_[cy * width_ + cx];}
  int8_t cellAt(double x, double y) const
  {
    int cx, cy;
    return worldToCell(x, y, cx, cy) ? cell(cx, cy) : static_cast<int8_t>(kUnknown);
  }
  // Metres to the nearest occupied cell; outside the map returns 0 (treated as a wall).
  float distanceAt(double x, double y) const
  {
    int cx, cy;
    return worldToCell(x, y, cx, cy) ? dist_[cy * width_ + cx] : 0.0f;
  }
  const std::vector<float> & distanceField() const {return dist_;}
  const std::vector<int8_t> & cells() const {return cells_;}

private:
  void computeDistanceField();

  int width_ = 0;
  int height_ = 0;
  double res_ = 0.05;
  double inv_res_ = 20.0;
  double ox_ = 0.0;
  double oy_ = 0.0;
  std::vector<int8_t> cells_;
  std::vector<float> dist_;
};

}  // namespace race_core
