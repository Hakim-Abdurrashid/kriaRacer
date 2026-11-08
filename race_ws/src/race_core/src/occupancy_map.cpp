#include "race_core/occupancy_map.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>

namespace race_core
{

namespace
{

// Felzenszwalb & Huttenlocher 1D squared Euclidean distance transform. Empty samples carry kBig.
constexpr double kBig = 1e20;

void edt1d(const double * f, double * d, int n, std::vector<int> & v, std::vector<double> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kBig;
  z[1] = kBig;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
    while (s <= z[k]) {
      --k;
      s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kBig;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

bool OccupancyMap::load(const std::string & yaml_path, std::string * error)
{
  namespace fs = std::filesystem;
  YAML::Node y;
  try {
    y = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    if (error) {*error = std::string("cannot read ") + yaml_path + ": " + e.what();}
    return false;
  }
  fs::path img = y["image"].as<std::string>();
  const fs::path dir = fs::path(yaml_path).parent_path();
  if (img.is_relative()) {img = dir / img;}
  if (!fs::exists(img)) {img = dir / img.filename();}
  cv::Mat m = cv::imread(img.string(), cv::IMREAD_GRAYSCALE);
  if (m.empty()) {
    if (error) {*error = "cannot read map image " + img.string();}
    return false;
  }
  const double res = y["resolution"].as<double>();
  const auto origin = y["origin"].as<std::vector<double>>();
  const bool negate = y["negate"] ? y["negate"].as<int>() != 0 : false;
  const double occ_th = y["occupied_thresh"] ? y["occupied_thresh"].as<double>() : 0.65;
  const double free_th = y["free_thresh"] ? y["free_thresh"].as<double>() : 0.25;

  std::vector<int8_t> cells(static_cast<size_t>(m.rows) * m.cols);
  for (int r = 0; r < m.rows; ++r) {
    const uint8_t * row = m.ptr<uint8_t>(r);
    const int cy = m.rows - 1 - r;    // image row 0 is the top of the map
    for (int c = 0; c < m.cols; ++c) {
      const double p = row[c] / 255.0;
      const double occ = negate ? p : 1.0 - p;
      cells[cy * m.cols + c] = occ > occ_th ? kOccupied : (occ < free_th ? kFree : kUnknown);
    }
  }
  fromCells(m.cols, m.rows, res, origin.at(0), origin.at(1), std::move(cells));
  return true;
}

void OccupancyMap::fromCells(int width, int height, double resolution, double origin_x,
  double origin_y, std::vector<int8_t> cells)
{
  width_ = width;
  height_ = height;
  res_ = resolution;
  inv_res_ = 1.0 / resolution;
  ox_ = origin_x;
  oy_ = origin_y;
  cells_ = std::move(cells);
  computeDistanceField();
}

void OccupancyMap::computeDistanceField()
{
  const int W = width_, H = height_, n = std::max(W, H);
  std::vector<double> f(static_cast<size_t>(W) * H);
  for (size_t i = 0; i < f.size(); ++i) {f[i] = cells_[i] == kOccupied ? 0.0 : kBig;}
  std::vector<int> v(n);
  std::vector<double> z(n + 1), col(n), out(n);
  for (int x = 0; x < W; ++x) {         // columns
    for (int yy = 0; yy < H; ++yy) {col[yy] = f[yy * W + x];}
    edt1d(col.data(), out.data(), H, v, z);
    for (int yy = 0; yy < H; ++yy) {f[yy * W + x] = out[yy];}
  }
  dist_.assign(f.size(), 0.0f);
  for (int yy = 0; yy < H; ++yy) {      // rows
    edt1d(&f[yy * W], out.data(), W, v, z);
    for (int x = 0; x < W; ++x) {
      dist_[yy * W + x] = out[x] >= 1e19 ? 1e3f : static_cast<float>(std::sqrt(out[x]) * res_);
    }
  }
}

}  // namespace race_core
