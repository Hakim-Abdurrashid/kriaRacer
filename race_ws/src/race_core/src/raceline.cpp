#include "race_core/raceline.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include "race_core/geometry.hpp"

namespace race_core
{

bool Raceline::load(const std::string & path, std::string * error)
{
  std::ifstream f(path);
  if (!f) {
    if (error) {*error = "cannot open raceline " + path;}
    return false;
  }
  std::vector<RacelinePoint> pts;
  std::string line;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {line.resize(hash);}
    for (auto & ch : line) {
      if (ch == ',' || ch == ';') {ch = ' ';}
    }
    std::istringstream ss(line);
    std::vector<double> v;
    double x;
    while (ss >> x) {v.push_back(x);}
    if (v.empty()) {continue;}
    if (v.size() < 7) {
      if (error) {*error = "raceline row with fewer than 7 columns in " + path;}
      return false;
    }
    RacelinePoint p;
    p.s = v[0];
    p.x = v[1];
    p.y = v[2];
    p.psi = v[3];
    p.kappa = v[4];
    p.v = v[5];
    p.ax = v[6];
    p.w_left = v.size() > 7 ? v[7] : 0.5;
    p.w_right = v.size() > 8 ? v[8] : 0.5;
    pts.push_back(p);
  }
  if (pts.size() < 10) {
    if (error) {*error = "raceline has fewer than 10 points: " + path;}
    return false;
  }
  setPoints(std::move(pts));
  return true;
}

void Raceline::setPoints(std::vector<RacelinePoint> pts)
{
  pts_ = std::move(pts);
  double s = 0.0;
  for (size_t i = 0; i < pts_.size(); ++i) {
    pts_[i].s = s;
    const auto & a = pts_[i];
    const auto & b = pts_[(i + 1) % pts_.size()];
    s += std::hypot(b.x - a.x, b.y - a.y);
  }
  length_ = s;
}

double Raceline::wrapS(double s) const
{
  s = std::fmod(s, length_);
  return s < 0 ? s + length_ : s;
}

double Raceline::ds(double s_from, double s_to) const
{
  double d = wrapS(s_to - s_from);
  return d >= 0.5 * length_ ? d - length_ : d;
}

Projection Raceline::project(double x, double y, int hint, int window) const
{
  Projection pr;
  if (pts_.empty()) {return pr;}
  const int n = size();
  int best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  const int lo = hint >= 0 ? hint - window : 0;
  const int hi = hint >= 0 ? hint + window : n - 1;
  for (int i = lo; i <= hi; ++i) {
    const auto & p = pts_[wrapIndex(i)];
    const double d2 = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = wrapIndex(i);
    }
  }
  // refine on the segment before or after the nearest point
  const auto & p0 = pts_[best];
  const auto & pn = pts_[wrapIndex(best + 1)];
  const auto & pp = pts_[wrapIndex(best - 1)];
  auto seg = [&](const RacelinePoint & a, const RacelinePoint & b, double & t) {
      const double ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
      t = l2 > 1e-12 ? ((x - a.x) * ex + (y - a.y) * ey) / l2 : 0.0;
      return t;
    };
  double t;
  const RacelinePoint * a = &p0;
  const RacelinePoint * b = &pn;
  if (seg(p0, pn, t) < 0.0) {
    a = &pp;
    b = &p0;
    seg(pp, p0, t);
  }
  t = std::min(1.0, std::max(0.0, t));
  const double ex = b->x - a->x, ey = b->y - a->y, l = std::hypot(ex, ey);
  const double qx = a->x + t * ex, qy = a->y + t * ey;
  pr.index = best;
  pr.s = wrapS(a->s + t * l);
  pr.psi = std::atan2(ey, ex);
  pr.d = l > 1e-9 ? (ex * (y - qy) - ey * (x - qx)) / l : 0.0;
  if (l <= 1e-9) {pr.d = std::hypot(x - qx, y - qy);}
  return pr;
}

RacelinePoint Raceline::at(double s) const
{
  s = wrapS(s);
  // binary search on s (points are sorted by construction)
  int lo = 0, hi = size() - 1;
  while (lo < hi) {
    const int mid = (lo + hi + 1) / 2;
    if (pts_[mid].s <= s) {lo = mid;} else {hi = mid - 1;}
  }
  const auto & a = pts_[lo];
  const auto & b = pts_[wrapIndex(lo + 1)];
  const double seg = (lo + 1 < size()) ? b.s - a.s : length_ - a.s;
  const double t = seg > 1e-9 ? (s - a.s) / seg : 0.0;
  RacelinePoint r;
  r.s = s;
  r.x = a.x + t * (b.x - a.x);
  r.y = a.y + t * (b.y - a.y);
  r.psi = a.psi + t * normalizeAngle(b.psi - a.psi);
  r.kappa = a.kappa + t * (b.kappa - a.kappa);
  r.v = a.v + t * (b.v - a.v);
  r.ax = a.ax + t * (b.ax - a.ax);
  r.w_left = a.w_left + t * (b.w_left - a.w_left);
  r.w_right = a.w_right + t * (b.w_right - a.w_right);
  return r;
}

}  // namespace race_core
