// Raceline loaded from make_raceline.py's CSV, with Frenet projection and interpolation.
#pragma once

#include <string>
#include <vector>

namespace race_core
{

struct RacelinePoint
{
  double s = 0, x = 0, y = 0, psi = 0, kappa = 0, v = 0, ax = 0, w_left = 0, w_right = 0;
};

struct Projection
{
  int index = -1;     // nearest raceline point
  double s = 0.0;     // arc length of the projected point
  double d = 0.0;     // signed lateral offset, left of the line positive
  double psi = 0.0;   // line heading at the projection
};

class Raceline
{
public:
  // CSV columns: s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2[,w_left_m,w_right_m]; '#' starts a comment.
  bool load(const std::string & path, std::string * error);
  void setPoints(std::vector<RacelinePoint> pts);   // recomputes s and the loop length

  bool empty() const {return pts_.empty();}
  int size() const {return static_cast<int>(pts_.size());}
  double length() const {return length_;}
  const RacelinePoint & operator[](int i) const {return pts_[wrapIndex(i)];}
  const std::vector<RacelinePoint> & points() const {return pts_;}

  // Nearest point search. hint >= 0 limits the search to +-window points around hint
  // (cheap tracking); hint < 0 searches the whole line.
  Projection project(double x, double y, int hint = -1, int window = 40) const;
  // Interpolated point at arc length s (wraps around the loop).
  RacelinePoint at(double s) const;
  double wrapS(double s) const;
  // Signed arc distance from s_from forward to s_to in [-L/2, L/2).
  double ds(double s_from, double s_to) const;
  int wrapIndex(int i) const
  {
    const int n = static_cast<int>(pts_.size());
    return ((i % n) + n) % n;
  }

private:
  std::vector<RacelinePoint> pts_;
  double length_ = 0.0;
};

}  // namespace race_core
