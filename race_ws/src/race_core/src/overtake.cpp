#include "race_core/overtake.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace race_core
{

namespace
{
// smooth 0 -> 1 over [0, 1]
double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * t);
}
}  // namespace

OvertakePlan planOvertake(const Raceline & line, double s_ego, double v_ego, const OpponentTrack & opp,
  int side, const OvertakeParams & p, const OvertakePlan * prev)
{
  OvertakePlan plan;
  if (line.empty() || !opp.valid) {return plan;}
  const double s_opp = opp.s;                       // opponent centre
  const double ahead = line.ds(s_ego, s_opp);
  // manoeuvre window along the line
  plan.s_full = line.wrapS(s_opp - 0.5 * p.opp_length - 0.3);
  plan.s_start = line.wrapS(line.ds(s_ego, plan.s_full) > p.ramp_in ? plan.s_full - p.ramp_in : s_ego);
  if (prev && prev->side != 0) {
    plan.s_start = prev->s_start;
    plan.s_full = prev->s_full;
  }
  // where we will be when we are hold_after past the opponent's front (constant speeds)
  const double dv = std::max(0.3, v_ego - opp.vs);
  const double t_pass = (std::max(0.0, ahead) + p.opp_length + p.hold_after) / dv;
  plan.s_release = line.wrapS(s_opp + std::max(0.0, opp.vs) * t_pass + 0.5 * p.opp_length + p.hold_after);
  plan.s_end = line.wrapS(plan.s_release + p.ramp_out);
  if (ahead > 8.0 || line.ds(s_ego, plan.s_end) > p.max_length || line.ds(s_ego, plan.s_end) < 0.0) {
    return plan;
  }

  const double offset = p.opp_half_width + p.clearance + p.car_half_width;
  double best_margin = -1e9;
  for (int sd : {+1, -1}) {
    if (side != 0 && sd != side) {continue;}
    const double d_target = opp.d + sd * offset;
    // worst-case room over the part of the manoeuvre at full offset (+ the ramps, scaled)
    double margin = 1e9;
    const double len = line.ds(plan.s_start, plan.s_end);
    for (double u = 0.0; u <= len; u += 0.1) {
      const double s = line.wrapS(plan.s_start + u);
      const RacelinePoint rp = line.at(s);
      double frac = 1.0;
      const double into = line.ds(plan.s_start, s);
      if (into < line.ds(plan.s_start, plan.s_full)) {
        frac = smoothstep(into / std::max(1e-3, line.ds(plan.s_start, plan.s_full)));
      } else if (into > line.ds(plan.s_start, plan.s_release)) {
        frac = 1.0 - smoothstep((into - line.ds(plan.s_start, plan.s_release)) / std::max(1e-3, p.ramp_out));
      }
      const double d = frac * d_target;
      const double room = sd > 0 ? rp.w_left - d : rp.w_right + d;   // centre to wall on our side
      margin = std::min(margin, room - p.car_half_width - p.wall_margin);
    }
    if (margin > best_margin) {
      best_margin = margin;
      plan.side = sd;
      plan.d_target = d_target;
    }
  }
  plan.margin = best_margin;
  plan.feasible = best_margin >= 0.0;
  return plan;
}

double overtakeOffset(const Raceline & line, const OvertakePlan & plan, double s)
{
  if (plan.side == 0) {return 0.0;}
  const double into = line.ds(plan.s_start, s);
  const double full = line.ds(plan.s_start, plan.s_full);
  const double rel = line.ds(plan.s_start, plan.s_release);
  const double end = line.ds(plan.s_start, plan.s_end);
  if (into < 0.0 || into > end) {return 0.0;}
  if (into < full) {return plan.d_target * smoothstep(into / std::max(1e-3, full));}
  if (into <= rel) {return plan.d_target;}
  return plan.d_target * (1.0 - smoothstep((into - rel) / std::max(1e-3, end - rel)));
}

Raceline shiftedLine(const Raceline & line, const OvertakePlan & plan)
{
  std::vector<RacelinePoint> pts = line.points();
  for (auto & q : pts) {
    const double d = overtakeOffset(line, plan, q.s);
    if (d == 0.0) {continue;}
    q.x -= std::sin(q.psi) * d;
    q.y += std::cos(q.psi) * d;
    q.w_left -= d;
    q.w_right += d;
  }
  Raceline out;
  out.setPoints(std::move(pts));
  return out;
}

}  // namespace race_core
