// Spline-style overtaking (after ForzaETH's "spliner"): a lateral offset d(s) from the raceline with
// smooth cosine ramps, on the side of the opponent with more room, held until we are past it.
//
// The controller keeps following a closed line: the raceline shifted by d(s) along its normals.
// Feasibility uses the raceline's w_left / w_right (wall distances from make_raceline.py) at every
// point of the manoeuvre, so a plan never asks the car to go closer than wall_margin to a wall.
#pragma once

#include "race_core/opponent.hpp"
#include "race_core/raceline.hpp"

namespace race_core
{

struct OvertakeParams
{
  double clearance = 0.15;       // m, side-to-side gap to the opponent
  double car_half_width = 0.15;  // m
  double opp_half_width = 0.15;  // m
  double opp_length = 0.50;      // m
  double wall_margin = 0.10;     // m, our side to the wall
  double ramp_in = 2.0;          // m of arc to move over
  double ramp_out = 2.0;         // m of arc to move back
  double hold_after = 0.8;       // m past the opponent's front before moving back
  double max_length = 14.0;      // m, give up if passing would take longer than this
};

struct OvertakePlan
{
  bool feasible = false;
  int side = 0;            // +1 pass on the left, -1 on the right
  double d_target = 0.0;   // lateral offset from the raceline while alongside (m)
  double s_start = 0.0;    // offset starts (wrapped arc lengths)
  double s_full = 0.0;     // full offset reached
  double s_release = 0.0;  // start moving back
  double s_end = 0.0;      // back on the raceline
  double margin = 0.0;     // spare lateral room on the chosen side (m)
};

// side: 0 = choose, +1/-1 = force that side. prev: the plan of an overtake in progress; its start
// and full-offset points are kept so re-planning never pulls the car back mid-manoeuvre.
// v_ego: our planned speed; with the opponent's speed it predicts where we will be past it, so the
// whole manoeuvre (not just the current stretch) must fit.
OvertakePlan planOvertake(const Raceline & line, double s_ego, double v_ego, const OpponentTrack & opp,
  int side, const OvertakeParams & p, const OvertakePlan * prev = nullptr);

// Lateral offset of the plan at arc length s (0 outside the manoeuvre).
double overtakeOffset(const Raceline & line, const OvertakePlan & plan, double s);

// The raceline with the plan's offset applied (same number of points, s recomputed).
Raceline shiftedLine(const Raceline & line, const OvertakePlan & plan);

}  // namespace race_core
