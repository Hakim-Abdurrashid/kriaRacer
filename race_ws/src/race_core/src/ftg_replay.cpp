// Offline harness: reads one scan per line as "angle_min,angle_increment,r0,r1,..."
// on stdin and prints "steering,speed" (or "stop") per line.
//   ftg_replay        race_core's port of follow_the_gap_2 (tools/ftg_parity.py)
//   ftg_replay iv     race_core's port of blaze_racer iv2026 follow_the_gap (tools/ftg_iv_parity.py)
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "race_core/follow_gap.hpp"
#include "race_core/follow_gap_iv.hpp"

int main(int argc, char ** argv)
{
  const bool iv = argc > 1 && std::string(argv[1]) == "iv";
  race_core::FollowGap planner;
  race_core::FollowGapIV planner_iv;
  std::string line;
  std::cout.precision(17);
  while (std::getline(std::cin, line)) {
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {vals.push_back(std::stod(tok));}
    if (vals.size() < 4) {continue;}
    std::vector<float> ranges(vals.begin() + 2, vals.end());
    const auto cmd = iv ? planner_iv.compute(ranges, vals[0], vals[1]) :
      planner.compute(ranges, vals[0], vals[1]);
    if (cmd) {
      std::cout << cmd->steering << "," << cmd->speed << "\n";
    } else {
      std::cout << "stop\n";
    }
  }
  return 0;
}
