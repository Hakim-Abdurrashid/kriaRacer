// Offline harness: reads one scan per line as "angle_min,angle_increment,r0,r1,..."
// on stdin and prints "steering,speed" (or "stop") per line. Used by
// tools/ftg_parity.py to compare against the Python follow_the_gap_2.
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "race_core/follow_gap.hpp"

int main()
{
  race_core::FollowGap planner;
  std::string line;
  std::cout.precision(9);
  while (std::getline(std::cin, line)) {
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {vals.push_back(std::stod(tok));}
    if (vals.size() < 4) {continue;}
    std::vector<float> ranges(vals.begin() + 2, vals.end());
    const auto cmd = planner.compute(ranges, vals[0], vals[1]);
    if (cmd) {
      std::cout << cmd->steering << "," << cmd->speed << "\n";
    } else {
      std::cout << "stop\n";
    }
  }
  return 0;
}
