// Replays a script of timed inputs through the firmware core and prints every
// output. tests/test_core_parity.py runs the same script through the Python
// simulator (robot/sim_robot.py) and requires byte-identical output.
//
// argv: motors_enabled(0|1) sensor_enabled(0|1)
// stdin, one step per line:
//   <ms> CONNECT | <ms> DISCONNECT | <ms> TICK | <ms> OVERFLOW | <ms> DIST <cm> | <ms> LINE <text>
// stdout: R <ack> | E <status/event> | H <hello> | M <left> <right> | A <buzzer> <led>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "robot_core.h"

using namespace sentinel;

struct PrintOutputs : RobotOutputs {
  void setMotors(int l, int r) override { std::printf("M %d %d\n", l, r); }
  void setAlarm(bool b, bool l) override { std::printf("A %d %d\n", b ? 1 : 0, l ? 1 : 0); }
};

int main(int argc, char** argv) {
  PrintOutputs out;
  RobotConfig cfg;
  cfg.motorsEnabled = argc > 1 && std::atoi(argv[1]) == 1;
  cfg.sensorEnabled = argc > 2 && std::atoi(argv[2]) == 1;
  RobotCore core(out, cfg);
  char buf[OUT_LEN];
  std::string line;
  bool begun = false;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    size_t sp = line.find(' ');
    uint32_t t = static_cast<uint32_t>(std::strtoul(line.substr(0, sp).c_str(), nullptr, 10));
    std::string rest = sp == std::string::npos ? "" : line.substr(sp + 1);
    if (!begun) { core.begin(t); begun = true; }
    if (rest == "CONNECT") {
      core.onClientConnected(t, buf, sizeof buf);
      std::printf("H %s\n", buf);
    } else if (rest == "DISCONNECT") {
      core.onClientDisconnected(t);
    } else if (rest == "TICK") {
      core.tick(t);
    } else if (rest == "OVERFLOW") {
      core.handleOverflow(buf, sizeof buf);
      std::printf("R %s\n", buf);
    } else if (rest.rfind("DIST ", 0) == 0) {
      core.onDistance(std::atoi(rest.c_str() + 5), t);
    } else if (rest.rfind("LINE ", 0) == 0) {
      core.handleLine(rest.c_str() + 5, t, buf, sizeof buf);
      std::printf("R %s\n", buf);
    }
    while (core.popMessage(buf, sizeof buf)) std::printf("E %s\n", buf);
  }
  return 0;
}
