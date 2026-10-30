// PC simulator of the robot: the REAL firmware core (robot_core.cpp) behind a
// TCP server, so the laptop software can be tested against the exact ESP32
// logic without hardware. Linux / macOS (POSIX sockets).
//
// Build:  g++ -std=c++17 -O2 -Wall -Wextra -I../sentinel_robot robot_sim.cpp ../sentinel_robot/robot_core.cpp -o robot_sim
// Run:    ./robot_sim [port] [motors_enabled 0|1] [sensor_enabled 0|1]      (default 3333 0 0)
// With the sensor enabled, type a distance in cm on stdin (e.g. "20") to simulate
// the ultrasonic sensor; "-1" means no echo.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "robot_core.h"

using namespace sentinel;

static uint32_t nowMs() {
  using namespace std::chrono;
  static const auto t0 = steady_clock::now();
  return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - t0).count());
}

struct PrintOutputs : RobotOutputs {
  void setMotors(int l, int r) override { std::printf("[hw] motors left=%d right=%d\n", l, r); std::fflush(stdout); }
  void setAlarm(bool b, bool l) override { std::printf("[hw] buzzer=%d led=%d\n", b, l); std::fflush(stdout); }
};

static void sendLine(int fd, const char* s) {
  if (fd < 0) return;
  char buf[OUT_LEN + 2];
  int n = std::snprintf(buf, sizeof buf, "%s\n", s);
  (void)!send(fd, buf, static_cast<size_t>(n), MSG_NOSIGNAL);
}

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  int port = argc > 1 ? std::atoi(argv[1]) : 3333;
  PrintOutputs out;
  RobotConfig cfg;
  cfg.motorsEnabled = argc > 2 && std::atoi(argv[2]) == 1;
  cfg.sensorEnabled = argc > 3 && std::atoi(argv[3]) == 1;
  RobotCore core(out, cfg);
  core.begin(nowMs());

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || listen(srv, 2) != 0) {
    std::perror("bind/listen");
    return 1;
  }
  std::printf("[sim] robot simulator (firmware core, protocol v%d) listening on 127.0.0.1:%d\n",
              PROTOCOL_VERSION, port);
  std::fflush(stdout);

  int cli = -1;
  LineReader reader;
  char buf[OUT_LEN];
  char stdinBuf[64];
  size_t stdinLen = 0;
  bool stdinOpen = cfg.sensorEnabled;
  for (;;) {
    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(srv, &rf);
    int maxfd = srv;
    if (cli >= 0) { FD_SET(cli, &rf); if (cli > maxfd) maxfd = cli; }
    if (stdinOpen) FD_SET(0, &rf);
    timeval tv{0, 5000};   // 5 ms loop like the ESP32 loop()
    select(maxfd + 1, &rf, nullptr, nullptr, &tv);

    if (stdinOpen && FD_ISSET(0, &rf)) {      // simulated ultrasonic readings
      char c;
      ssize_t n = read(0, &c, 1);
      if (n <= 0) {
        stdinOpen = false;
      } else if (c == '\n') {
        stdinBuf[stdinLen] = '\0';
        core.onDistance(std::atoi(stdinBuf), nowMs());
        stdinLen = 0;
      } else if (stdinLen < sizeof stdinBuf - 1) {
        stdinBuf[stdinLen++] = c;
      }
    }
    if (FD_ISSET(srv, &rf)) {
      int nc = accept(srv, nullptr, nullptr);
      if (nc >= 0) {
        if (cli >= 0) {                          // newest connection wins
          sendLine(cli, "{\"version\":3,\"type\":\"event\",\"event\":\"replaced\"}");
          close(cli);
          core.onClientDisconnected(nowMs());
        }
        cli = nc;
        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
        reader.reset();
        core.onClientConnected(nowMs(), buf, sizeof buf);
        sendLine(cli, buf);
        std::printf("[sim] laptop connected\n");
        std::fflush(stdout);
      }
    }
    if (cli >= 0 && FD_ISSET(cli, &rf)) {
      char data[512];
      ssize_t n = recv(cli, data, sizeof data, 0);
      if (n <= 0) {
        close(cli);
        cli = -1;
        core.onClientDisconnected(nowMs());
        std::printf("[sim] laptop disconnected - motors stopped\n");
        std::fflush(stdout);
      } else {
        for (ssize_t i = 0; i < n; ++i) {
          int r = reader.feed(data[i]);
          if (r == 1) {
            core.handleLine(reader.line(), nowMs(), buf, sizeof buf);
            sendLine(cli, buf);
          } else if (r == -1) {
            core.handleOverflow(buf, sizeof buf);
            sendLine(cli, buf);
          }
        }
      }
    }
    core.tick(nowMs());
    while (core.popMessage(buf, sizeof buf)) {
      if (std::strstr(buf, "\"type\":\"status\"") == nullptr) {
        std::printf("[sim] %s\n", buf);
        std::fflush(stdout);
      }
      sendLine(cli, buf);
    }
  }
}
