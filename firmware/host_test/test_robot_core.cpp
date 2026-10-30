// Host (PC) unit tests for the ESP32 robot core (protocol v3, JSON). No hardware needed.
// Build and run (from this folder):
//   g++ -std=c++17 -Wall -Wextra -Werror -I../sentinel_robot test_robot_core.cpp
//       ../sentinel_robot/robot_core.cpp -o test_robot_core   (one line)
//   ./test_robot_core
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "robot_core.h"

using namespace sentinel;

static int g_failed = 0, g_passed = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    if (cond) { ++g_passed; }                                                    \
    else { ++g_failed; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static bool has(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

struct FakeOutputs : RobotOutputs {
  int left = 99, right = 99;
  bool buzzer = true, led = true;
  std::vector<std::string> log;
  void setMotors(int l, int r) override { left = l; right = r; log.push_back("M" + std::to_string(l) + "," + std::to_string(r)); }
  void setAlarm(bool b, bool l) override { buzzer = b; led = l; log.push_back(std::string("A") + (b ? "1" : "0") + (l ? "1" : "0")); }
};

struct Rig {
  FakeOutputs out;
  RobotConfig cfg;
  RobotCore core;
  char buf[OUT_LEN];
  long seq = 0;
  explicit Rig(bool motors = true, bool sensor = false) : cfg(make(motors, sensor)), core(out, cfg) {
    core.begin(0);
    core.onClientConnected(0, buf, sizeof buf);
  }
  static RobotConfig make(bool motors, bool sensor) {
    RobotConfig c;
    c.motorsEnabled = motors;
    c.sensorEnabled = sensor;
    return c;
  }
  std::string raw(const std::string& line, uint32_t now) {
    core.handleLine(line.c_str(), now, buf, sizeof buf);
    return buf;
  }
  // send a well-formed command with the next sequence number
  std::string cmd(const char* type, const std::string& extra, uint32_t now) {
    ++seq;
    std::string line = "{\"version\":3,\"type\":\"" + std::string(type) + "\",\"command_id\":\"c" +
                       std::to_string(seq) + "\",\"seq\":" + std::to_string(seq) + (extra.empty() ? "" : "," + extra) + "}";
    return raw(line, now);
  }
  std::vector<std::string> msgs() {
    std::vector<std::string> v;
    char m[OUT_LEN];
    while (core.popMessage(m, sizeof m)) v.push_back(m);
    return v;
  }
  std::vector<std::string> events() {      // messages without the periodic status
    std::vector<std::string> v;
    for (auto& m : msgs())
      if (m.find("\"type\":\"status\"") == std::string::npos) v.push_back(m);
    return v;
  }
  bool okLast(const std::string& reply) const {   // reply is the plain OK ack of the latest command
    return reply == "{\"version\":3,\"type\":\"ack\",\"command_id\":\"c" + std::to_string(seq) +
                    "\",\"seq\":" + std::to_string(seq) + ",\"ok\":true}";
  }
  std::string status(uint32_t now) {
    core.buildStatus(now, buf, sizeof buf);
    return buf;
  }
};

static std::string ok(long seq) {
  return "{\"version\":3,\"type\":\"ack\",\"command_id\":\"c" + std::to_string(seq) + "\",\"seq\":" +
         std::to_string(seq) + ",\"ok\":true}";
}

static void test_json_parser() {
  JObject o;
  CHECK(parseJsonObject("{}", o) && o.n == 0);
  CHECK(parseJsonObject(" { \"a\" : 1 , \"b\":\"x y\",\"c\":true,\"d\":false,\"e\":null,\"f\":-0.25 } ", o) && o.n == 6);
  CHECK(o.get("a")->type == J_INT && o.get("a")->i == 1);
  CHECK(o.get("b")->type == J_STR && std::string(o.get("b")->s) == "x y");
  CHECK(o.get("c")->b && !o.get("d")->b && o.get("e")->type == J_NULL);
  CHECK(o.get("f")->type == J_NUM && o.get("f")->d == -0.25);
  const char* bad[] = {"", "[]", "{", "{\"a\":1,}", "{\"a\":01}", "{\"a\":1.}", "{\"a\":.5}", "{\"a\":1e3}",
                       "{\"a\":{}}", "{\"a\":[1]}", "{\"a\":1,\"a\":2}", "{\"a\":\"x\\\"y\"}", "{\"\":1}",
                       "{\"a\":tru}", "{\"a\":1}x", "{a:1}", "{\"a\":1234567890}", "{\"a\":0.1234567}",
                       "{\"a\":\"012345678901234567890123456789012\"}", "{\"a\":1 \"b\":2}", "{\"a\":--1}",
                       "{\"a\":1,\"b\":2,\"c\":3,\"d\":4,\"e\":5,\"f\":6,\"g\":7,\"h\":8,\"i\":9,\"j\":10,\"k\":11,\"l\":12,\"m\":13}"};
  for (const char* b : bad) CHECK(!parseJsonObject(b, o));
  CHECK(parseJsonObject("{\"a\":999999999,\"b\":-999999999,\"c\":0,\"d\":0.000001}", o));
}

static void test_boot_hello_and_status() {
  Rig r;
  CHECK(r.out.left == 0 && r.out.right == 0 && !r.out.buzzer && !r.out.led);
  CHECK(std::string(r.buf) ==
        "{\"version\":3,\"type\":\"event\",\"event\":\"hello\",\"protocol\":3,\"state\":\"idle\","
        "\"motors_enabled\":true,\"sensor_installed\":false,\"led_installed\":true,\"buzzer_installed\":true,"
                                "\"comm_fault\":false,\"watchdog_trips\":0}");
  CHECK(r.status(2500) ==
        "{\"version\":3,\"type\":\"status\",\"state\":\"idle\",\"motors_stopped\":true,\"left\":0,\"right\":0,"
        "\"motors_enabled\":true,\"follow_mode\":false,\"target_id\":null,\"emergency_active\":false,"
        "\"emergency_reason\":null,\"obstacle_detected\":false,\"obstacle_source\":\"none\","
        "\"sensor_installed\":false,\"distance_cm\":null,\"watchdog_ok\":true,\"watchdog_trips\":0,"
        "\"led\":\"off\",\"buzzer\":\"off\",\"simulated_movement\":\"none\",\"model_running\":false,"
                                "\"fall_alert\":false,\"last_command_id\":null,\"uptime_s\":2}");
  CHECK(r.cmd("ping", "", 10) == ok(1));
  CHECK(r.cmd("status_request", "", 20) == ok(2));
  auto m = r.msgs();
  CHECK(m.size() == 1 && has(m[0], "\"type\":\"status\"") && has(m[0], "\"last_command_id\":\"c2\""));
  r.core.tick(900);
  CHECK(r.msgs().empty());
  r.core.tick(1020);                                     // periodic status 1 s after the last one
  m = r.msgs();
  CHECK(m.size() == 1 && has(m[0], "\"type\":\"status\""));
  r.core.onClientDisconnected(1100);
  r.core.tick(5000);
  CHECK(r.msgs().empty());                               // no status without a laptop
}

static void test_common_field_validation() {
  Rig r;
  CHECK(r.raw("not json", 1) == "{\"version\":3,\"type\":\"ack\",\"command_id\":null,\"seq\":0,\"ok\":false,\"error\":\"BAD_JSON\"}");
  CHECK(has(r.raw("{\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1}", 1), "\"error\":\"MISSING_FIELD\",\"field\":\"version\""));
  CHECK(has(r.raw("{\"version\":2,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1}", 1), "\"command_id\":\"a\",\"seq\":1,\"ok\":false,\"error\":\"BAD_VERSION\""));
  CHECK(has(r.raw("{\"version\":\"3\",\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1}", 1), "BAD_VERSION"));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"seq\":1}", 1), "\"command_id\":null,\"seq\":1,\"ok\":false,\"error\":\"MISSING_FIELD\",\"field\":\"command_id\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"bad id\",\"seq\":1}", 1), "\"field\":\"command_id\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\"}", 1), "\"field\":\"seq\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":0}", 1), "BAD_FIELD\",\"field\":\"seq\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1.5}", 1), "BAD_FIELD\",\"field\":\"seq\""));
  CHECK(has(r.raw("{\"version\":3,\"command_id\":\"a\",\"seq\":1}", 1), "\"field\":\"type\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"fly\",\"command_id\":\"a\",\"seq\":2}", 1), "UNKNOWN_TYPE"));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":2}", 1), "BAD_SEQ"));      // duplicate
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1}", 1), "BAD_SEQ"));      // stale
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":3,\"x\":1}", 1), "UNKNOWN_FIELD\",\"field\":\"x\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":9}", 1), "\"ok\":true"));
  std::string longLine(250, 'x');
  CHECK(has(r.raw(longLine, 1), "TOO_LONG"));
  r.core.handleOverflow(r.buf, sizeof r.buf);
  CHECK(has(r.buf, "TOO_LONG"));
  CHECK(r.core.rejected() == 16);
  char hello[OUT_LEN];
  r.core.onClientConnected(2, hello, sizeof hello);     // new connection: sequence restarts
  CHECK(has(r.raw("{\"version\":3,\"type\":\"ping\",\"command_id\":\"a\",\"seq\":1}", 3), "\"ok\":true"));
}

static void test_motor_commands() {
  Rig r;
  const std::string base = "\"mode\":\"manual\",";
  CHECK(r.cmd("motor", base + "\"left\":40,\"right\":40,\"duration_ms\":300", 100) ==
        "{\"version\":3,\"type\":\"ack\",\"command_id\":\"c1\",\"seq\":1,\"ok\":true,\"left\":40,\"right\":40}");
  CHECK(r.out.left == 40 && r.out.right == 40 && r.core.state() == STATE_MOVING);
  CHECK(has(r.cmd("motor", base + "\"left\":-90,\"right\":100,\"duration_ms\":300", 110), "\"left\":-60,\"right\":60"));  // clamp
  CHECK(r.out.left == -60 && r.out.right == 60);
  CHECK(r.cmd("stop", "", 120) == ok(3));
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE);
  CHECK(has(r.cmd("motor", base + "\"left\":0,\"right\":0,\"duration_ms\":300", 130), "\"ok\":true"));
  CHECK(r.core.state() == STATE_IDLE);
  struct { const char* extra; const char* code; } bad[] = {
      {"\"mode\":\"auto\",\"left\":1,\"right\":1,\"duration_ms\":300", "\"field\":\"mode\""},
      {"\"mode\":\"manual\",\"left\":101,\"right\":1,\"duration_ms\":300", "\"field\":\"left\""},
      {"\"mode\":\"manual\",\"left\":1,\"right\":-101,\"duration_ms\":300", "\"field\":\"right\""},
      {"\"mode\":\"manual\",\"left\":1,\"right\":1,\"duration_ms\":0", "\"field\":\"duration_ms\""},
      {"\"mode\":\"manual\",\"left\":1,\"right\":1,\"duration_ms\":2001", "\"field\":\"duration_ms\""},
      {"\"mode\":\"manual\",\"left\":1.5,\"right\":1,\"duration_ms\":300", "\"field\":\"left\""},
      {"\"mode\":\"manual\",\"left\":\"1\",\"right\":1,\"duration_ms\":300", "\"field\":\"left\""},
      {"\"mode\":\"manual\",\"left\":1,\"duration_ms\":300", "MISSING_FIELD\",\"field\":\"right\""},
      {"\"mode\":\"manual\",\"left\":1,\"right\":1,\"duration_ms\":300,\"speed\":5", "UNKNOWN_FIELD\",\"field\":\"speed\""},
  };
  for (auto& b : bad) CHECK(has(r.cmd("motor", b.extra, 140), b.code));
  CHECK(r.core.state() == STATE_IDLE && r.out.left == 0);   // invalid values never reach the outputs
}

static void test_motor_outputs_disabled_in_phase2() {
  Rig r(false);
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":40,\"right\":30,\"duration_ms\":300", 1), "\"ok\":true"));
  CHECK(r.out.left == 0 && r.out.right == 0);    // pins never driven
  CHECK(r.core.left() == 40 && has(r.status(2), "\"motors_stopped\":false"));   // logical state tracked
}

static void test_watchdog_and_comm_fault() {
  Rig r;
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":30,\"duration_ms\":2000", 1000), "\"ok\":true"));
  r.core.tick(1400);
  CHECK(r.events().empty());
  CHECK(r.cmd("ping", "", 1450) == ok(2));            // heartbeat feeds the watchdog
  r.core.tick(1900);
  CHECK(r.events().empty());
  CHECK(has(r.cmd("fly", "", 1920), "UNKNOWN_TYPE"));  // invalid does NOT feed it
  r.core.tick(1951);
  auto m = r.events();
  CHECK(m.size() == 1 && m[0] == "{\"version\":3,\"type\":\"event\",\"event\":\"watchdog_stop\"}");
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE && r.core.watchdogTrips() == 1);
  CHECK(r.core.commFault() && has(r.status(1960), "\"watchdog_ok\":false"));
  CHECK(r.cmd("ping", "", 1970) == ok(4));
  CHECK(r.core.commFault());                           // a ping is not a fresh control state
  CHECK(r.cmd("stop", "", 1980) == ok(5));
  CHECK(!r.core.commFault());
}

static void test_disconnect_reports_comm_fault_and_never_resumes() {
  Rig r;
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":4", 0), "\"ok\":true"));
  CHECK(has(r.cmd("follow", "\"target_id\":4,\"left\":30,\"right\":30,\"duration_ms\":400,\"confidence\":0.9", 10), "\"ok\":true"));
  r.core.onClientDisconnected(20);
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE && !r.core.followMode());
  char hello[OUT_LEN];
  r.core.onClientConnected(500, hello, sizeof hello);
  CHECK(has(hello, "\"comm_fault\":true"));
  r.core.tick(900);
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE);   // reconnect does not resume anything
  r.seq = 0;
  CHECK(has(r.cmd("follow", "\"target_id\":4,\"left\":30,\"right\":30,\"duration_ms\":400,\"confidence\":0.9", 950), "FOLLOW_NOT_ENABLED"));
}

static void test_move_duration_expires() {
  Rig r;
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":20,\"right\":25,\"duration_ms\":300", 1000), "\"ok\":true"));
  CHECK(r.cmd("ping", "", 1200) == ok(2));
  r.core.tick(1299);
  CHECK(r.events().empty());
  r.core.tick(1300);
  auto m = r.events();
  CHECK(m.size() == 1 && has(m[0], "move_done") && r.out.left == 0);
}

static void test_emergency_latch_and_clear() {
  Rig r;
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":1", 0), "\"ok\":true"));
  CHECK(has(r.cmd("follow", "\"target_id\":1,\"left\":30,\"right\":30,\"duration_ms\":2000,\"confidence\":1", 10), "\"ok\":true"));
  r.core.tick(200);
  r.out.log.clear();
  CHECK(r.cmd("emergency_stop", "\"reason\":\"fall_event\"", 210) == ok(3));
  CHECK(r.out.log.size() >= 2 && r.out.log[0] == "M0,0" && r.out.log[1] == "A11");   // stop BEFORE alarm
  CHECK(r.core.state() == STATE_EMERGENCY && !r.core.followMode());
  std::string st = r.status(220);
  CHECK(has(st, "\"state\":\"emergency\"") && has(st, "\"emergency_reason\":\"fall_event\"") && has(st, "\"buzzer\":\"emergency\""));
  const char* refused[][2] = {{"motor", "\"mode\":\"manual\",\"left\":-30,\"right\":-30,\"duration_ms\":300"},
                              {"follow_mode", "\"enabled\":true,\"target_id\":1"},
                              {"follow", "\"target_id\":1,\"left\":0,\"right\":0,\"duration_ms\":300,\"confidence\":1"}};
  for (auto& c : refused) CHECK(has(r.cmd(c[0], c[1], 230), "EMERGENCY_ACTIVE"));
  CHECK(has(r.cmd("emergency_stop", "\"reason\":\"bad reason!\"", 240), "\"field\":\"reason\""));
  CHECK(r.okLast(r.cmd("emergency_stop", "\"reason\":\"again\"", 250)));   // repeat keeps the first reason
  CHECK(has(r.status(260), "\"emergency_reason\":\"fall_event\""));
  // buzzer pattern 300 on / 200 off
  r.core.tick(210 + 100); CHECK(r.out.buzzer && r.out.led);
  r.core.tick(210 + 350); CHECK(!r.out.buzzer && r.out.led);
  r.core.tick(210 + 600); CHECK(r.out.buzzer);
  CHECK(r.okLast(r.cmd("mute", "", 900)));
  r.core.tick(1010); CHECK(!r.out.buzzer && r.out.led && r.core.state() == STATE_EMERGENCY);
  CHECK(has(r.cmd("clear_emergency", "\"operator_confirmed\":false", 1100), "NEEDS_CONFIRM"));
  CHECK(has(r.cmd("clear_emergency", "\"operator_confirmed\":1", 1110), "\"field\":\"operator_confirmed\""));
  CHECK(r.core.state() == STATE_EMERGENCY);
  CHECK(r.okLast(r.cmd("clear_emergency", "\"operator_confirmed\":true", 1120)));
  CHECK(r.core.state() == STATE_IDLE && !r.out.buzzer && !r.out.led && r.out.left == 0);
  r.core.tick(1500);
  CHECK(r.out.left == 0 && !r.core.followMode());       // clearing never resumes movement
}

static void test_emergency_buzzer_is_bounded() {
  Rig r;
  CHECK(r.cmd("emergency_stop", "\"reason\":\"test\"", 1000) == ok(1));
  r.core.tick(1000 + 59900);
  CHECK(r.msgs().size() >= 1);                            // periodic status messages only
  r.core.tick(1000 + 60000);
  auto m = r.msgs();
  bool timeout = false;
  for (auto& s : m) timeout |= has(s, "buzzer_timeout");
  CHECK(timeout && !r.out.buzzer && r.out.led);           // buzzer stops, LED stays on
  r.core.tick(1000 + 61000);
  m = r.msgs();
  timeout = false;
  for (auto& s : m) timeout |= has(s, "buzzer_timeout");
  CHECK(!timeout);                                         // reported only once
  CHECK(r.cmd("buzzer", "\"pattern\":\"emergency\"", 70000) == ok(2));   // manual pattern bounded too
  CHECK(r.cmd("clear_emergency", "\"operator_confirmed\":true", 70010) == ok(3));
  CHECK(r.cmd("buzzer", "\"pattern\":\"emergency\"", 70020) == ok(4));
  r.core.tick(70020 + 59700); CHECK(r.out.buzzer);                 // still sounding just before the limit
  r.core.tick(70020 + 60000);
  CHECK(!r.out.buzzer && has(r.status(70020 + 60001), "\"buzzer\":\"off\""));
}

static void test_led_patterns() {
  Rig r;
  CHECK(r.cmd("led", "\"state\":\"on\"", 0) == ok(1));
  CHECK(r.out.led);
  CHECK(r.cmd("led", "\"state\":\"blink\",\"interval_ms\":250", 1000) == ok(2));   // 250 on / 250 off
  r.core.tick(1010); CHECK(r.out.led);
  r.core.tick(1260); CHECK(!r.out.led);
  r.core.tick(1510); CHECK(r.out.led);
  CHECK(r.cmd("led", "\"state\":\"blink\"", 2000) == ok(3));                        // default 500
  r.core.tick(2100); CHECK(r.out.led);
  r.core.tick(2600); CHECK(!r.out.led);
  CHECK(has(r.status(2600), "\"led\":\"blink\""));
  CHECK(r.cmd("led", "\"state\":\"off\"", 2700) == ok(4));
  r.core.tick(3000); CHECK(!r.out.led);
  CHECK(has(r.cmd("led", "\"state\":\"red\"", 3100), "\"field\":\"state\""));
  CHECK(has(r.cmd("led", "\"state\":\"blink\",\"interval_ms\":50", 3100), "\"field\":\"interval_ms\""));
}

static void test_buzzer_patterns() {
  Rig r;
  CHECK(r.cmd("buzzer", "\"pattern\":\"short_beep\"", 1000) == ok(1));
  CHECK(r.out.buzzer);
  r.core.tick(1149); CHECK(r.out.buzzer);
  r.core.tick(1150); CHECK(!r.out.buzzer && has(r.status(1150), "\"buzzer\":\"off\""));
  CHECK(r.cmd("buzzer", "\"pattern\":\"warning\"", 2000) == ok(2));     // 3 x (150 on, 150 off)
  r.core.tick(2100); CHECK(r.out.buzzer);
  r.core.tick(2200); CHECK(!r.out.buzzer);
  r.core.tick(2310); CHECK(r.out.buzzer);
  r.core.tick(2610); CHECK(r.out.buzzer);
  r.core.tick(2760); CHECK(!r.out.buzzer);
  r.core.tick(2900); CHECK(!r.out.buzzer && has(r.status(2900), "\"buzzer\":\"off\""));
  CHECK(r.cmd("buzzer", "\"pattern\":\"emergency\"", 3000) == ok(3));
  r.core.tick(3100); CHECK(r.out.buzzer);
  r.core.tick(3350); CHECK(!r.out.buzzer);
  CHECK(r.cmd("buzzer", "\"pattern\":\"off\"", 3400) == ok(4));
  r.core.tick(3500); CHECK(!r.out.buzzer);
  CHECK(has(r.cmd("buzzer", "\"pattern\":\"siren\"", 3600), "\"field\":\"pattern\""));
}

static void test_follow_rules() {
  Rig r;
  const char* drive = "\"target_id\":7,\"left\":25,\"right\":35,\"duration_ms\":400,\"confidence\":0.8";
  CHECK(has(r.cmd("follow", drive, 0), "FOLLOW_NOT_ENABLED"));
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true", 10), "MISSING_FIELD\",\"field\":\"target_id\""));
  CHECK(has(r.cmd("follow_mode", "\"enabled\":\"yes\",\"target_id\":7", 10), "\"field\":\"enabled\""));
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":70000", 10), "\"field\":\"target_id\""));
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":7", 1000), "\"ok\":true"));
  CHECK(r.core.followMode() && r.core.followTarget() == 7 && r.out.buzzer);   // one beep
  CHECK(has(r.status(1010), "\"state\":\"following\"") && has(r.status(1010), "\"target_id\":7"));
  r.core.tick(1300); CHECK(r.out.led);                    // slow blink while following
  r.core.tick(1600); CHECK(!r.out.led && !r.out.buzzer);
  CHECK(has(r.cmd("follow", drive, 1700), "\"ok\":true,\"left\":25,\"right\":35"));
  CHECK(r.core.state() == STATE_MOVING);
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":10,\"right\":10,\"duration_ms\":300", 1710), "FOLLOW_ACTIVE"));
  CHECK(has(r.cmd("follow", "\"target_id\":8,\"left\":25,\"right\":35,\"duration_ms\":400,\"confidence\":0.8", 1720), "TARGET_MISMATCH"));
  CHECK(r.core.state() == STATE_MOVING);
  CHECK(has(r.cmd("follow", "\"target_id\":7,\"left\":25,\"right\":35,\"duration_ms\":400,\"confidence\":0.3", 1730), "LOW_CONFIDENCE"));
  CHECK(r.core.state() == STATE_IDLE && r.out.left == 0);   // low confidence stops the motors
  CHECK(has(r.cmd("follow", "\"target_id\":7,\"left\":25,\"right\":35,\"duration_ms\":400,\"confidence\":1.5", 1740), "\"field\":\"confidence\""));
  CHECK(has(r.cmd("follow", "\"target_id\":7,\"left\":25,\"right\":35,\"duration_ms\":400,\"confidence\":\"high\"", 1740), "\"field\":\"confidence\""));
  CHECK(has(r.cmd("follow", "\"target_id\":7,\"left\":90,\"right\":-90,\"duration_ms\":400,\"confidence\":1", 1750), "\"left\":60,\"right\":-60"));
  CHECK(has(r.cmd("follow_mode", "\"enabled\":false", 1800), "\"ok\":true"));
  CHECK(!r.core.followMode() && r.out.left == 0 && r.core.state() == STATE_IDLE);
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":7", 1900), "\"ok\":true"));
  CHECK(r.okLast(r.cmd("stop", "", 1950)));            // stop also ends following
  CHECK(!r.core.followMode());
}

static void test_camera_obstacle() {
  Rig r;
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":30,\"duration_ms\":2000", 1000), "\"ok\":true"));
  CHECK(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":40", 1100) == ok(2));
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE && r.core.obstacleActive() && r.out.buzzer);
  CHECK(has(r.status(1110), "\"obstacle_detected\":true,\"obstacle_source\":\"camera\""));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":10,\"duration_ms\":300", 1120), "OBSTACLE_AHEAD"));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":-20,\"right\":20,\"duration_ms\":300", 1130), "\"ok\":true"));   // spin away
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":-20,\"right\":-20,\"duration_ms\":300", 1140), "\"ok\":true"));  // back off
  r.core.tick(1250); CHECK(r.out.led);                     // fast blink
  r.core.tick(1350); CHECK(!r.out.led);
  CHECK(has(r.cmd("obstacle", "\"source\":\"lidar\",\"distance_cm\":40", 1400), "\"field\":\"source\""));
  CHECK(has(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":1000", 1400), "\"field\":\"distance_cm\""));
  r.core.tick(1500);
  r.msgs();
  CHECK(r.okLast(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":0", 2000)));   // refresh
  r.core.tick(3499);
  CHECK(r.core.obstacleActive());
  r.core.tick(3500);
  auto m = r.msgs();
  bool clr = false;
  for (auto& s : m) clr |= has(s, "obstacle_clear");
  CHECK(!r.core.obstacleActive() && clr);
  CHECK(r.okLast(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":0", 3600)));
  CHECK(r.okLast(r.cmd("obstacle_clear", "", 3610)));
  m = r.msgs();
  CHECK(!r.core.obstacleActive() && m.size() == 1 && has(m[0], "obstacle_clear"));
  // following during an obstacle: no motion at all (stop-on-obstacle)
  CHECK(has(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":2", 3700), "\"ok\":true"));
  CHECK(has(r.cmd("follow", "\"target_id\":2,\"left\":20,\"right\":20,\"duration_ms\":400,\"confidence\":1", 3710), "\"ok\":true"));
  CHECK(r.okLast(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":0", 3720)));
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE);
  CHECK(has(r.cmd("follow", "\"target_id\":2,\"left\":-20,\"right\":20,\"duration_ms\":400,\"confidence\":1", 3730), "OBSTACLE_AHEAD"));
  CHECK(has(r.cmd("follow", "\"target_id\":2,\"left\":0,\"right\":0,\"duration_ms\":400,\"confidence\":1", 3740), "\"ok\":true"));
}

static void test_sensor_obstacle() {
  Rig off;
  off.core.onDistance(5, 10);
  CHECK(!off.core.obstacleActive() && has(off.status(10), "\"distance_cm\":null"));
  Rig r(true, true);
  CHECK(has(r.buf, "\"sensor_installed\":true"));
  r.core.onDistance(100, 10);
  CHECK(!r.core.obstacleActive() && r.msgs().empty() && has(r.status(10), "\"distance_cm\":100"));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":30,\"duration_ms\":2000", 20), "\"ok\":true"));
  r.core.onDistance(20, 30);
  auto m = r.msgs();
  CHECK(m.size() == 1 && m[0] == "{\"version\":3,\"type\":\"event\",\"event\":\"obstacle\",\"source\":\"sensor\",\"distance_cm\":20}");
  CHECK(r.out.left == 0 && r.core.state() == STATE_IDLE);
  r.core.onDistance(30, 40);                                // hysteresis: still blocked
  CHECK(r.core.obstacleActive() && r.msgs().empty());
  CHECK(has(r.status(40), "\"obstacle_source\":\"sensor\""));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":30,\"duration_ms\":300", 50), "OBSTACLE_AHEAD"));
  r.core.onDistance(40, 70);
  m = r.msgs();
  CHECK(!r.core.obstacleActive() && m.size() == 1 && has(m[0], "obstacle_clear"));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":-20,\"right\":-20,\"duration_ms\":300", 80), "\"ok\":true"));
  r.core.onDistance(10, 90);                                // reversing is not stopped
  CHECK(r.core.state() == STATE_MOVING);
  r.core.onDistance(-1, 100);                               // no echo = nothing in range
  CHECK(!r.core.obstacleActive());
}

static void test_queue_keeps_newest() {
  Rig r(true, true);
  for (int i = 0; i < 6; ++i) {
    r.core.onDistance(10, 10 + i * 2);
    r.core.onDistance(100, 11 + i * 2);
  }
  auto m = r.msgs();
  CHECK(m.size() == 6 && has(m.back(), "obstacle_clear"));
}

static void test_line_reader() {
  LineReader lr;
  const char* msg = "{\"a\":1}\r\n{}\n";
  std::vector<std::string> lines;
  for (const char* p = msg; *p; ++p)
    if (lr.feed(*p) == 1) lines.push_back(lr.line());
  CHECK(lines.size() == 2 && lines[0] == "{\"a\":1}" && lines[1] == "{}");
  int last = 0;
  for (int i = 0; i < 300; ++i) last = lr.feed('x');
  CHECK(last == 0 && lr.feed('\n') == -1);
  for (const char* p = "{}\n"; *p; ++p) last = lr.feed(*p);
  CHECK(last == 1 && std::string(lr.line()) == "{}");
}

static void test_millis_wraparound() {
  Rig r;
  uint32_t t0 = 0xFFFFFF00u;
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":30,\"right\":30,\"duration_ms\":1000", t0), "\"ok\":true"));
  r.core.tick(t0 + 200);
  CHECK(r.core.state() == STATE_MOVING);
  CHECK(r.cmd("ping", "", t0 + 300) == ok(2));
  r.core.tick(t0 + 900);
  CHECK(r.core.state() == STATE_IDLE && r.core.watchdogTrips() == 1);
}

// status "led" reports the pattern actually shown, with the indicator priority
static void test_status_reports_effective_led() {
  Rig r;
  auto led = [&](uint32_t t) { std::string s = r.status(t); size_t i = s.find("\"led\":\"") + 7;
                               return s.substr(i, s.find('"', i) - i); };
  CHECK(led(0) == "off");
  CHECK(r.okLast(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":3", 10)));
  CHECK(led(20) == "blink_following");
  CHECK(r.okLast(r.cmd("led", "\"state\":\"on\"", 30)));
  CHECK(led(40) == "on");                                   // led command beats following
  CHECK(r.okLast(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":20", 50)));
  CHECK(led(60) == "blink_obstacle");                       // obstacle beats the led command
  CHECK(r.okLast(r.cmd("led", "\"state\":\"off\"", 70)));
  CHECK(r.okLast(r.cmd("emergency_stop", "\"reason\":\"x\"", 80)));
  CHECK(led(90) == "on");                                   // emergency beats everything
  CHECK(r.out.led);
  // buzzer field = the pattern, also during its silent 200 ms phases
  for (uint32_t t = 100; t < 1100; t += 50) CHECK(has(r.status(t), "\"buzzer\":\"emergency\""));
  CHECK(r.okLast(r.cmd("mute", "", 1200)));
  CHECK(has(r.status(1250), "\"buzzer\":\"off\""));
}

// named commands: movement is SIMULATED (motor outputs never move, even with motorsEnabled)
static void test_named_commands_are_simulated() {
  Rig r(true);                            // motors ENABLED on purpose: named movement must still not drive
  auto cmd = [&](const char* name, uint32_t t, const char* extra = "") {
    std::string e = std::string("\"command\":\"") + name + "\"" + extra;
    return r.cmd("command", e, t);
  };
  std::string a = cmd("LEFT", 10);
  CHECK(has(a, "\"ok\":true") && has(a, "\"command\":\"LEFT\"") &&
        has(a, "\"result\":\"SIMULATED_MOVEMENT_RECEIVED: LEFT\""));
  CHECK(r.core.simulatedMovement() == SIM_LEFT && r.out.left == 0 && r.out.right == 0);
  CHECK(has(r.status(20), "\"simulated_movement\":\"left\""));
  for (auto& line : r.out.log) CHECK(line[0] != 'M' || line == "M0,0");   // no motor output ever moved
  // ends by itself after the default 2000 ms (pings keep the watchdog quiet)
  for (uint32_t t = 200; t < 2000; t += 200) { CHECK(r.okLast(r.cmd("ping", "", t))); r.core.tick(t); }
  r.core.tick(2015);
  CHECK(r.core.simulatedMovement() == SIM_NONE);
  bool done = false;
  for (auto& m : r.events()) done |= has(m, "\"move_done\",\"simulated\":true");
  CHECK(done);
  // watchdog: silence while a simulated movement is active
  CHECK(has(cmd("FORWARD", 3000, ",\"duration_ms\":2000"), "SIMULATED_MOVEMENT_RECEIVED: FORWARD"));
  r.core.tick(3400);
  CHECK(r.core.simulatedMovement() == SIM_FORWARD);
  r.core.tick(3600);
  CHECK(r.core.simulatedMovement() == SIM_NONE && r.core.commFault());
  bool wd = false;
  for (auto& m : r.events()) wd |= has(m, "\"watchdog_stop\",\"simulated\":true");
  CHECK(wd);
  CHECK(has(cmd("STOP", 3700), "\"result\":\"STOPPED\"") && !r.core.commFault());
  // disconnect clears it and reports a comm fault; nothing resumes
  CHECK(has(cmd("RIGHT", 3800), "SIMULATED"));
  r.core.onClientDisconnected(3810);
  CHECK(r.core.simulatedMovement() == SIM_NONE && r.core.commFault());
  r.core.onClientConnected(3820, r.buf, sizeof r.buf);
  CHECK(has(r.buf, "\"comm_fault\":true") && has(r.buf, "\"led_installed\":true"));
  r.seq = 0;
  CHECK(has(r.status(3830), "\"simulated_movement\":\"none\""));
  // validation
  CHECK(has(cmd("JUMP", 3900), "UNKNOWN_COMMAND"));
  CHECK(has(cmd("left", 3910), "UNKNOWN_COMMAND"));                       // names are case-sensitive
  CHECK(has(cmd("STOP", 3920, ",\"duration_ms\":300"), "\"field\":\"duration_ms\""));
  CHECK(has(cmd("LEFT", 3930, ",\"duration_ms\":5000"), "\"field\":\"duration_ms\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"command\",\"command_id\":\"z\",\"seq\":99,\"command\":7}", 3940),
            "\"field\":\"command\""));
  CHECK(has(r.raw("{\"version\":3,\"type\":\"command\",\"command_id\":\"z\",\"seq\":100}", 3950),
            "MISSING_FIELD"));
  r.seq = 100;                            // the raw lines above used sequence numbers 99 and 100
  // LED / buzzer
  CHECK(has(cmd("LED_ON", 4000), "\"result\":\"LED_ON\"") && has(r.status(4001), "\"led\":\"on\""));
  CHECK(has(cmd("LED_BLINK", 4010), "LED_BLINK") && has(r.status(4011), "\"led\":\"blink\""));
  CHECK(has(cmd("LED_OFF", 4020), "LED_OFF") && has(r.status(4021), "\"led\":\"off\""));
  CHECK(has(cmd("BUZZER_ON", 5000), "BUZZER_ON") && r.out.buzzer);
  r.core.tick(30000);
  CHECK(r.out.buzzer && has(r.status(30000), "\"buzzer\":\"on\""));
  r.core.tick(65001);                                                      // bounded to 60 s
  CHECK(!r.out.buzzer && has(r.status(65001), "\"buzzer\":\"off\""));
  bool to = false;
  for (auto& m : r.events()) to |= has(m, "buzzer_timeout");
  CHECK(to);
  CHECK(has(cmd("BUZZER_SHORT_BEEP", 66000), "BUZZER_SHORT_BEEP") && r.out.buzzer);
  r.core.tick(66200);
  CHECK(!r.out.buzzer);
  CHECK(has(cmd("BUZZER_WARNING", 66300), "BUZZER_WARNING"));
  CHECK(has(cmd("STATUS", 66400), "STATUS_SENT"));
  bool st = false;
  for (auto& m : r.msgs()) st |= has(m, "\"type\":\"status\"");
  CHECK(st);
  // emergency: latched; movement refused; BUZZER_OFF silences; reset needs clear_emergency
  CHECK(has(cmd("EMERGENCY_STOP", 67000), "EMERGENCY_LATCHED") && r.core.state() == STATE_EMERGENCY);
  CHECK(has(r.status(67001), "\"emergency_reason\":\"operator\""));
  CHECK(has(cmd("BACKWARD", 67010), "EMERGENCY_ACTIVE"));
  CHECK(has(cmd("BUZZER_OFF", 67020), "BUZZER_OFF") && has(r.status(67021), "\"buzzer\":\"off\""));
  CHECK(r.core.state() == STATE_EMERGENCY);
  CHECK(r.okLast(r.cmd("clear_emergency", "\"operator_confirmed\":true", 67030)));
  // following: named movement refused; obstacle: FORWARD refused, BACKWARD allowed
  CHECK(r.okLast(r.cmd("follow_mode", "\"enabled\":true,\"target_id\":1", 67040)));
  CHECK(has(cmd("LEFT", 67050), "FOLLOW_ACTIVE"));
  CHECK(has(cmd("STOP", 67060), "STOPPED") && !r.core.followMode());
  CHECK(r.okLast(r.cmd("obstacle", "\"source\":\"camera\",\"distance_cm\":20", 67070)));
  CHECK(has(cmd("FORWARD", 67080), "OBSTACLE_AHEAD"));
  CHECK(has(cmd("BACKWARD", 67090), "SIMULATED_MOVEMENT_RECEIVED: BACKWARD"));
  for (auto& line : r.out.log) CHECK(line[0] != 'M' || line == "M0,0");
}

// the longest possible status message still fits the 512-byte buffer
static void test_status_worst_case_fits() {
  RobotConfig c;
  c.sensorEnabled = true;
  FakeOutputs o;
  RobotCore core(o, c);
  char buf[OUT_LEN];
  core.begin(0);
  core.onClientConnected(0, buf, sizeof buf);
  core.handleLine("{\"version\":3,\"type\":\"follow_mode\",\"command_id\":\"aaaaaaaaaaaaaaaaaaaaaaaa\","
                  "\"seq\":999999998,\"enabled\":true,\"target_id\":65535}", 10, buf, sizeof buf);
  core.onDistance(999, 20);
  core.handleLine("{\"version\":3,\"type\":\"emergency_stop\",\"command_id\":\"bbbbbbbbbbbbbbbbbbbbbbbb\","
                  "\"seq\":999999999,\"reason\":\"cccccccccccccccccccccccc\"}", 30, buf, sizeof buf);
  core.buildStatus(4000000000u, buf, sizeof buf);
  std::string s = buf;
  CHECK(s.size() < OUT_LEN - 1 && s.back() == '}');
}

// AI events from the laptop: MODEL_STARTED / FALL_DETECTED / FALL_CLEARED / MODEL_STOPPED
static void test_ai_events_led_buzzer_and_logs() {
  Rig r(true);                            // motors enabled on purpose: nothing here may drive them
  auto ev = [&](const char* name, uint32_t t) {
    return r.cmd("command", std::string("\"command\":\"") + name + "\"", t);
  };
  auto log = [&]() { return std::string(r.core.lastLog()); };
  // safe start: LED off before the model runs
  r.core.tick(5);
  CHECK(!r.out.led && !r.out.buzzer && !r.core.modelRunning());
  // 1. MODEL_STARTED -> LED on; 2. stays continuously on
  CHECK(has(ev("MODEL_STARTED", 10), "\"result\":\"MODEL_RUNNING\""));
  CHECK(has(log(), "[RX] Event received: MODEL_STARTED") && has(log(), "[LED] Model running: LED continuously ON"));
  bool always = true;
  for (uint32_t t = 20; t < 3000; t += 37) {
    if (t % 370 == 20) CHECK(r.okLast(r.cmd("ping", "", t)));
    r.core.tick(t);
    always &= r.out.led && !r.out.buzzer;
  }
  CHECK(always);
  CHECK(has(r.status(3000), "\"led\":\"on\"") && has(r.status(3000), "\"model_running\":true"));
  CHECK(has(ev("MODEL_STARTED", 3010), "MODEL_RUNNING") && has(log(), "(already)"));
  // a LED test during the model run, then LED_OFF returns to the model light (not off)
  CHECK(has(ev("LED_BLINK", 3020), "LED_BLINK"));
  CHECK(has(ev("LED_OFF", 3030), "LED_OFF") && has(log(), "automatic state (ON - model running)"));
  r.core.tick(3040);
  CHECK(r.out.led);
  // 3. FALL_DETECTED -> blinking LED + buzzer pattern, non-blocking
  CHECK(has(ev("FALL_DETECTED", 4000), "\"result\":\"FALL_ALERT_ACTIVE\""));
  CHECK(has(log(), "[RX] Event received: FALL_DETECTED") && has(log(), "[ALERT] Fall detected") &&
        has(log(), "[LED] Fall alert: blinking") && has(log(), "[BUZZER] Fall warning active"));
  int ledChanges = 0, buzOn = 0;
  bool prevLed = r.out.led;
  for (uint32_t t = 4000; t < 6400; t += 50) {
    r.core.tick(t);
    if (r.out.led != prevLed) { ++ledChanges; prevLed = r.out.led; }
    if (r.out.buzzer) ++buzOn;
    if (t % 400 == 0) CHECK(r.okLast(r.cmd("ping", "", t)));   // the link keeps working meanwhile
  }
  CHECK(ledChanges >= 8 && buzOn > 0 && buzOn < 48);            // blinking, and the buzzer pulses (not constant)
  CHECK(has(r.status(6400), "\"led\":\"blink_fall\"") && has(r.status(6400), "\"buzzer\":\"fall_alert\""));
  // repeated FALL_DETECTED does not restart / stack the alert
  CHECK(has(ev("FALL_DETECTED", 6500), "FALL_ALERT_ACTIVE") && has(log(), "already active - not restarted"));
  // movement refused during the fall alert (named and typed), no motor output
  CHECK(has(ev("LEFT", 6510), "FALL_ALERT_ACTIVE") && has(log(), "[REJECTED] LEFT: FALL_ALERT_ACTIVE"));
  CHECK(has(r.cmd("motor", "\"mode\":\"manual\",\"left\":20,\"right\":20,\"duration_ms\":300", 6520),
            "FALL_ALERT_ACTIVE"));
  // 4. FALL_CLEARED -> buzzer stops, LED back to continuously on
  CHECK(has(ev("FALL_CLEARED", 7000), "FALL_ALERT_CLEARED"));
  CHECK(has(log(), "[ALERT] Fall alert cleared") && has(log(), "[LED] Model running: LED continuously ON") &&
        has(log(), "[BUZZER] Alert stopped"));
  bool on = true;
  for (uint32_t t = 7000; t < 8000; t += 25) { r.core.tick(t); on &= r.out.led && !r.out.buzzer; }
  CHECK(on);
  CHECK(has(ev("FALL_CLEARED", 8010), "NO_FALL_ALERT"));
  // fall alert buzzer is bounded (60 s) but the LED keeps blinking until FALL_CLEARED; BUZZER_OFF mutes
  CHECK(has(ev("FALL_DETECTED", 9000), "FALL_ALERT_ACTIVE"));
  for (uint32_t t = 9500; t < 70000; t += 400) { r.cmd("ping", "", t); r.core.tick(t); }
  r.core.tick(70100);
  CHECK(!r.out.buzzer && has(r.status(70100), "\"buzzer\":\"off\"") && has(r.status(70100), "blink_fall"));
  bool to = false;
  for (auto& m : r.events()) to |= has(m, "\"alert\":\"fall\"");
  CHECK(to);
  CHECK(has(ev("FALL_CLEARED", 70200), "FALL_ALERT_CLEARED"));
  CHECK(has(ev("FALL_DETECTED", 70300), "FALL_ALERT_ACTIVE"));
  CHECK(has(ev("BUZZER_OFF", 70310), "BUZZER_OFF") && has(log(), "alarm muted"));
  bool silent = true;
  for (uint32_t t = 70320; t < 72000; t += 40) { r.core.tick(t); silent &= !r.out.buzzer; }
  CHECK(silent && has(r.status(72000), "blink_fall"));
  // emergency has priority over the fall alert; clearing the emergency keeps the fall alert
  CHECK(has(ev("EMERGENCY_STOP", 72100), "EMERGENCY_LATCHED"));
  bool solid = true;
  for (uint32_t t = 72100; t < 73000; t += 30) { r.core.tick(t); solid &= r.out.led; }
  CHECK(solid && has(r.status(73000), "\"led\":\"on\"") && has(r.status(73000), "\"buzzer\":\"emergency\""));
  CHECK(r.okLast(r.cmd("clear_emergency", "\"operator_confirmed\":true", 73100)));
  CHECK(has(log(), "Fall alert is still active") && r.core.fallAlert());
  CHECK(has(ev("FALL_CLEARED", 73200), "FALL_ALERT_CLEARED"));
  // 5. MODEL_STOPPED -> LED off, buzzer off (also ends a fall alert)
  CHECK(has(ev("FALL_DETECTED", 74000), "FALL_ALERT_ACTIVE"));
  CHECK(has(ev("MODEL_STOPPED", 74100), "MODEL_STOPPED") && has(log(), "[LED] Model stopped: LED OFF"));
  r.core.tick(74200);
  CHECK(!r.out.led && !r.out.buzzer && !r.core.fallAlert());
  // 6. movement acknowledged, never drives a motor
  CHECK(has(ev("FORWARD", 75000), "SIMULATED_MOVEMENT_RECEIVED: FORWARD"));
  CHECK(has(log(), "[RX] Command received: FORWARD") &&
        has(log(), "[MOVEMENT] FORWARD received - SIMULATION ONLY, motors disabled"));
  for (auto& line : r.out.log) CHECK(line[0] != 'M' || line == "M0,0");
  // 7. invalid messages: logged and rejected
  CHECK(has(r.raw("not json", 75100), "BAD_JSON") && has(log(), "[RX] Malformed JSON"));
  CHECK(has(ev("JUMP", 75200), "UNKNOWN_COMMAND") && has(log(), "[REJECTED] JUMP: UNKNOWN_COMMAND"));
  r.core.handleOverflow(r.buf, sizeof r.buf);
  CHECK(has(r.core.lastLog(), "TOO_LONG"));
  // 8. MODEL_STARTED, then a lost connection: model light off, nothing resumes
  CHECK(has(ev("MODEL_STARTED", 76000), "MODEL_RUNNING") && has(ev("RIGHT", 76010), "SIMULATED"));
  r.core.onClientDisconnected(76020);
  r.core.tick(76030);
  CHECK(!r.out.led && r.core.simulatedMovement() == SIM_NONE && !r.core.modelRunning());
  // heartbeats are not printed
  r.core.onClientConnected(76100, r.buf, sizeof r.buf);
  r.seq = 0;
  CHECK(r.okLast(r.cmd("ping", "", 76110)) && std::string(r.core.lastLog()).empty());
}

int main() {
  test_json_parser();
  test_ai_events_led_buzzer_and_logs();
  test_named_commands_are_simulated();
  test_status_worst_case_fits();
  test_status_reports_effective_led();
  test_boot_hello_and_status();
  test_common_field_validation();
  test_motor_commands();
  test_motor_outputs_disabled_in_phase2();
  test_watchdog_and_comm_fault();
  test_disconnect_reports_comm_fault_and_never_resumes();
  test_move_duration_expires();
  test_emergency_latch_and_clear();
  test_emergency_buzzer_is_bounded();
  test_led_patterns();
  test_buzzer_patterns();
  test_follow_rules();
  test_camera_obstacle();
  test_sensor_obstacle();
  test_queue_keeps_newest();
  test_line_reader();
  test_millis_wraparound();
  std::printf("%d checks passed, %d failed\n", g_passed, g_failed);
  return g_failed == 0 ? 0 : 1;
}
