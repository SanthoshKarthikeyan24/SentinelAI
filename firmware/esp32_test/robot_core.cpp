// SentinelAI robot controller core (protocol v3, JSON lines) - see robot_core.h and docs/PROTOCOL.md.
// robot/sim_robot.py mirrors this file; tests/test_core_parity.py checks that both behave identically.
#include "robot_core.h"

#include <stdio.h>
#include <string.h>

namespace sentinel {

// ------------------------------------------------------------------ names
const char* ledName(LedMode m) {
  switch (m) {
    case LED_ON: return "on";
    case LED_BLINK: return "blink";
    default: return "off";
  }
}

const char* buzzerName(BuzzerMode m) {
  switch (m) {
    case BUZ_SHORT: return "short_beep";
    case BUZ_WARNING: return "warning";
    case BUZ_EMERGENCY: return "emergency";
    case BUZ_ON: return "on";
    default: return "off";
  }
}

const char* simMoveName(SimMove m) {
  switch (m) {
    case SIM_FORWARD: return "forward";
    case SIM_BACKWARD: return "backward";
    case SIM_LEFT: return "left";
    case SIM_RIGHT: return "right";
    default: return "none";
  }
}

bool validId(const char* s) {
  size_t n = 0;
  for (; s[n]; ++n) {
    char c = s[n];
    bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' ||
              c == '-' || c == '.';
    if (!ok || n >= 24) return false;
  }
  return n >= 1;
}

// --------------------------------------------------------------- JSON subset
const JField* JObject::get(const char* key) const {
  for (int k = 0; k < n; ++k)
    if (strcmp(f[k].key, key) == 0) return &f[k];
  return nullptr;
}

namespace {

void skipWs(const char*& p) {
  while (*p == ' ' || *p == '\t') ++p;
}

bool isDigit(char c) { return c >= '0' && c <= '9'; }

bool printable(char c) {
  unsigned char u = static_cast<unsigned char>(c);
  return u >= 0x20 && u <= 0x7e && c != '\\';
}

}  // namespace

bool parseJsonObject(const char* text, JObject& o) {
  o.n = 0;
  const char* p = text;
  skipWs(p);
  if (*p != '{') return false;
  ++p;
  skipWs(p);
  if (*p == '}') {
    ++p;
    skipWs(p);
    return *p == '\0';
  }
  for (;;) {
    if (o.n >= MAX_FIELDS) return false;
    JField& f = o.f[o.n];
    if (*p != '"') return false;                       // key
    ++p;
    size_t k = 0;
    while (*p && *p != '"') {
      if (!printable(*p) || k >= MAX_KEY) return false;
      f.key[k++] = *p++;
    }
    if (*p != '"' || k == 0) return false;
    f.key[k] = '\0';
    ++p;
    for (int j = 0; j < o.n; ++j)
      if (strcmp(o.f[j].key, f.key) == 0) return false;   // duplicate key
    skipWs(p);
    if (*p != ':') return false;
    ++p;
    skipWs(p);
    f.s[0] = '\0';
    f.i = 0;
    f.d = 0.0;
    f.b = false;
    if (*p == '"') {                                    // string
      ++p;
      size_t m = 0;
      while (*p && *p != '"') {
        if (!printable(*p) || m >= MAX_STR) return false;
        f.s[m++] = *p++;
      }
      if (*p != '"') return false;
      f.s[m] = '\0';
      ++p;
      f.type = J_STR;
    } else if (strncmp(p, "true", 4) == 0) {
      f.type = J_BOOL;
      f.b = true;
      p += 4;
    } else if (strncmp(p, "false", 5) == 0) {
      f.type = J_BOOL;
      p += 5;
    } else if (strncmp(p, "null", 4) == 0) {
      f.type = J_NULL;
      p += 4;
    } else if (*p == '-' || isDigit(*p)) {             // number
      bool neg = false;
      if (*p == '-') { neg = true; ++p; }
      if (!isDigit(*p)) return false;
      if (*p == '0' && isDigit(p[1])) return false;   // no leading zeros
      long ip = 0;
      int nd = 0;
      while (isDigit(*p)) {
        if (nd >= 9) return false;
        ip = ip * 10 + (*p - '0');
        ++p;
        ++nd;
      }
      if (*p == '.') {
        ++p;
        if (!isDigit(*p)) return false;
        long fr = 0;
        int fd = 0;
        double pw = 1.0;
        while (isDigit(*p)) {
          if (fd >= 6) return false;
          fr = fr * 10 + (*p - '0');
          pw *= 10.0;
          ++p;
          ++fd;
        }
        double v = static_cast<double>(ip) + static_cast<double>(fr) / pw;
        f.type = J_NUM;
        f.d = neg ? -v : v;
      } else {
        f.type = J_INT;
        f.i = neg ? -ip : ip;
        f.d = static_cast<double>(f.i);
      }
    } else {
      return false;                                     // nested objects, arrays, other
    }
    ++o.n;
    skipWs(p);
    if (*p == ',') {
      ++p;
      skipWs(p);
      continue;
    }
    if (*p == '}') {
      ++p;
      skipWs(p);
      return *p == '\0';
    }
    return false;
  }
}

// ------------------------------------------------------------------ LineReader
int LineReader::feed(char c) {
  if (c == '\n') {
    bool wasOverflow = overflow_;
    buf_[len_] = '\0';
    if (wasOverflow) {
      reset();
      return -1;
    }
    if (len_ > 0 && buf_[len_ - 1] == '\r') buf_[--len_] = '\0';
    len_ = 0;
    overflow_ = false;
    return 1;
  }
  if (overflow_) return 0;
  if (len_ >= MAX_LINE) {
    overflow_ = true;
    return 0;
  }
  buf_[len_++] = c;
  buf_[len_] = '\0';
  return 0;
}

// --------------------------------------------------------- message schema
namespace {

struct TypeSpec {
  const char* name;
  const char* required[6];
  const char* optional[2];
};

// Order of the lists = order in which fields are checked (shared with robot/sim_robot.py).
const TypeSpec kTypes[] = {
    {"ping", {nullptr}, {nullptr}},
    {"status_request", {nullptr}, {nullptr}},
    {"motor", {"mode", "left", "right", "duration_ms", nullptr}, {nullptr}},
    {"stop", {nullptr}, {nullptr}},
    {"follow_mode", {"enabled", nullptr}, {"target_id", nullptr}},
    {"follow", {"target_id", "left", "right", "duration_ms", "confidence", nullptr}, {nullptr}},
    {"led", {"state", nullptr}, {"interval_ms", nullptr}},
    {"buzzer", {"pattern", nullptr}, {nullptr}},
    {"mute", {nullptr}, {nullptr}},
    {"emergency_stop", {"reason", nullptr}, {nullptr}},
    {"clear_emergency", {"operator_confirmed", nullptr}, {nullptr}},
    {"obstacle", {"source", "distance_cm", nullptr}, {nullptr}},
    {"obstacle_clear", {nullptr}, {nullptr}},
    {"command", {"command", nullptr}, {"duration_ms", nullptr}},
};

// Named commands accepted by the "command" message type (see handleNamed()).
const char* const kNamed[] = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "LED_ON", "LED_OFF", "LED_BLINK",
                              "BUZZER_ON", "BUZZER_OFF", "BUZZER_SHORT_BEEP", "BUZZER_WARNING", "STATUS",
                              "EMERGENCY_STOP", "MODEL_STARTED", "MODEL_STOPPED", "FALL_DETECTED", "FALL_CLEARED",
                              nullptr};

bool isEventName(const char* n) {   // AI state events (printed as "Event received")
  return strncmp(n, "MODEL_", 6) == 0 || strncmp(n, "FALL_", 5) == 0;
}
const int kNumTypes = sizeof(kTypes) / sizeof(kTypes[0]);

bool isCommon(const char* k) {
  return strcmp(k, "version") == 0 || strcmp(k, "type") == 0 || strcmp(k, "command_id") == 0 ||
         strcmp(k, "seq") == 0;
}

bool inList(const char* const* list, const char* k) {
  for (int i = 0; list[i]; ++i)
    if (strcmp(list[i], k) == 0) return true;
  return false;
}

bool intIn(const JField* f, long lo, long hi) { return f && f->type == J_INT && f->i >= lo && f->i <= hi; }

bool strOneOf(const JField* f, const char* const* opts) {
  if (!f || f->type != J_STR) return false;
  return inList(opts, f->s);
}

int clampAbs(int v, int lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

void appendf(char* out, size_t len, size_t& pos, const char* fmt, const char* s) {
  if (pos >= len) return;
  int n = snprintf(out + pos, len - pos, fmt, s);
  if (n > 0) pos += static_cast<size_t>(n);
}

}  // namespace

// ------------------------------------------------------------------- RobotCore
RobotCore::RobotCore(RobotOutputs& out, const RobotConfig& cfg) : out_(out), cfg_(cfg) {
  if (cfg_.maxSpeedPct < 0) cfg_.maxSpeedPct = 0;
  if (cfg_.maxSpeedPct > 100) cfg_.maxSpeedPct = 100;
  if (cfg_.obstacleClearCm < cfg_.obstacleStopCm) cfg_.obstacleClearCm = cfg_.obstacleStopCm;
  memset(queue_, 0, sizeof queue_);
}

const char* RobotCore::stateName() const {
  if (state_ == STATE_EMERGENCY) return "emergency";
  if (state_ == STATE_MOVING) return "moving";
  if (follow_) return "following";
  return "idle";
}

void RobotCore::begin(uint32_t nowMs) {
  bootMs_ = nowMs;
  lastValidMs_ = nowMs;
  lastStatusMs_ = nowMs;
  state_ = STATE_IDLE;
  follow_ = false;
  followTarget_ = -1;
  modelRunning_ = false;       // safe start: LED off until the laptop says the AI model runs
  fallAlert_ = false;
  stopMotors();
  outputsInitialised_ = false;
  updateIndicators(nowMs);
}

void RobotCore::pushMessage(const char* text) {
  if (qCount_ == OUT_QUEUE) {          // full: drop the oldest so the newest always gets through
    qHead_ = (qHead_ + 1) % OUT_QUEUE;
    --qCount_;
  }
  int slot = (qHead_ + qCount_) % OUT_QUEUE;
  snprintf(queue_[slot], OUT_LEN, "%s", text);
  ++qCount_;
}

bool RobotCore::popMessage(char* msg, size_t len) {
  if (qCount_ == 0) return false;
  snprintf(msg, len, "%s", queue_[qHead_]);
  qHead_ = (qHead_ + 1) % OUT_QUEUE;
  --qCount_;
  return true;
}

void RobotCore::stopMotors() {
  left_ = right_ = 0;
  out_.setMotors(0, 0);
  if (state_ == STATE_MOVING) state_ = STATE_IDLE;
}

bool RobotCore::drive(int left, int right, uint32_t nowMs, uint32_t durationMs) {
  if (state_ == STATE_EMERGENCY) {   // defence in depth: never drive in emergency
    stopMotors();
    return false;
  }
  if (left == 0 && right == 0) {
    stopMotors();
    return true;
  }
  left_ = left;
  right_ = right;
  moveStartMs_ = nowMs;
  moveDurationMs_ = durationMs;
  state_ = STATE_MOVING;
  out_.setMotors(cfg_.motorsEnabled ? left : 0, cfg_.motorsEnabled ? right : 0);
  return true;
}

void RobotCore::enterEmergency(const char* reason, uint32_t nowMs) {
  stopMotors();                 // 1. motors to zero first
  simMove_ = SIM_NONE;
  follow_ = false;
  followTarget_ = -1;
  if (state_ != STATE_EMERGENCY) {   // a repeated emergency_stop does not restart the buzzer timer
    state_ = STATE_EMERGENCY;
    snprintf(emergencyReason_, sizeof emergencyReason_, "%s", reason);
    emergencyStartMs_ = nowMs;
    muted_ = false;
    emergencyBuzzReported_ = false;
  }
  updateIndicators(nowMs);      // 2. red LED + 3. emergency buzzer
}

void RobotCore::startBuzzer(BuzzerMode m, uint32_t nowMs) {
  buzzerMode_ = m;
  buzzerStartMs_ = nowMs;
}

// Fall alert buzzer: beep-beep-pause (200 on, 200 off, 200 on, 600 off), bounded like the emergency buzzer.
bool RobotCore::fallBuzzerActive(uint32_t nowMs) const {
  return fallAlert_ && !fallMuted_ && nowMs - fallStartMs_ < cfg_.emergencyBuzzMaxMs;
}

bool RobotCore::buzzerSounding(uint32_t nowMs) const {
  if (state_ == STATE_EMERGENCY) {   // priority 1: emergency alarm
    uint32_t t = nowMs - emergencyStartMs_;
    return !muted_ && t < cfg_.emergencyBuzzMaxMs && (t % 500u) < 300u;
  }
  if (fallAlert_) {                  // priority 2: fall alert (test commands cannot override it)
    if (!fallBuzzerActive(nowMs)) return false;
    uint32_t c = (nowMs - fallStartMs_) % 1200u;
    return c < 200u || (c >= 400u && c < 600u);
  }
  uint32_t t = nowMs - buzzerStartMs_;
  switch (buzzerMode_) {
    case BUZ_SHORT: return t < 150u;
    case BUZ_WARNING: return t < 900u && (t % 300u) < 150u;
    case BUZ_EMERGENCY: return t < cfg_.emergencyBuzzMaxMs && (t % 500u) < 300u;
    case BUZ_ON: return t < cfg_.emergencyBuzzMaxMs;     // continuous, but never longer than 60 s
    default: return false;
  }
}

// LED and buzzer levels from the robot state (priority order) and the time.
void RobotCore::updateIndicators(uint32_t nowMs) {
  bool led;
  if (state_ == STATE_EMERGENCY) {
    led = true;                                                   // solid: emergency
  } else if (fallAlert_) {
    led = ((nowMs - fallStartMs_) / cfg_.fallBlinkMs) % 2u == 0u;  // blinking: fall alert
  } else if (obstacleActive()) {
    led = (nowMs / 100u) % 2u == 0u;                              // fast blink: obstacle
  } else if (ledMode_ == LED_ON) {
    led = true;
  } else if (ledMode_ == LED_BLINK) {
    led = (nowMs / ledIntervalMs_) % 2u == 0u;
  } else if (modelRunning_) {
    led = true;                                                   // solid: AI model running
  } else {
    led = follow_ && (nowMs / 500u) % 2u == 0u;                   // slow blink: following
  }
  // finished one-shot patterns return to OFF; bounded emergency patterns report once
  if (state_ != STATE_EMERGENCY) {
    uint32_t t = nowMs - buzzerStartMs_;
    if ((buzzerMode_ == BUZ_SHORT && t >= 150u) || (buzzerMode_ == BUZ_WARNING && t >= 900u)) {
      buzzerMode_ = BUZ_OFF;
    } else if ((buzzerMode_ == BUZ_EMERGENCY || buzzerMode_ == BUZ_ON) && t >= cfg_.emergencyBuzzMaxMs) {
      buzzerMode_ = BUZ_OFF;
      pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"buzzer_timeout\"}");
    }
  } else if (!muted_ && !emergencyBuzzReported_ && nowMs - emergencyStartMs_ >= cfg_.emergencyBuzzMaxMs) {
    emergencyBuzzReported_ = true;
    pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"buzzer_timeout\"}");
  }
  if (fallAlert_ && !fallMuted_ && !fallBuzzReported_ && nowMs - fallStartMs_ >= cfg_.emergencyBuzzMaxMs) {
    fallBuzzReported_ = true;     // the fall buzzer stops after 60 s; the LED keeps blinking until FALL_CLEARED
    pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"buzzer_timeout\",\"alert\":\"fall\"}");
  }
  bool buzzer = buzzerSounding(nowMs);
  if (!outputsInitialised_ || buzzer != buzzerLevel_ || led != ledLevel_) {
    outputsInitialised_ = true;
    buzzerLevel_ = buzzer;
    ledLevel_ = led;
    out_.setAlarm(buzzer, led);
  }
}

void RobotCore::onClientConnected(uint32_t nowMs, char* hello, size_t len) {
  connected_ = true;
  lastSeq_ = 0;                 // every connection starts its sequence again at 1
  lastValidMs_ = nowMs;
  lastStatusMs_ = nowMs;
  snprintf(hello, len,
           "{\"version\":3,\"type\":\"event\",\"event\":\"hello\",\"protocol\":%d,\"state\":\"%s\","
           "\"motors_enabled\":%s,\"sensor_installed\":%s,\"led_installed\":%s,\"buzzer_installed\":%s,"
           "\"comm_fault\":%s,\"watchdog_trips\":%lu}",
           PROTOCOL_VERSION, stateName(), cfg_.motorsEnabled ? "true" : "false",
           cfg_.sensorEnabled ? "true" : "false", cfg_.ledInstalled ? "true" : "false",
           cfg_.buzzerInstalled ? "true" : "false", commFault_ ? "true" : "false",
           static_cast<unsigned long>(watchdogTrips_));
}

void RobotCore::onClientDisconnected(uint32_t /*nowMs*/) {
  connected_ = false;
  // the link failed while (simulated) moving: report it later; never resume afterwards
  if (state_ == STATE_MOVING || simMove_ != SIM_NONE) commFault_ = true;
  simMove_ = SIM_NONE;
  modelRunning_ = false;        // "model running" LED needs a live laptop; a fall alert stays until cleared
  follow_ = false;              // never keep following without the laptop
  followTarget_ = -1;
  stopMotors();
}

void RobotCore::buildStatus(uint32_t nowMs, char* msg, size_t len) const {
  char target[24], dist[24], reason[MAX_STR + 3], lastId[MAX_STR + 3];
  if (followTarget_ >= 0) snprintf(target, sizeof target, "%ld", followTarget_);
  else snprintf(target, sizeof target, "null");
  if (cfg_.sensorEnabled && distanceCm_ >= 0) snprintf(dist, sizeof dist, "%d", distanceCm_);
  else snprintf(dist, sizeof dist, "null");
  if (state_ == STATE_EMERGENCY) snprintf(reason, sizeof reason, "\"%s\"", emergencyReason_);
  else snprintf(reason, sizeof reason, "null");
  if (lastCommandId_[0]) snprintf(lastId, sizeof lastId, "\"%s\"", lastCommandId_);
  else snprintf(lastId, sizeof lastId, "null");
  const char* src = localObstacle_ && remoteObstacle_ ? "both"
                  : localObstacle_ ? "sensor" : remoteObstacle_ ? "camera" : "none";
  // the pattern being played (not the on/off phase at this instant)
  const char* buz = state_ == STATE_EMERGENCY
                        ? ((!muted_ && nowMs - emergencyStartMs_ < cfg_.emergencyBuzzMaxMs) ? "emergency" : "off")
                    : fallAlert_ ? (fallBuzzerActive(nowMs) ? "fall_alert" : "off")
                        : buzzerName(buzzerMode_);
  // the LED pattern actually shown (same priority as updateIndicators), not just the last led command
  const char* led = state_ == STATE_EMERGENCY ? "on"
                  : fallAlert_ ? "blink_fall"
                  : obstacleActive() ? "blink_obstacle"
                  : ledMode_ != LED_OFF ? ledName(ledMode_)
                  : modelRunning_ ? "on"
                  : follow_ ? "blink_following" : "off";
  snprintf(msg, len,
           "{\"version\":3,\"type\":\"status\",\"state\":\"%s\",\"motors_stopped\":%s,\"left\":%d,\"right\":%d,"
           "\"motors_enabled\":%s,\"follow_mode\":%s,\"target_id\":%s,\"emergency_active\":%s,"
           "\"emergency_reason\":%s,\"obstacle_detected\":%s,\"obstacle_source\":\"%s\","
           "\"sensor_installed\":%s,\"distance_cm\":%s,\"watchdog_ok\":%s,\"watchdog_trips\":%lu,"
           "\"led\":\"%s\",\"buzzer\":\"%s\",\"simulated_movement\":\"%s\",\"model_running\":%s,"
           "\"fall_alert\":%s,\"last_command_id\":%s,\"uptime_s\":%lu}",
           stateName(), (left_ == 0 && right_ == 0) ? "true" : "false", left_, right_,
           cfg_.motorsEnabled ? "true" : "false", follow_ ? "true" : "false", target,
           state_ == STATE_EMERGENCY ? "true" : "false", reason, obstacleActive() ? "true" : "false", src,
           cfg_.sensorEnabled ? "true" : "false", dist, commFault_ ? "false" : "true",
           static_cast<unsigned long>(watchdogTrips_), led, buz, simMoveName(simMove_),
           modelRunning_ ? "true" : "false", fallAlert_ ? "true" : "false", lastId,
           static_cast<unsigned long>((nowMs - bootMs_) / 1000u));
}

void RobotCore::pushStatus(uint32_t nowMs) {
  char msg[OUT_LEN];
  buildStatus(nowMs, msg, sizeof msg);
  pushMessage(msg);
  lastStatusMs_ = nowMs;
}

void RobotCore::logf(const char* fmt, const char* a, const char* b, const char* c) {
  size_t used = strlen(log_);
  if (used + 2 >= sizeof log_) return;
  if (used) log_[used++] = '\n';
  snprintf(log_ + used, sizeof log_ - used, fmt, a, b, c);
}

void RobotCore::ackError(char* out, size_t len, const char* id, long seq, const char* code, const char* field) {
  ++rejected_;
  if (strcmp(code, "BAD_JSON") == 0) logf("[RX] Malformed JSON - rejected (BAD_JSON)");
  else if (strcmp(code, "TOO_LONG") == 0) logf("[RX] Message longer than 200 characters - rejected (TOO_LONG)");
  else {
    if (!log_[0]) logf("[RX] Message received: %s", curLabel_);   // e.g. a rejected heartbeat
    if (field) logf("[REJECTED] %s: %s (field '%s')", curLabel_, code, field);
    else logf("[REJECTED] %s: %s", curLabel_, code);
  }
  size_t pos = 0;
  if (id) appendf(out, len, pos, "{\"version\":3,\"type\":\"ack\",\"command_id\":\"%s\"", id);
  else appendf(out, len, pos, "%s", "{\"version\":3,\"type\":\"ack\",\"command_id\":null");
  char tail[96];
  snprintf(tail, sizeof tail, ",\"seq\":%ld,\"ok\":false,\"error\":\"%s\"", seq, code);
  appendf(out, len, pos, "%s", tail);
  if (field) appendf(out, len, pos, ",\"field\":\"%s\"", field);
  appendf(out, len, pos, "%s", "}");
}

void RobotCore::handleOverflow(char* out, size_t len) {
  log_[0] = '\0';
  ackError(out, len, nullptr, 0, "TOO_LONG", nullptr);
}

void RobotCore::onDistance(int cm, uint32_t nowMs) {
  if (!cfg_.sensorEnabled) return;
  bool wasActive = obstacleActive();
  bool wasLocal = localObstacle_;
  distanceCm_ = cm;
  if (cm >= 0 && cm < cfg_.obstacleStopCm) localObstacle_ = true;
  else if (cm < 0 || cm > cfg_.obstacleClearCm) localObstacle_ = false;
  if (localObstacle_ && !wasLocal) {
    char e[OUT_LEN];
    snprintf(e, sizeof e,
             "{\"version\":3,\"type\":\"event\",\"event\":\"obstacle\",\"source\":\"sensor\",\"distance_cm\":%d}", cm);
    pushMessage(e);
    if (!wasActive && state_ != STATE_EMERGENCY) startBuzzer(BUZ_SHORT, nowMs);
  }
  if (localObstacle_ && state_ == STATE_MOVING && (follow_ || left_ + right_ > 0)) stopMotors();
  if (localObstacle_ && simMove_ == SIM_FORWARD) simMove_ = SIM_NONE;
  if (wasActive && !obstacleActive()) pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"obstacle_clear\"}");
  updateIndicators(nowMs);
}

void RobotCore::handleLine(const char* line, uint32_t nowMs, char* out, size_t len) {
  log_[0] = '\0';
  curLabel_ = "message";
  if (strlen(line) > MAX_LINE) {
    handleOverflow(out, len);
    return;
  }
  static JObject o;   // static: keeps ~1 KB off the ESP32 task stack (single-threaded use)
  if (!parseJsonObject(line, o)) {
    ackError(out, len, nullptr, 0, "BAD_JSON", nullptr);
    return;
  }
  const JField* fid = o.get("command_id");
  const JField* fseq = o.get("seq");
  const JField* fver = o.get("version");
  const JField* ftype = o.get("type");
  {   // Serial Monitor header: "[RX] Command received: LEFT" (heartbeats and follow stream stay quiet)
    const JField* fc = o.get("command");
    bool isCmd = ftype && ftype->type == J_STR && strcmp(ftype->s, "command") == 0;
    if (isCmd && fc && fc->type == J_STR) curLabel_ = fc->s;
    else if (ftype && ftype->type == J_STR) curLabel_ = ftype->s;
    bool quiet = !isCmd && (strcmp(curLabel_, "ping") == 0 || strcmp(curLabel_, "follow") == 0);
    if (!quiet) logf(isCmd && isEventName(curLabel_) ? "[RX] Event received: %s" : "[RX] Command received: %s", curLabel_);
  }
  const char* id = (fid && fid->type == J_STR && validId(fid->s)) ? fid->s : nullptr;
  long eseq = (fseq && fseq->type == J_INT && fseq->i >= 0 && fseq->i <= 999999999L) ? fseq->i : 0;

  if (!fver) { ackError(out, len, id, eseq, "MISSING_FIELD", "version"); return; }
  if (fver->type != J_INT || fver->i != PROTOCOL_VERSION) { ackError(out, len, id, eseq, "BAD_VERSION", nullptr); return; }
  if (!fid) { ackError(out, len, id, eseq, "MISSING_FIELD", "command_id"); return; }
  if (!id) { ackError(out, len, id, eseq, "BAD_FIELD", "command_id"); return; }
  if (!fseq) { ackError(out, len, id, eseq, "MISSING_FIELD", "seq"); return; }
  if (fseq->type != J_INT || fseq->i < 1 || fseq->i > 999999999L) {
    ackError(out, len, id, eseq, "BAD_FIELD", "seq");
    return;
  }
  if (fseq->i <= lastSeq_) { ackError(out, len, id, eseq, "BAD_SEQ", nullptr); return; }
  lastSeq_ = fseq->i;
  if (!ftype) { ackError(out, len, id, eseq, "MISSING_FIELD", "type"); return; }
  if (ftype->type != J_STR) { ackError(out, len, id, eseq, "BAD_FIELD", "type"); return; }
  const TypeSpec* spec = nullptr;
  for (int k = 0; k < kNumTypes; ++k)
    if (strcmp(kTypes[k].name, ftype->s) == 0) spec = &kTypes[k];
  if (!spec) { ackError(out, len, id, eseq, "UNKNOWN_TYPE", nullptr); return; }
  for (int k = 0; k < o.n; ++k) {
    const char* key = o.f[k].key;
    if (!isCommon(key) && !inList(spec->required, key) && !inList(spec->optional, key)) {
      ackError(out, len, id, eseq, "UNKNOWN_FIELD", key);
      return;
    }
  }
  for (int k = 0; spec->required[k]; ++k) {
    if (!o.get(spec->required[k])) {
      ackError(out, len, id, eseq, "MISSING_FIELD", spec->required[k]);
      return;
    }
  }
  const char* type = spec->name;
  const char* bad = nullptr;      // name of the first invalid field
  const char* err = nullptr;      // state-related refusal
  int okLeft = 0, okRight = 0;
  bool withWheels = false;
  bool wantStatus = false;
  const int lim = cfg_.maxSpeedPct;
  const char* named = nullptr;    // named command, echoed in the ack
  char namedResult[48] = {0};

  if (strcmp(type, "ping") == 0) {
    // heartbeat only
  } else if (strcmp(type, "status_request") == 0) {
    wantStatus = true;
  } else if (strcmp(type, "motor") == 0) {
    static const char* const modes[] = {"manual", nullptr};
    const JField* l = o.get("left");
    const JField* r = o.get("right");
    const JField* d = o.get("duration_ms");
    if (!strOneOf(o.get("mode"), modes)) bad = "mode";
    else if (!intIn(l, -100, 100)) bad = "left";
    else if (!intIn(r, -100, 100)) bad = "right";
    else if (!intIn(d, cfg_.minMoveMs, cfg_.maxMoveMs)) bad = "duration_ms";
    else if (state_ == STATE_EMERGENCY) err = "EMERGENCY_ACTIVE";
    else if (fallAlert_ && (l->i != 0 || r->i != 0)) err = "FALL_ALERT_ACTIVE";
    else if (follow_) err = "FOLLOW_ACTIVE";
    else {
      int lv = clampAbs(static_cast<int>(l->i), lim), rv = clampAbs(static_cast<int>(r->i), lim);
      if (obstacleActive() && lv + rv > 0) {
        err = "OBSTACLE_AHEAD";
      } else {
        lastValidMs_ = nowMs;
        simMove_ = SIM_NONE;
        drive(lv, rv, nowMs, static_cast<uint32_t>(d->i));
        commFault_ = false;
        okLeft = lv; okRight = rv; withWheels = true;
        logf(cfg_.motorsEnabled ? "[MOVEMENT] motor request accepted" :
             "[MOVEMENT] motor request accepted - SIMULATION ONLY, motors disabled");
      }
    }
  } else if (strcmp(type, "stop") == 0) {
    stopMotors();
    simMove_ = SIM_NONE;
    follow_ = false;
    followTarget_ = -1;
    commFault_ = false;
    logf("[MOVEMENT] STOP - all movement cleared");
  } else if (strcmp(type, "follow_mode") == 0) {
    const JField* en = o.get("enabled");
    const JField* tg = o.get("target_id");
    if (!en || en->type != J_BOOL) bad = "enabled";
    else if (tg && !intIn(tg, 0, 65535)) bad = "target_id";
    else if (en->b && !tg) { ackError(out, len, id, eseq, "MISSING_FIELD", "target_id"); return; }
    else if (en->b) {
      if (state_ == STATE_EMERGENCY) {
        err = "EMERGENCY_ACTIVE";
      } else if (fallAlert_) {
        err = "FALL_ALERT_ACTIVE";
      } else {
        logf("[FOLLOW] Follow mode ON");
        if (!follow_) startBuzzer(BUZ_SHORT, nowMs);   // one beep: "I am following"
        simMove_ = SIM_NONE;
        follow_ = true;
        followTarget_ = tg->i;
        commFault_ = false;
      }
    } else {
      stopMotors();
      follow_ = false;
      followTarget_ = -1;
      commFault_ = false;
    }
  } else if (strcmp(type, "follow") == 0) {
    const JField* tg = o.get("target_id");
    const JField* l = o.get("left");
    const JField* r = o.get("right");
    const JField* d = o.get("duration_ms");
    const JField* c = o.get("confidence");
    if (!intIn(tg, 0, 65535)) bad = "target_id";
    else if (!intIn(l, -100, 100)) bad = "left";
    else if (!intIn(r, -100, 100)) bad = "right";
    else if (!intIn(d, cfg_.minMoveMs, cfg_.maxMoveMs)) bad = "duration_ms";
    else if (!(c->type == J_INT || c->type == J_NUM) || c->d < 0.0 || c->d > 1.0) bad = "confidence";
    else if (state_ == STATE_EMERGENCY) err = "EMERGENCY_ACTIVE";
    else if (fallAlert_) err = "FALL_ALERT_ACTIVE";
    else if (!follow_) err = "FOLLOW_NOT_ENABLED";
    else if (tg->i != followTarget_) err = "TARGET_MISMATCH";
    else if (c->d < cfg_.followMinConfidence) {
      stopMotors();               // uncertain target: do not keep moving on the old request
      err = "LOW_CONFIDENCE";
    } else {
      int lv = clampAbs(static_cast<int>(l->i), lim), rv = clampAbs(static_cast<int>(r->i), lim);
      if (obstacleActive() && (lv != 0 || rv != 0)) {
        err = "OBSTACLE_AHEAD";   // stop-on-obstacle: no following motion at all
      } else {
        lastValidMs_ = nowMs;
        drive(lv, rv, nowMs, static_cast<uint32_t>(d->i));
        commFault_ = false;
        okLeft = lv; okRight = rv; withWheels = true;
      }
    }
  } else if (strcmp(type, "led") == 0) {
    static const char* const states[] = {"on", "off", "blink", nullptr};
    const JField* st = o.get("state");
    const JField* iv = o.get("interval_ms");
    if (!strOneOf(st, states)) bad = "state";
    else if (iv && !intIn(iv, 100, 2000)) bad = "interval_ms";
    else {
      ledMode_ = strcmp(st->s, "on") == 0 ? LED_ON : strcmp(st->s, "blink") == 0 ? LED_BLINK : LED_OFF;
      ledIntervalMs_ = iv ? static_cast<uint32_t>(iv->i) : 500u;
      logf("[LED] led %s", st->s);
    }
  } else if (strcmp(type, "buzzer") == 0) {
    static const char* const pats[] = {"off", "short_beep", "warning", "emergency", "on", nullptr};
    const JField* pt = o.get("pattern");
    if (!strOneOf(pt, pats)) bad = "pattern";
    else {
      BuzzerMode m = strcmp(pt->s, "short_beep") == 0 ? BUZ_SHORT : strcmp(pt->s, "warning") == 0 ? BUZ_WARNING
                   : strcmp(pt->s, "emergency") == 0 ? BUZ_EMERGENCY : strcmp(pt->s, "on") == 0 ? BUZ_ON : BUZ_OFF;
      startBuzzer(m, nowMs);
      logf("[BUZZER] buzzer %s", pt->s);
    }
  } else if (strcmp(type, "mute") == 0) {
    if (state_ == STATE_EMERGENCY) muted_ = true;
    if (fallAlert_) fallMuted_ = true;
    buzzerMode_ = BUZ_OFF;
    logf("[BUZZER] Muted");
  } else if (strcmp(type, "emergency_stop") == 0) {
    const JField* rs = o.get("reason");
    if (!rs || rs->type != J_STR || !validId(rs->s)) bad = "reason";
    else {
      enterEmergency(rs->s, nowMs);
      logf("[EMERGENCY] Emergency stop latched (reason: %s) - LED ON, alarm buzzer", rs->s);
    }
  } else if (strcmp(type, "clear_emergency") == 0) {
    const JField* oc = o.get("operator_confirmed");
    if (!oc || oc->type != J_BOOL) bad = "operator_confirmed";
    else if (!oc->b) err = "NEEDS_CONFIRM";
    else {
      stopMotors();
      simMove_ = SIM_NONE;
      state_ = STATE_IDLE;
      follow_ = false;
      followTarget_ = -1;
      muted_ = false;
      buzzerMode_ = BUZ_OFF;
      ledMode_ = LED_OFF;
      emergencyReason_[0] = '\0';
      logf("[EMERGENCY] Cleared by the operator - all movement stays stopped");
      if (fallAlert_) logf("[ALERT] Fall alert is still active (needs FALL_CLEARED)");
      else logf(modelRunning_ ? "[LED] Model running: LED continuously ON" : "[LED] LED OFF");
    }
  } else if (strcmp(type, "obstacle") == 0) {
    static const char* const srcs[] = {"camera", nullptr};
    const JField* dc = o.get("distance_cm");
    if (!strOneOf(o.get("source"), srcs)) bad = "source";
    else if (!intIn(dc, 0, 999)) bad = "distance_cm";
    else {
      bool wasActive = obstacleActive();
      remoteObstacle_ = true;
      remoteSetMs_ = nowMs;       // (re)start the hold period
      if (!wasActive && state_ != STATE_EMERGENCY) startBuzzer(BUZ_SHORT, nowMs);
      if (state_ == STATE_MOVING && (follow_ || left_ + right_ > 0)) stopMotors();
      if (simMove_ == SIM_FORWARD) simMove_ = SIM_NONE;
    }
  } else if (strcmp(type, "obstacle_clear") == 0) {
    bool wasActive = obstacleActive();
    remoteObstacle_ = false;
    if (wasActive && !obstacleActive()) pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"obstacle_clear\"}");
  } else if (strcmp(type, "command") == 0) {
    const JField* nm = o.get("command");
    if (!nm || nm->type != J_STR) bad = "command";
    else if (!inList(kNamed, nm->s)) err = "UNKNOWN_COMMAND";
    else err = handleNamed(nm->s, o.get("duration_ms"), nowMs, wantStatus, bad, namedResult, sizeof namedResult);
    named = nm ? nm->s : nullptr;
  }

  if (bad) { ackError(out, len, id, eseq, "BAD_FIELD", bad); return; }
  if (err) { ackError(out, len, id, eseq, err, nullptr); return; }
  lastValidMs_ = nowMs;           // only accepted commands feed the watchdog
  snprintf(lastCommandId_, sizeof lastCommandId_, "%s", id);
  updateIndicators(nowMs);
  if (withWheels)
    snprintf(out, len, "{\"version\":3,\"type\":\"ack\",\"command_id\":\"%s\",\"seq\":%ld,\"ok\":true,\"left\":%d,\"right\":%d}",
             id, eseq, okLeft, okRight);
  else if (named)
    snprintf(out, len,
             "{\"version\":3,\"type\":\"ack\",\"command_id\":\"%s\",\"seq\":%ld,\"ok\":true,\"command\":\"%s\","
             "\"result\":\"%s\"}",
             id, eseq, named, namedResult);
  else
    snprintf(out, len, "{\"version\":3,\"type\":\"ack\",\"command_id\":\"%s\",\"seq\":%ld,\"ok\":true}", id, eseq);
  if (wantStatus) pushStatus(nowMs);
}

// Named commands: a readable alias layer over the same safety rules. Movement names are SIMULATED.
const char* RobotCore::handleNamed(const char* name, const JField* dur, uint32_t nowMs, bool& wantStatus,
                                   const char*& bad, char* result, size_t rlen) {
  SimMove mv = strcmp(name, "FORWARD") == 0 ? SIM_FORWARD : strcmp(name, "BACKWARD") == 0 ? SIM_BACKWARD
             : strcmp(name, "LEFT") == 0 ? SIM_LEFT : strcmp(name, "RIGHT") == 0 ? SIM_RIGHT : SIM_NONE;
  bool isMove = mv != SIM_NONE;
  if (dur && (!isMove || !intIn(dur, cfg_.minMoveMs, cfg_.maxMoveMs))) {
    bad = "duration_ms";          // only movement names take a duration, within the movement limits
    return nullptr;
  }
  if (isMove) {
    if (state_ == STATE_EMERGENCY) return "EMERGENCY_ACTIVE";
    if (fallAlert_) return "FALL_ALERT_ACTIVE";
    if (follow_) return "FOLLOW_ACTIVE";
    if (mv == SIM_FORWARD && obstacleActive()) return "OBSTACLE_AHEAD";
    stopMotors();                 // one control source at a time; never touches the motor outputs otherwise
    simMove_ = mv;
    simStartMs_ = nowMs;
    simDurationMs_ = dur ? static_cast<uint32_t>(dur->i) : cfg_.simMoveDefaultMs;
    commFault_ = false;
    snprintf(result, rlen, "SIMULATED_MOVEMENT_RECEIVED: %s", name);
    logf("[MOVEMENT] %s received - SIMULATION ONLY, motors disabled", name);
    return nullptr;
  }
  if (strcmp(name, "STOP") == 0) {
    stopMotors();
    simMove_ = SIM_NONE;
    follow_ = false;
    followTarget_ = -1;
    commFault_ = false;
    snprintf(result, rlen, "STOPPED");
    logf("[MOVEMENT] STOP received - simulated movement cleared (no motors)");
  } else if (strcmp(name, "LED_ON") == 0 || strcmp(name, "LED_OFF") == 0 || strcmp(name, "LED_BLINK") == 0) {
    ledMode_ = strcmp(name, "LED_ON") == 0 ? LED_ON : strcmp(name, "LED_BLINK") == 0 ? LED_BLINK : LED_OFF;
    ledIntervalMs_ = 500u;
    snprintf(result, rlen, "%s", name);
    if (ledMode_ != LED_OFF) logf("[LED] Test command: %s", name + 4);
    else logf("[LED] Test command OFF: LED back to automatic state (%s)", modelRunning_ ? "ON - model running" : "off");
    if (state_ == STATE_EMERGENCY || fallAlert_)
      logf("[LED] (not visible now: the %s pattern has priority)", state_ == STATE_EMERGENCY ? "emergency" : "fall alert");
  } else if (strncmp(name, "BUZZER_", 7) == 0) {
    const char* p = name + 7;
    BuzzerMode m = strcmp(p, "ON") == 0 ? BUZ_ON : strcmp(p, "SHORT_BEEP") == 0 ? BUZ_SHORT
                 : strcmp(p, "WARNING") == 0 ? BUZ_WARNING : BUZ_OFF;
    if (m == BUZ_OFF && state_ == STATE_EMERGENCY) muted_ = true;   // BUZZER_OFF silences an emergency too
    if (m == BUZ_OFF && fallAlert_) fallMuted_ = true;               // ... and the fall alert buzzer (LED keeps blinking)
    startBuzzer(m, nowMs);
    snprintf(result, rlen, "%s", name);
    if (m == BUZ_OFF) logf("[BUZZER] OFF%s", (state_ == STATE_EMERGENCY || fallAlert_) ? " - alarm muted, alert stays active" : "");
    else if (state_ == STATE_EMERGENCY || fallAlert_) logf("[BUZZER] Test %s ignored while an alarm is active", p);
    else logf("[BUZZER] Test command: %s%s", p, m == BUZ_ON ? " (stops by itself after 60 s)" : "");
  } else if (strcmp(name, "STATUS") == 0) {
    wantStatus = true;
    snprintf(result, rlen, "STATUS_SENT");
    logf("[STATUS] Status sent to the laptop");
  } else if (strcmp(name, "EMERGENCY_STOP") == 0) {
    enterEmergency("operator", nowMs);
    snprintf(result, rlen, "EMERGENCY_LATCHED");
    logf("[EMERGENCY] Emergency stop latched - LED ON, alarm buzzer. Reset needs operator confirmation");
  } else if (strcmp(name, "MODEL_STARTED") == 0) {
    bool was = modelRunning_;
    modelRunning_ = true;
    ledMode_ = LED_OFF;           // end any LED test so the "model running" light is visible
    snprintf(result, rlen, "MODEL_RUNNING");
    if (state_ == STATE_EMERGENCY) logf("[LED] Model running - emergency active, LED stays ON (emergency has priority)");
    else if (fallAlert_) logf("[LED] Model running - fall alert active, LED keeps blinking");
    else logf(was ? "[LED] Model running (already): LED continuously ON" : "[LED] Model running: LED continuously ON");
  } else if (strcmp(name, "MODEL_STOPPED") == 0) {
    modelRunning_ = false;
    fallAlert_ = false;           // nothing watches any more: end the fall alert too
    fallMuted_ = false;
    ledMode_ = LED_OFF;
    buzzerMode_ = BUZ_OFF;
    snprintf(result, rlen, "MODEL_STOPPED");
    if (state_ == STATE_EMERGENCY) {
      logf("[LED] Model stopped - emergency still latched: LED stays ON until the operator reset");
    } else {
      logf("[LED] Model stopped: LED OFF");
      logf("[BUZZER] OFF");
    }
  } else if (strcmp(name, "FALL_DETECTED") == 0) {
    snprintf(result, rlen, "FALL_ALERT_ACTIVE");
    if (fallAlert_) {             // repeated event: never restart or stack the alert
      logf("[ALERT] Fall alert already active - not restarted");
    } else {
      fallAlert_ = true;
      fallMuted_ = false;
      fallBuzzReported_ = false;
      fallStartMs_ = nowMs;
      stopMotors();               // a fall stops every (simulated) movement and following
      simMove_ = SIM_NONE;
      follow_ = false;
      followTarget_ = -1;
      logf("[ALERT] Fall detected");
      if (state_ == STATE_EMERGENCY) {
        logf("[LED] Emergency active: LED stays ON (emergency has priority over the fall alert)");
      } else {
        logf("[LED] Fall alert: blinking");
        logf("[BUZZER] Fall warning active");
      }
    }
  } else if (strcmp(name, "FALL_CLEARED") == 0) {
    bool was = fallAlert_;
    fallAlert_ = false;
    fallMuted_ = false;
    snprintf(result, rlen, was ? "FALL_ALERT_CLEARED" : "NO_FALL_ALERT");
    if (!was) {
      logf("[ALERT] No fall alert was active");
    } else {
      logf("[ALERT] Fall alert cleared");
      if (state_ == STATE_EMERGENCY) logf("[LED] Emergency still latched: LED stays ON");
      else logf(modelRunning_ ? "[LED] Model running: LED continuously ON" : "[LED] Model not running: LED OFF");
      logf("[BUZZER] Alert stopped");
    }
  }
  return nullptr;
}

void RobotCore::tick(uint32_t nowMs) {
  if (remoteObstacle_ && nowMs - remoteSetMs_ >= cfg_.obstacleHoldMs) {   // camera report expired
    remoteObstacle_ = false;
    if (!obstacleActive()) pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"obstacle_clear\"}");
  }
  if (state_ == STATE_MOVING) {
    // unsigned subtraction stays correct when millis() wraps around after ~49 days
    if (nowMs - lastValidMs_ > cfg_.watchdogMs) {
      stopMotors();
      ++watchdogTrips_;
      commFault_ = true;
      pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"watchdog_stop\"}");
    } else if (nowMs - moveStartMs_ >= moveDurationMs_) {
      stopMotors();
      pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"move_done\"}");
    }
  }
  if (simMove_ != SIM_NONE) {    // simulated movement follows the same watchdog / time-out rules
    if (nowMs - lastValidMs_ > cfg_.watchdogMs) {
      simMove_ = SIM_NONE;
      ++watchdogTrips_;
      commFault_ = true;
      pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"watchdog_stop\",\"simulated\":true}");
    } else if (nowMs - simStartMs_ >= simDurationMs_) {
      simMove_ = SIM_NONE;
      pushMessage("{\"version\":3,\"type\":\"event\",\"event\":\"move_done\",\"simulated\":true}");
    }
  }
  updateIndicators(nowMs);
  if (connected_ && nowMs - lastStatusMs_ >= cfg_.statusIntervalMs) pushStatus(nowMs);
}

}  // namespace sentinel
