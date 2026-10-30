// SentinelAI robot controller core - protocol v3 (JSON lines) and all safety logic.
//
// Plain C++ (no Arduino headers): exactly the same code runs on the ESP32,
// in the PC unit tests, in the transcript replay tool and in the PC simulator.
// Hardware goes through RobotOutputs. Specification: docs/PROTOCOL.md.
#ifndef SENTINEL_ROBOT_CORE_H
#define SENTINEL_ROBOT_CORE_H

#include <stddef.h>
#include <stdint.h>

namespace sentinel {

static const int PROTOCOL_VERSION = 3;
static const size_t MAX_LINE = 200;    // longest accepted laptop -> robot line (without '\n')
static const size_t OUT_LEN = 640;     // buffer size callers must provide for outgoing messages
static const size_t LOG_LEN = 400;     // human-readable log of the last handled line (Serial Monitor)
static const int MAX_FIELDS = 12;      // JSON members per message
static const size_t MAX_KEY = 23;
static const size_t MAX_STR = 32;

// ------------------------------------------------------------ strict JSON subset
enum JType { J_STR = 0, J_INT = 1, J_NUM = 2, J_BOOL = 3, J_NULL = 4 };

struct JField {
  char key[MAX_KEY + 1];
  JType type;
  char s[MAX_STR + 1];
  long i;
  double d;     // numeric value for J_INT and J_NUM
  bool b;
};

struct JObject {
  JField f[MAX_FIELDS];
  int n = 0;
  const JField* get(const char* key) const;
};

// Parses one flat JSON object (see docs/PROTOCOL.md section 1). Returns false on any violation.
bool parseJsonObject(const char* text, JObject& out);

// ------------------------------------------------------------------- hardware
class RobotOutputs {
 public:
  virtual ~RobotOutputs() {}
  // left/right: -100..100 percent of full PWM; 0 = stop. Positive = forward.
  virtual void setMotors(int left, int right) = 0;
  // Current buzzer and red-LED level (patterns are produced by the core).
  virtual void setAlarm(bool buzzer, bool led) = 0;
};

struct RobotConfig {
  uint32_t watchdogMs = 500;          // stop motors if no valid message for this long while moving
  uint32_t minMoveMs = 50;            // shortest allowed movement duration
  uint32_t maxMoveMs = 2000;          // longest allowed movement duration (never indefinite)
  int maxSpeedPct = 60;               // wheel speeds above this are clamped (safety)
  bool motorsEnabled = false;         // Phase 2: false = never drive the motor pins
  uint32_t obstacleHoldMs = 1500;     // a laptop obstacle report blocks motion this long
  bool sensorEnabled = false;         // optional distance sensor on the robot
  int obstacleStopCm = 25;            // sensor: closer than this = obstacle
  int obstacleClearCm = 35;           // sensor: cleared when farther than this (hysteresis)
  uint32_t statusIntervalMs = 1000;   // periodic status while a laptop is connected
  uint32_t emergencyBuzzMaxMs = 60000;  // emergency buzzer stops after this (LED stays on)
  double followMinConfidence = 0.5;   // follow requests below this are refused (and stop the motors)
  uint32_t simMoveDefaultMs = 2000;   // named FORWARD/BACKWARD/LEFT/RIGHT: simulated, ends by itself
  uint32_t fallBlinkMs = 250;         // LED blink half-period while a fall alert is active
  bool ledInstalled = true;           // reported in hello (the firmware cannot sense the LED itself)
  bool buzzerInstalled = true;        // reported in hello
};

enum RobotState { STATE_IDLE = 0, STATE_MOVING = 1, STATE_EMERGENCY = 2 };
enum LedMode { LED_OFF = 0, LED_ON = 1, LED_BLINK = 2 };
enum BuzzerMode { BUZ_OFF = 0, BUZ_SHORT = 1, BUZ_WARNING = 2, BUZ_EMERGENCY = 3, BUZ_ON = 4 };
// Named movement commands are SIMULATED in this firmware version: they are validated,
// recorded, acknowledged and reported, but never reach the motor outputs.
enum SimMove { SIM_NONE = 0, SIM_FORWARD = 1, SIM_BACKWARD = 2, SIM_LEFT = 3, SIM_RIGHT = 4 };

const char* ledName(LedMode m);
const char* buzzerName(BuzzerMode m);
const char* simMoveName(SimMove m);
bool validId(const char* s);   // 1..24 chars of A-Z a-z 0-9 _ - .

// Collects bytes into lines; rejects over-long lines without buffering them.
class LineReader {
 public:
  // Returns 1 when a complete line is in line(), -1 when an over-long line ended, 0 otherwise.
  int feed(char c);
  const char* line() const { return buf_; }
  void reset() { len_ = 0; overflow_ = false; buf_[0] = '\0'; }

 private:
  char buf_[MAX_LINE + 1] = {0};
  size_t len_ = 0;
  bool overflow_ = false;
};

class RobotCore {
 public:
  RobotCore(RobotOutputs& out, const RobotConfig& cfg);

  void begin(uint32_t nowMs);
  void onClientConnected(uint32_t nowMs, char* hello, size_t len);
  void onClientDisconnected(uint32_t nowMs);
  // Handles one complete line; always writes exactly one acknowledgement.
  void handleLine(const char* line, uint32_t nowMs, char* ack, size_t len);
  void handleOverflow(char* ack, size_t len);
  // Distance in cm from the optional sensor (negative = no echo). Ignored if no sensor.
  void onDistance(int cm, uint32_t nowMs);
  // Call often: obstacle hold, watchdog, movement time-out, buzzer limits, periodic status, indicators.
  void tick(uint32_t nowMs);
  // Outgoing status/event messages queued by tick()/onDistance()/status_request.
  bool popMessage(char* msg, size_t len);
  void buildStatus(uint32_t nowMs, char* msg, size_t len) const;

  RobotState state() const { return state_; }
  const char* stateName() const;
  int left() const { return left_; }
  int right() const { return right_; }
  bool followMode() const { return follow_; }
  long followTarget() const { return followTarget_; }
  bool obstacleActive() const { return localObstacle_ || remoteObstacle_; }
  int distanceCm() const { return distanceCm_; }
  uint32_t watchdogTrips() const { return watchdogTrips_; }
  bool commFault() const { return commFault_; }
  uint32_t rejected() const { return rejected_; }
  bool buzzerOn() const { return buzzerLevel_; }
  bool ledOn() const { return ledLevel_; }
  bool clientConnected() const { return connected_; }
  SimMove simulatedMovement() const { return simMove_; }
  bool modelRunning() const { return modelRunning_; }
  bool fallAlert() const { return fallAlert_; }
  // Human-readable lines (separated by '\n') describing what the last handleLine()/handleOverflow()
  // did - printed on the Serial Monitor. Empty for heartbeats.
  const char* lastLog() const { return log_; }

 private:
  void ackError(char* out, size_t len, const char* id, long seq, const char* code, const char* field);
  bool drive(int left, int right, uint32_t nowMs, uint32_t durationMs);
  void stopMotors();
  void enterEmergency(const char* reason, uint32_t nowMs);
  void startBuzzer(BuzzerMode m, uint32_t nowMs);
  void updateIndicators(uint32_t nowMs);
  void pushMessage(const char* text);
  void pushStatus(uint32_t nowMs);
  bool buzzerSounding(uint32_t nowMs) const;
  bool fallBuzzerActive(uint32_t nowMs) const;
  void logf(const char* fmt, const char* a = "", const char* b = "", const char* c = "");
  // named "command" messages; returns the error code (nullptr = accepted), sets result text
  const char* handleNamed(const char* name, const struct JField* dur, uint32_t nowMs, bool& wantStatus,
                          const char*& bad, char* result, size_t rlen);

  RobotOutputs& out_;
  RobotConfig cfg_;
  RobotState state_ = STATE_IDLE;
  int left_ = 0;
  int right_ = 0;
  bool follow_ = false;
  long followTarget_ = -1;
  bool connected_ = false;
  bool commFault_ = false;
  uint32_t lastValidMs_ = 0;
  uint32_t moveStartMs_ = 0;
  uint32_t moveDurationMs_ = 0;
  SimMove simMove_ = SIM_NONE;
  uint32_t simStartMs_ = 0;
  uint32_t simDurationMs_ = 0;
  // AI state reported by the laptop
  bool modelRunning_ = false;
  bool fallAlert_ = false;
  bool fallMuted_ = false;
  bool fallBuzzReported_ = false;
  uint32_t fallStartMs_ = 0;
  char log_[LOG_LEN] = {0};
  const char* curLabel_ = "message";
  long lastSeq_ = 0;
  uint32_t watchdogTrips_ = 0;
  uint32_t rejected_ = 0;
  uint32_t bootMs_ = 0;
  uint32_t lastStatusMs_ = 0;
  char lastCommandId_[MAX_STR + 1] = {0};
  // emergency
  char emergencyReason_[MAX_STR + 1] = {0};
  uint32_t emergencyStartMs_ = 0;
  bool muted_ = false;
  bool emergencyBuzzReported_ = false;
  // obstacles
  bool localObstacle_ = false;
  bool remoteObstacle_ = false;
  uint32_t remoteSetMs_ = 0;
  int distanceCm_ = -1;
  // indicators
  LedMode ledMode_ = LED_OFF;
  uint32_t ledIntervalMs_ = 500;
  BuzzerMode buzzerMode_ = BUZ_OFF;
  uint32_t buzzerStartMs_ = 0;
  bool buzzerLevel_ = false;
  bool ledLevel_ = false;
  bool outputsInitialised_ = false;
  // outgoing queue
  static const int OUT_QUEUE = 6;
  char queue_[OUT_QUEUE][OUT_LEN];
  int qHead_ = 0;
  int qCount_ = 0;
};

}  // namespace sentinel

#endif  // SENTINEL_ROBOT_CORE_H
