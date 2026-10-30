// =====================================================================
//  SentinelAI robot controller - ESP32 DevKit V1 (Arduino core for ESP32)
//
//  * Wi-Fi TCP server on TCP_PORT; one laptop connection at a time.
//  * Protocol v3: one JSON object per line (docs/PROTOCOL.md, docs/COMMANDS.md):
//    motor, stop, follow_mode, follow, led, buzzer, mute, emergency_stop,
//    clear_emergency, obstacle, obstacle_clear, ping, status_request.
//    Status every second. Optional HC-SR04 obstacle sensor.
//  * Safety: motors stop on STOP, watchdog timeout, movement time-out,
//    disconnection, Wi-Fi loss and FALL/EMERGENCY. Emergency latches the
//    buzzer + red LED until clear_emergency (operator_confirmed) or a power cycle.
//  * Protocol and safety logic live in robot_core.cpp (tested on a PC);
//    this file only connects them to Wi-Fi and the GPIO pins.
//
//  Settings: config.h.   Tested with: Arduino core for ESP32 3.3.12,
//  board "DOIT ESP32 DEVKIT V1". Core 2.0.x is supported via #if blocks.
// =====================================================================
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "config.h"
#include "robot_core.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define SENTINEL_CORE3 1
#else
#define SENTINEL_CORE3 0
#endif

using sentinel::RobotCore;
using sentinel::RobotConfig;
using sentinel::LineReader;

// ------------------------------------------------------------- PWM helpers
// The LEDC (PWM) API changed between core 2.x and 3.x.
static const int CH_LEFT = 0, CH_RIGHT = 1, CH_BUZZER = 2;

[[maybe_unused]] static void pwmAttach(int pin, int channel, int freq, int bits) {
#if SENTINEL_CORE3
  (void)channel;
  ledcAttach(pin, freq, bits);
#else
  ledcSetup(channel, freq, bits);
  ledcAttachPin(pin, channel);
#endif
}

[[maybe_unused]] static void pwmWrite(int pin, int channel, uint32_t duty) {
#if SENTINEL_CORE3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

[[maybe_unused]] static void pwmTone(int pin, int channel, uint32_t hz) {
#if SENTINEL_CORE3
  (void)channel;
  ledcWriteTone(pin, hz);
#else
  (void)pin;
  ledcWriteTone(channel, hz);
#endif
}

// ---------------------------------------------------------- hardware layer
class EspOutputs : public sentinel::RobotOutputs {
 public:
  void begin() {
    // Motor pins LOW first, before anything else can happen.
    const int pins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB};
    for (int p : pins) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
#if MOTORS_ENABLED
    pwmAttach(PIN_ENA, CH_LEFT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    pwmAttach(PIN_ENB, CH_RIGHT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    pwmWrite(PIN_ENA, CH_LEFT, 0);
    pwmWrite(PIN_ENB, CH_RIGHT, 0);
#endif
    pinMode(PIN_RED_LED, OUTPUT);
    digitalWrite(PIN_RED_LED, LOW);
#if BUZZER_PASSIVE
    pwmAttach(PIN_BUZZER, CH_BUZZER, BUZZER_TONE_HZ, 8);
    pwmWrite(PIN_BUZZER, CH_BUZZER, 0);
#else
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#endif
  }

  void setMotors(int left, int right) override {
    lastLeft = left;
    lastRight = right;
#if MOTORS_ENABLED
    driveOne(LEFT_MOTOR_INVERT ? -left : left, PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT, LEFT_TRIM_PCT);
    driveOne(RIGHT_MOTOR_INVERT ? -right : right, PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, RIGHT_TRIM_PCT);
#else
    // Phase 2: outputs stay LOW whatever is requested.
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
#endif
  }

  void setAlarm(bool buzzer, bool led) override {
    buzzerOn = buzzer;
    ledOn = led;
    digitalWrite(PIN_RED_LED, led ? HIGH : LOW);
#if BUZZER_PASSIVE
    pwmTone(PIN_BUZZER, CH_BUZZER, buzzer ? BUZZER_TONE_HZ : 0);
#else
    digitalWrite(PIN_BUZZER, buzzer ? HIGH : LOW);
#endif
  }

  int lastLeft = 0, lastRight = 0;
  bool buzzerOn = false, ledOn = false;

 private:
#if MOTORS_ENABLED
  // speed -100..100 % -> PWM duty. Calibration (config.h, docs/HARDWARE.md):
  //   MIN_DUTY_PCT: duty where the wheel starts turning (dead-zone compensation)
  //   trimPct: scales one side so both wheels turn at the same speed (straight driving)
  void driveOne(int speed, int inA, int inB, int en, int ch, int trimPct) {
    const uint32_t maxDuty = (1u << MOTOR_PWM_BITS) - 1u;
    uint32_t pct = speed == 0 ? 0u
                 : static_cast<uint32_t>(MIN_DUTY_PCT + abs(speed) * (100 - MIN_DUTY_PCT) / 100);
    pct = pct * static_cast<uint32_t>(trimPct) / 100u;
    if (pct > 100u) pct = 100u;
    uint32_t duty = pct * maxDuty / 100u;
    if (speed == 0) {
      pwmWrite(en, ch, 0);            // speed 0 first, then release direction pins
      digitalWrite(inA, LOW);
      digitalWrite(inB, LOW);
      return;
    }
    digitalWrite(inA, speed > 0 ? HIGH : LOW);
    digitalWrite(inB, speed > 0 ? LOW : HIGH);
    pwmWrite(en, ch, duty);
  }
#endif
};

// ---------------------------------------------------------------- globals
static EspOutputs outputs;
static RobotConfig makeConfig() {
  RobotConfig c;
  c.watchdogMs = WATCHDOG_MS;
  c.minMoveMs = MIN_MOVE_MS;
  c.maxMoveMs = MAX_MOVE_MS;
  c.maxSpeedPct = MAX_SPEED_PCT;
  c.motorsEnabled = MOTORS_ENABLED != 0;
  c.obstacleHoldMs = OBSTACLE_HOLD_MS;
  c.sensorEnabled = OBSTACLE_SENSOR_ENABLED != 0;
  c.obstacleStopCm = OBSTACLE_STOP_CM;
  c.obstacleClearCm = OBSTACLE_CLEAR_CM;
  c.statusIntervalMs = STATUS_INTERVAL_MS;
  c.emergencyBuzzMaxMs = EMERGENCY_BUZZ_MAX_MS;
  c.followMinConfidence = FOLLOW_MIN_CONFIDENCE;
  return c;
}
static RobotCore core(outputs, makeConfig());
static LineReader reader;
static WiFiServer server(TCP_PORT);
static WiFiClient client;
static bool clientActive = false;
static bool networkUp = false;
static uint32_t lastWifiAttemptMs = 0;
static char replyBuf[sentinel::OUT_LEN];

static void sendLine(const char* text) {
  if (clientActive && client.connected()) {
    client.print(text);
    client.print('\n');
  }
#if LOG_COMMANDS
  Serial.print(F("  -> "));
  Serial.println(text);
#endif
}

static void sendQuiet(const char* text) {
  if (clientActive && client.connected()) {
    client.print(text);
    client.print('\n');
  }
}

static void dropClient(const char* why) {
  if (clientActive) {
    Serial.printf("[link] laptop disconnected (%s) - motors stopped\n", why);
    client.stop();
  }
  clientActive = false;
  reader.reset();
  core.onClientDisconnected(millis());
}

// ------------------------------------------------------------------ Wi-Fi
static void startWifi() {
  WiFi.persistent(false);
#if SENTINEL_WIFI_MODE == SENTINEL_MODE_ACCESS_POINT
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.printf("[wifi] access point '%s' %s, robot IP %s, port %d\n", AP_SSID, ok ? "started" : "FAILED",
                WiFi.softAPIP().toString().c_str(), TCP_PORT);
  networkUp = ok;
#else
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ROBOT_HOSTNAME);
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY), IPAddress(STATIC_SUBNET));
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[wifi] connecting to '%s' ...\n", WIFI_SSID);
#endif
  lastWifiAttemptMs = millis();
}

static void maintainWifi() {
#if SENTINEL_WIFI_MODE == SENTINEL_MODE_STATION
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !networkUp) {
    networkUp = true;
    Serial.printf("[wifi] connected. Robot IP %s  port %d  (put this IP in config/settings.yaml)\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
    if (MDNS.begin(ROBOT_HOSTNAME)) MDNS.addService("sentinel", "tcp", TCP_PORT);
  } else if (!up && networkUp) {
    networkUp = false;
    Serial.println(F("[wifi] connection lost - stopping motors"));
    dropClient("wifi lost");
  } else if (!up && millis() - lastWifiAttemptMs > 10000) {
    Serial.println(F("[wifi] still not connected, retrying (check SSID/password, 2.4 GHz)"));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttemptMs = millis();
  }
#endif
}

// ------------------------------------------------------------ connections
static void acceptClient() {
#if SENTINEL_CORE3
  WiFiClient incoming = server.accept();
#else
  WiFiClient incoming = server.available();
#endif
  if (!incoming) return;
  if (clientActive) {
    // The newest connection wins: after a laptop crash the old socket may look alive.
    if (client.connected()) client.print("{\"version\":3,\"type\":\"event\",\"event\":\"replaced\"}\n");
    dropClient("replaced by a new connection");
  }
  client = incoming;
  client.setNoDelay(true);
  clientActive = true;
  reader.reset();
  core.onClientConnected(millis(), replyBuf, sizeof replyBuf);
  Serial.printf("[link] laptop connected from %s\n", client.remoteIP().toString().c_str());
  sendLine(replyBuf);
}

static void readClient() {
  if (!clientActive) return;
  if (!client.connected()) {
    dropClient("closed");
    return;
  }
  int budget = 256;   // bound the work per loop so the watchdog check always runs
  while (budget-- > 0 && client.available() > 0) {
    char c = static_cast<char>(client.read());
    int r = reader.feed(c);
    if (r == 1) {
#if LOG_COMMANDS
      Serial.print(F("<- "));
      Serial.println(reader.line());
#endif
      core.handleLine(reader.line(), millis(), replyBuf, sizeof replyBuf);
      if (core.lastLog()[0]) Serial.println(core.lastLog());   // plain-language summary
      sendLine(replyBuf);
    } else if (r == -1) {
      core.handleOverflow(replyBuf, sizeof replyBuf);
      sendLine(replyBuf);
    }
  }
}

// ------------------------------------------------- ultrasonic sensor (optional)
// HC-SR04: a 10 us pulse on TRIG starts a measurement; ECHO goes high for a time
// proportional to the distance. The echo is timed with an interrupt, so the main
// loop (and with it the watchdog) is never blocked waiting for the echo.
// ECHO is a 5 V signal: use a voltage divider (e.g. 1 k + 2 k) to the ESP32 pin!
#if OBSTACLE_SENSOR_ENABLED
static volatile uint32_t echoRiseUs = 0;
static volatile uint32_t echoWidthUs = 0;
static volatile bool echoDone = false;
static uint32_t lastPingMs = 0;
static bool waitingEcho = false;

static void IRAM_ATTR onEcho() {
  if (digitalRead(PIN_US_ECHO)) {
    echoRiseUs = micros();
  } else {
    echoWidthUs = micros() - echoRiseUs;
    echoDone = true;
  }
}

static void sensorBegin() {
  pinMode(PIN_US_TRIG, OUTPUT);
  digitalWrite(PIN_US_TRIG, LOW);
  pinMode(PIN_US_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO), onEcho, CHANGE);
}

static void sensorPoll() {
  uint32_t now = millis();
  if (waitingEcho && echoDone) {
    waitingEcho = false;
    uint32_t width = echoWidthUs;
    int cm = width > 0 && width < 30000u ? static_cast<int>(width / 58u) : -1;   // 58 us per cm (round trip)
    core.onDistance(cm, now);
  } else if (waitingEcho && now - lastPingMs > 40) {      // no echo: nothing within ~4 m
    waitingEcho = false;
    core.onDistance(-1, now);
  }
  if (!waitingEcho && now - lastPingMs >= OBSTACLE_PING_MS) {
    echoDone = false;
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    lastPingMs = now;
    waitingEcho = true;
  }
}
#endif

// ------------------------------------------------------------ status LED
static void updateStatusLed() {
  uint32_t t = millis();
  bool on;
  if (core.state() == sentinel::STATE_EMERGENCY) on = (t / 100) % 2;       // 5 Hz: emergency
  else if (!networkUp) on = (t / 500) % 2;                                  // 1 Hz: no Wi-Fi
  else if (!clientActive) on = (t % 2000) < 100;                            // short flash: waiting
  else on = true;                                                           // solid: laptop linked
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ------------------------------------------------------------- Arduino API
void setup() {
  outputs.begin();                       // safe pin states before anything else
  pinMode(PIN_STATUS_LED, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println();
  Serial.printf("SentinelAI robot controller, protocol v%d, motors %s\n", sentinel::PROTOCOL_VERSION,
                MOTORS_ENABLED ? "ENABLED" : "DISABLED (Phase 2 safe mode)");
  core.begin(millis());
#if OBSTACLE_SENSOR_ENABLED
  sensorBegin();
  Serial.printf("Ultrasonic obstacle sensor ON (TRIG %d, ECHO %d): stop < %d cm\n", PIN_US_TRIG, PIN_US_ECHO,
                OBSTACLE_STOP_CM);
#endif
  startWifi();
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  maintainWifi();
  if (networkUp) acceptClient();
  readClient();
#if OBSTACLE_SENSOR_ENABLED
  sensorPoll();
#endif
  core.tick(millis());
  while (core.popMessage(replyBuf, sizeof replyBuf)) {
    bool isStatus = strstr(replyBuf, "\"type\":\"status\"") != nullptr;
    if (isStatus) {
      sendQuiet(replyBuf);                  // periodic status: not printed every second
    } else {
      Serial.printf("[robot] %s\n", replyBuf);
      sendLine(replyBuf);
    }
  }
  updateStatusLed();
  delay(1);   // yield to the Wi-Fi stack
}
