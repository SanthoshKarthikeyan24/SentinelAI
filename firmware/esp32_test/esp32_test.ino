// =====================================================================
//  SentinelAI - ESP32 COMMUNICATION TEST firmware (stage 1)
//  Board: ESP32 DevKit V1 ("DOIT ESP32 DEVKIT V1" in the Arduino IDE)
//
//  What it does
//    * joins your Wi-Fi (or makes its own), prints its IP on the Serial Monitor
//    * TCP server on port 3333; the laptop connects and sends one JSON object per line
//      (protocol v3 - docs/PROTOCOL.md; named commands - docs/STAGE1_COMM_TEST.md)
//    * every message is validated and gets exactly one acknowledgement
//    * LED_ON / LED_OFF / LED_BLINK and BUZZER_* drive the red LED and the buzzer
//    * FORWARD / BACKWARD / LEFT / RIGHT / STOP are SIMULATED: recorded, acknowledged
//      with "SIMULATED_MOVEMENT_RECEIVED: <name>", reported in the status - nothing else.
//    * AI events from the laptop: MODEL_STARTED (LED continuously ON), FALL_DETECTED
//      (LED blinks + buzzer beep-beep pattern), FALL_CLEARED (back to LED ON),
//      MODEL_STOPPED (LED and buzzer OFF). All timing is non-blocking (millis()).
//    * every received command is printed in plain words on the Serial Monitor (115200 baud)
//    * status every second; EMERGENCY_STOP latches until the laptop sends
//      clear_emergency with operator_confirmed:true
//    * watchdog: if the laptop goes silent for 500 ms during a simulated movement, the
//      movement is cleared and the fault is reported when the laptop reconnects.
//      A reconnection never restores an old movement request.
//
//  THERE IS NO MOTOR CODE IN THIS SKETCH. No motor pin is configured or written.
//
//  The protocol/safety logic is robot_core.cpp - the SAME file as in
//  firmware/sentinel_robot/ (kept identical by tools/sync_firmware_core.py and a test),
//  so everything tested here carries over to the robot firmware.
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

using sentinel::LineReader;
using sentinel::RobotConfig;
using sentinel::RobotCore;

static const char* const FIRMWARE_NAME = "SentinelAI ESP32 communication test";

// --------------------------------------------------------- buzzer tone helper
#if BUZZER_PASSIVE
static void toneOut(uint32_t hz) {
#if SENTINEL_CORE3
  static bool attached = false;
  if (!attached) {
    ledcAttach(PIN_BUZZER, BUZZER_TONE_HZ, 8);
    attached = true;
  }
  ledcWriteTone(PIN_BUZZER, hz);
#else
  static bool attached = false;
  if (!attached) {
    ledcSetup(2, BUZZER_TONE_HZ, 8);
    ledcAttachPin(PIN_BUZZER, 2);
    attached = true;
  }
  ledcWriteTone(2, hz);
#endif
}
#endif

// ---------------------------------------------------------- hardware layer
// Only the red LED and the buzzer. setMotors() is required by the core's interface but
// does nothing except remember the (always zero) request for the Serial Monitor.
class TestOutputs : public sentinel::RobotOutputs {
 public:
  void begin() {
#if LED_INSTALLED
    pinMode(PIN_RED_LED, OUTPUT);
    digitalWrite(PIN_RED_LED, LOW);
#endif
#if BUZZER_INSTALLED
#if BUZZER_PASSIVE
    toneOut(0);
#else
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#endif
#endif
  }

  void setMotors(int left, int right) override {   // no motors in this firmware
    lastLeft = left;
    lastRight = right;
  }

  void setAlarm(bool buzzer, bool led) override {
    buzzerOn = buzzer;
    ledOn = led;
#if LED_INSTALLED
    digitalWrite(PIN_RED_LED, led ? HIGH : LOW);
#endif
#if BUZZER_INSTALLED
#if BUZZER_PASSIVE
    toneOut(buzzer ? BUZZER_TONE_HZ : 0);
#else
    digitalWrite(PIN_BUZZER, buzzer ? HIGH : LOW);
#endif
#endif
  }

  int lastLeft = 0, lastRight = 0;
  bool buzzerOn = false, ledOn = false;
};

// ---------------------------------------------------------------- globals
static TestOutputs outputs;
static RobotConfig makeConfig() {
  RobotConfig c;
  c.watchdogMs = WATCHDOG_MS;
  c.motorsEnabled = false;          // fixed: this firmware has no motor code
  c.sensorEnabled = false;
  c.statusIntervalMs = STATUS_INTERVAL_MS;
  c.emergencyBuzzMaxMs = BUZZER_MAX_MS;
  c.simMoveDefaultMs = SIM_MOVE_MS;
  c.ledInstalled = LED_INSTALLED != 0;
  c.buzzerInstalled = BUZZER_INSTALLED != 0;
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

static void sendLine(const char* text, bool log) {
  if (clientActive && client.connected()) {
    client.print(text);
    client.print('\n');
  }
#if LOG_RAW_JSON
  if (log) {
    Serial.print(F("[TX] "));
    Serial.println(text);
  }
#else
  (void)log;
#endif
}

// Print the core's plain-language description of what it just did (one or more lines).
static void printCoreLog() {
#if LOG_COMMANDS
  const char* t = core.lastLog();
  if (t[0]) Serial.println(t);
#endif
}

// Plain-language text for messages the core sends by itself.
static void printEvent(const char* msg) {
  if (strstr(msg, "\"event\":\"watchdog_stop\"")) {
    Serial.println(F("[SAFETY] Laptop silent for 0.5 s - simulated movement cleared (watchdog)"));
  } else if (strstr(msg, "\"event\":\"move_done\"")) {
    Serial.println(F("[MOVEMENT] Simulated movement finished (time limit reached)"));
  } else if (strstr(msg, "\"event\":\"buzzer_timeout\"")) {
    Serial.println(F("[BUZZER] Stopped after the 60 s limit (the alert itself stays active)"));
  } else if (strstr(msg, "\"event\":\"obstacle_clear\"")) {
    Serial.println(F("[OBSTACLE] Cleared"));
  } else {
    Serial.printf("[EVENT] %s\n", msg);
  }
}

static void dropClient(const char* why) {
  if (clientActive) {
    Serial.printf("[TCP] Laptop client disconnected (%s) - simulated movement cleared, model LED off\n", why);
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
  Serial.printf("[WiFi] Access point '%s' %s\n", AP_SSID, ok ? "started" : "FAILED");
  Serial.printf("[WiFi] ESP32 IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("[TCP] Listening on port %d\n", TCP_PORT);
  networkUp = ok;
#else
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ROBOT_HOSTNAME);
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY), IPAddress(STATIC_SUBNET));
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to '%s' ...\n", WIFI_SSID);   // the password is never printed
#endif
  lastWifiAttemptMs = millis();
}

static void maintainWifi() {
#if SENTINEL_WIFI_MODE == SENTINEL_MODE_STATION
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !networkUp) {
    networkUp = true;
    Serial.println(F("[WiFi] Connected"));
    Serial.printf("[WiFi] ESP32 IP: %s   (signal %d dBm)\n", WiFi.localIP().toString().c_str(),
                  static_cast<int>(WiFi.RSSI()));
    Serial.printf("[TCP] Listening on port %d\n", TCP_PORT);
    Serial.println(F("[WiFi] Use this IP with: python -m robot.test_console --host <IP>"));
    if (MDNS.begin(ROBOT_HOSTNAME)) MDNS.addService("sentinel", "tcp", TCP_PORT);
  } else if (!up && networkUp) {
    networkUp = false;
    Serial.println(F("[WiFi] Disconnected - laptop link closed, simulated movement cleared. Reconnecting..."));
    dropClient("wifi lost");
  } else if (!up && millis() - lastWifiAttemptMs > 10000) {
    Serial.println(F("[WiFi] Still not connected - retrying (check name/password in secrets.h; 2.4 GHz only)"));
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
    // The newest connection wins: after a laptop crash the old socket can look alive for minutes.
    if (client.connected()) client.print("{\"version\":3,\"type\":\"event\",\"event\":\"replaced\"}\n");
    dropClient("replaced by a new connection");
  }
  client = incoming;
  client.setNoDelay(true);
  clientActive = true;
  reader.reset();
  core.onClientConnected(millis(), replyBuf, sizeof replyBuf);
  Serial.printf("[TCP] Laptop client connected (from %s)\n", client.remoteIP().toString().c_str());
  sendLine(replyBuf, true);
}

static bool isQuiet(const char* line) {   // heartbeats are not printed (10 per second)
  return strstr(line, "\"type\":\"ping\"") != nullptr;
}

static void readClient() {
  if (!clientActive) return;
  if (!client.connected()) {
    dropClient("closed");
    return;
  }
  int budget = 256;   // bounded work per loop so the watchdog check always runs
  while (budget-- > 0 && client.available() > 0) {
    char c = static_cast<char>(client.read());
    int r = reader.feed(c);
    if (r == 1) {
      bool log = !isQuiet(reader.line());
#if LOG_RAW_JSON
      if (log) {
        Serial.print(F("[RX-JSON] "));
        Serial.println(reader.line());
      }
#endif
      core.handleLine(reader.line(), millis(), replyBuf, sizeof replyBuf);
      printCoreLog();
      sendLine(replyBuf, log || strstr(replyBuf, "\"ok\":false") != nullptr);
    } else if (r == -1) {
      core.handleOverflow(replyBuf, sizeof replyBuf);
      printCoreLog();
      sendLine(replyBuf, true);
    }
  }
}

// ------------------------------------------------------------ status LED
static void updateStatusLed() {
#if PIN_STATUS_LED >= 0
  uint32_t t = millis();
  bool on;
  if (core.state() == sentinel::STATE_EMERGENCY) on = (t / 100) % 2;     // 5 Hz: emergency
  else if (!networkUp) on = (t / 500) % 2;                                // 1 Hz: no Wi-Fi
  else if (!clientActive) on = (t % 2000) < 100;                          // short flash: waiting for laptop
  else on = true;                                                         // solid: laptop connected
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

// Blink the red LED 3x and beep once at power-up: a wiring check that needs no Wi-Fi.
static void selfTest() {
#if STARTUP_SELFTEST
  Serial.println(F("[SELFTEST] Red LED blinks 3 times, buzzer beeps once"));
  for (int i = 0; i < 3; ++i) {
    outputs.setAlarm(false, true);
    delay(150);
    outputs.setAlarm(false, false);
    delay(150);
  }
  outputs.setAlarm(true, false);
  delay(120);
  outputs.setAlarm(false, false);
#endif
}

// ------------------------------------------------------------- Arduino API
void setup() {
  outputs.begin();                       // safe output states before anything else
#if PIN_STATUS_LED >= 0
  pinMode(PIN_STATUS_LED, OUTPUT);
#endif
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.printf("[BOOT] %s - protocol v%d\n", FIRMWARE_NAME, sentinel::PROTOCOL_VERSION);
  Serial.println(F("[BOOT] Motors: NONE in this firmware - movement commands are simulated only"));
  Serial.println(F("[BOOT] Safe start: LED off, buzzer off, no movement, waiting for the laptop"));
#if LED_INSTALLED
  Serial.printf("[BOOT] Red LED on GPIO %d\n", PIN_RED_LED);
#else
  Serial.println(F("[BOOT] Red LED: not installed (commands are still accepted and reported)"));
#endif
#if BUZZER_INSTALLED
  Serial.printf("[BOOT] Buzzer (%s) on GPIO %d via transistor\n", BUZZER_PASSIVE ? "passive" : "active", PIN_BUZZER);
#else
  Serial.println(F("[BOOT] Buzzer: not installed (commands are still accepted and reported)"));
#endif
  selfTest();
  core.begin(millis());
  startWifi();
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  maintainWifi();
  if (networkUp) acceptClient();
  readClient();
  core.tick(millis());
  while (core.popMessage(replyBuf, sizeof replyBuf)) {
    if (strstr(replyBuf, "\"type\":\"status\"") != nullptr) {
      sendLine(replyBuf, false);          // periodic status: sent, not printed every second
    } else {
      printEvent(replyBuf);
      sendLine(replyBuf, false);
    }
  }
  updateStatusLed();
  delay(1);   // yield to the Wi-Fi stack
}
