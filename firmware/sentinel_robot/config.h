// =====================================================================
//  SentinelAI robot - settings. Edit this file, then upload the sketch.
// =====================================================================
#ifndef SENTINEL_CONFIG_H
#define SENTINEL_CONFIG_H

// ---------------------------------------------------------------- Wi-Fi
// STATION (recommended): the ESP32 joins the same Wi-Fi as the phone and the
// laptop (e.g. the phone's hotspot or a home router), so the laptop can reach
// the phone camera and the robot at the same time. See docs/NETWORK.md.
// ACCESS_POINT: the ESP32 creates its own Wi-Fi "SentinelRobot". Then the
// phone and laptop must both join it - test that the camera still works!
#define SENTINEL_MODE_STATION 1
#define SENTINEL_MODE_ACCESS_POINT 2
#define SENTINEL_WIFI_MODE SENTINEL_MODE_STATION

// Wi-Fi name and passwords live in secrets.h (NOT in this file, so they are never
// published). Copy secrets.example.h to secrets.h and fill it in. 2.4 GHz only.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h not found - copy secrets.example.h to secrets.h and enter your Wi-Fi details"
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define AP_PASSWORD "change-me-123"
#endif

// Optional fixed IP in station mode (makes config/settings.yaml easier). 0 = use DHCP.
#define USE_STATIC_IP 0
#define STATIC_IP 192, 168, 43, 50
#define STATIC_GATEWAY 192, 168, 43, 1
#define STATIC_SUBNET 255, 255, 255, 0

// Access-point mode: the robot's own network (AP_PASSWORD in secrets.h, at least 8 characters)
#define AP_SSID "SentinelRobot"
#define AP_CHANNEL 6                // robot IP in AP mode is 192.168.4.1

#define ROBOT_HOSTNAME "sentinel-robot"   // also advertised as sentinel-robot.local (mDNS)
#define TCP_PORT 3333                     // must match robot.port in config/settings.yaml

// ---------------------------------------------------------------- Safety
#define WATCHDOG_MS 500        // no valid message for this long -> motors stop
#define MIN_MOVE_MS 50         // shortest movement command accepted
#define MAX_MOVE_MS 2000       // longest movement command accepted
#define MAX_SPEED_PCT 60       // speeds above this are clamped

#define STATUS_INTERVAL_MS 1000        // status message to the laptop every second
#define EMERGENCY_BUZZ_MAX_MS 60000     // emergency buzzer stops after 60 s (LED stays on)
#define FOLLOW_MIN_CONFIDENCE 0.5       // follow requests below this target confidence are refused

// Obstacles
#define OBSTACLE_HOLD_MS 1500  // a laptop OBSTACLE report blocks forward motion this long (re-sent while seen)
#define OBSTACLE_SENSOR_ENABLED 0   // 1 = HC-SR04 ultrasonic sensor fitted (optional extra hardware)
#define OBSTACLE_STOP_CM 25    // sensor: stop forward motion when something is closer than this
#define OBSTACLE_CLEAR_CM 35   // sensor: allow forward again when it is farther than this
#define OBSTACLE_PING_MS 60    // sensor: one measurement every 60 ms

// PHASE 2: keep 0 so the motor driver pins are never driven, even if a
// movement command arrives. Set to 1 only in Phase 3, with the wheels lifted.
#define MOTORS_ENABLED 0

// ---------------------------------------------------------------- Pins
// Proposed GPIOs for an ESP32 DevKit V1 (DOIT, 30 pins). They avoid the boot
// strapping pins (0, 2*, 5, 12, 15) and pins that pulse at boot (1, 3, 14).
// *GPIO2 is the on-board blue LED, used only as a status light.
// Phase 3 will verify these against your actual wiring before motors are used.
#define PIN_ENA 25             // L298N ENA (left motor speed, PWM)  - remove the ENA jumper
#define PIN_IN1 26             // L298N IN1 (left motor direction)
#define PIN_IN2 27             // L298N IN2
#define PIN_ENB 13             // L298N ENB (right motor speed, PWM) - remove the ENB jumper
#define PIN_IN3 33             // L298N IN3 (right motor direction)
#define PIN_IN4 32             // L298N IN4
#define PIN_BUZZER 23          // buzzer (through a transistor if it draws > 10 mA)
#define PIN_RED_LED 22         // red LED + series resistor (e.g. 220-330 ohm) to GND
#define PIN_STATUS_LED 2       // on-board LED: link status, fast blink = EMERGENCY
#define PIN_US_TRIG 18         // HC-SR04 TRIG (only if OBSTACLE_SENSOR_ENABLED)
#define PIN_US_ECHO 19         // HC-SR04 ECHO through a 5 V -> 3.3 V voltage divider!

// Motor calibration (Phase 3, see docs/HARDWARE.md section 6)
#define MIN_DUTY_PCT 0         // PWM % at which the wheels just start turning (dead zone)
#define LEFT_TRIM_PCT 100      // reduce the faster wheel (e.g. 94) so the robot drives straight
#define RIGHT_TRIM_PCT 100
#define LEFT_MOTOR_INVERT 0    // set to 1 if the left wheel turns the wrong way (Phase 3)
#define RIGHT_MOTOR_INVERT 0
#define BUZZER_PASSIVE 0       // 0 = active buzzer (on/off), 1 = passive buzzer (needs a tone)
#define BUZZER_TONE_HZ 2000
#define MOTOR_PWM_HZ 1000
#define MOTOR_PWM_BITS 8

// ---------------------------------------------------------------- Debug
#define SERIAL_BAUD 115200
#define LOG_COMMANDS 1         // print every command and reply on the Serial Monitor

#endif  // SENTINEL_CONFIG_H
