// =====================================================================
//  SentinelAI - ESP32 COMMUNICATION TEST - settings.
//  Edit this file (and secrets.h), then upload esp32_test.ino.
//
//  This test firmware contains NO motor code at all: FORWARD / BACKWARD /
//  LEFT / RIGHT are only recorded and acknowledged ("simulated movement").
//  No setting in this file can make a motor move.
// =====================================================================
#ifndef SENTINEL_TEST_CONFIG_H
#define SENTINEL_TEST_CONFIG_H

// ---------------------------------------------------------------- Wi-Fi
// STATION (recommended): the ESP32 joins the same Wi-Fi as the phone and the laptop
// (home router, or the phone's hotspot). See docs/STAGE1_COMM_TEST.md section 2.
// ACCESS_POINT: the ESP32 makes its own Wi-Fi "SentinelRobot" (last resort - the laptop
// may then lose the phone camera).
#define SENTINEL_MODE_STATION 1
#define SENTINEL_MODE_ACCESS_POINT 2
#define SENTINEL_WIFI_MODE SENTINEL_MODE_STATION

// Wi-Fi name and password: in secrets.h (copy secrets.example.h -> secrets.h). Never published.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h not found - copy secrets.example.h to secrets.h and enter your Wi-Fi details"
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define AP_PASSWORD "change-me-123"
#endif

// Optional fixed IP (station mode). 0 = the router/hotspot gives an address (DHCP).
#define USE_STATIC_IP 0
#define STATIC_IP 192, 168, 1, 50
#define STATIC_GATEWAY 192, 168, 1, 1
#define STATIC_SUBNET 255, 255, 255, 0

#define AP_SSID "SentinelRobot"     // access-point mode only; robot IP is then 192.168.4.1
#define AP_CHANNEL 6
#define ROBOT_HOSTNAME "sentinel-robot"
#define TCP_PORT 3333               // must equal robot.port in config/settings.yaml

// ------------------------------------------------------- LED and buzzer
// Set to 0 if the part is not wired yet: commands are still accepted and reported,
// the pin is simply left alone, and "led_installed"/"buzzer_installed" say false.
#define LED_INSTALLED 1
#define PIN_RED_LED 22              // external red LED + 330 ohm resistor to GND (docs/STAGE1_COMM_TEST.md §9)
#define BUZZER_INSTALLED 1
#define PIN_BUZZER 23               // drives an NPN transistor base through 1 kohm (NOT the buzzer directly)
#define BUZZER_PASSIVE 0            // 0 = active buzzer (beeps on plain DC), 1 = passive (needs a tone)
#define BUZZER_TONE_HZ 2000

// On-board blue LED used as a link indicator. On most "ESP32 DevKit V1 (DOIT)" boards it
// is GPIO2, but some clones have none or use another pin. -1 = do not use it.
#define PIN_STATUS_LED 2

// At power-up: blink the red LED 3 times and beep once, so you can check the wiring
// without Wi-Fi. 0 = off.
#define STARTUP_SELFTEST 1

// ---------------------------------------------------------------- Safety
#define WATCHDOG_MS 500             // no valid message for this long -> simulated movement cleared
#define SIM_MOVE_MS 2000            // a simulated movement request ends by itself after 2 s
#define STATUS_INTERVAL_MS 1000     // status message to the laptop every second
#define BUZZER_MAX_MS 60000         // continuous / emergency buzzer stops after 60 s

// ---------------------------------------------------------------- Debug
#define SERIAL_BAUD 115200
#define LOG_COMMANDS 1              // print every received command / event in plain words
#define LOG_RAW_JSON 0              // 1 = also print the raw JSON lines ([RX-JSON] / [TX]) for protocol debugging

#endif  // SENTINEL_TEST_CONFIG_H
