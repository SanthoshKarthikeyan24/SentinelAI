# ESP32 firmware: install, configure, upload (Phase 2)

> **Start with the stage-1 communication test** (`firmware/esp32_test/`, no motor code):
> docs/STAGE1_COMM_TEST.md. This page describes the full robot firmware `firmware/sentinel_robot/`.

Phase 2 needs **only the ESP32 board and a USB cable**. Do not connect the
L298N, motors, battery, buzzer or LED yet — that is Phase 3. The firmware ships
with `MOTORS_ENABLED 0`, so even a movement command never drives the motor pins.

## 1. Arduino IDE and ESP32 support (once)

1. Install **Arduino IDE 2.x** from arduino.cc.
2. *File → Preferences → Additional boards manager URLs*, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. *Tools → Board → Boards Manager*, search **esp32**, install
   **"esp32 by Espressif Systems"**. Tested here: **3.3.12** (recommended) and **2.0.17**.
4. USB driver: DevKit V1 boards use a **CP2102** or **CH340** USB chip. If no new
   COM port appears when you plug the board in, install the matching driver
   (Silicon Labs CP210x or WCH CH340) and use a data-capable USB cable.

## 2. Configure

Open `firmware/sentinel_robot/sentinel_robot.ino` in the Arduino IDE (the other
files in that folder open as tabs).

**Wi-Fi details:** copy `secrets.example.h` to **`secrets.h`** in the same folder and enter
your Wi-Fi name and password there. `secrets.h` is in `.gitignore`, so it is never published;
`config.h` contains only placeholders. Without `secrets.h` the sketch still compiles but shows a
warning and cannot join your Wi-Fi.

Then check **`config.h`**:

| Setting | Phase 2 value |
|---|---|
| `SENTINEL_WIFI_MODE` | `SENTINEL_MODE_STATION` (see `docs/NETWORK.md`) |
| `WIFI_SSID`, `WIFI_PASSWORD` (in `secrets.h`) | your phone hotspot / router (**2.4 GHz**) |
| `USE_STATIC_IP` | optional; otherwise read the IP from the Serial Monitor |
| `TCP_PORT` | `3333` (must equal `robot.port` in `config/settings.yaml`) |
| `MOTORS_ENABLED` | **`0`** — do not change in Phase 2 |

The firmware never prints the password on the Serial Monitor.

## 3. Upload

1. *Tools → Board → esp32 →* **DOIT ESP32 DEVKIT V1** (or "ESP32 Dev Module"; both compile).
2. *Tools → Port →* the new COM port.
3. Click **Upload**. If it stops at `Connecting....`, hold the **BOOT** button on
   the board until the upload starts.
4. Open *Tools → Serial Monitor*, set **115200 baud**, press the board's **EN** (reset) button.

Expected output:

```text
SentinelAI robot controller, protocol v3, motors DISABLED (Phase 2 safe mode)
[wifi] connecting to 'MyHotspot' ...
[wifi] connected. Robot IP 192.168.43.50  port 3333  (put this IP in config/settings.yaml)
```

On-board blue LED: **1 Hz blink** = no Wi-Fi, **short flash every 2 s** = waiting
for the laptop, **solid** = laptop connected, **fast 5 Hz blink** = EMERGENCY.

## 4. Laptop settings

In `config/settings.yaml`:

```yaml
robot:
  enabled: true          # after the communication test passes
  host: 192.168.43.50    # the IP from the Serial Monitor
  port: 3333
```

## 5. Phase 2 tests with the real board

```powershell
python -m robot.network_check --seconds 15          # camera + robot reachable together?
python -m robot.comm_test                           # automated test plan (15 steps)
python -m robot.comm_test --manual-network-test     # + switch laptop Wi-Fi off/on when asked
python -m robot.robot_console                       # try commands by hand
python run_app.py --robot 192.168.43.50             # full system: a confirmed fall -> robot EMERGENCY
```

While the tests run, watch the Serial Monitor: every command (`<-`) and reply
(`->`) is printed (the 1-second status messages are not), plus `[robot]` lines for events such
as `watchdog_stop`. Record the results in the hardware table of `reports/test_plan.md`.

## 6. Pins (motors, buzzer and LED are wired in Phase 3 – full wiring: docs/HARDWARE.md)

| Function | GPIO | Notes |
|---|---|---|
| L298N ENA (left speed, PWM) | 25 | remove the ENA jumper |
| L298N IN1 / IN2 (left direction) | 26 / 27 | |
| L298N ENB (right speed, PWM) | 13 | remove the ENB jumper |
| L298N IN3 / IN4 (right direction) | 33 / 32 | |
| Buzzer | 23 | via a transistor if it needs more than ~10 mA |
| Red LED | 22 | with a 220–330 Ω series resistor |
| Status LED | 2 | on-board |
| HC-SR04 TRIG / ECHO (optional) | 18 / 19 | ECHO through a voltage divider, see §7 |

These avoid the ESP32 strapping pins (0, 2, 5, 12, 15) for outputs that matter at
boot, and GPIO 14 (it pulses at boot). They will be checked against your actual
board and wiring in Phase 3 before any motor is connected.

## 7. Optional ultrasonic obstacle sensor (HC-SR04)

Not part of the confirmed hardware list - add it only if you have one. The
camera-based obstacle check works without it.

| HC-SR04 pin | Connect to |
|---|---|
| VCC | 5 V (from the regulator / USB 5 V, **not** 3.3 V) |
| GND | GND (common with the ESP32) |
| TRIG | GPIO 18 (3.3 V is enough to trigger) |
| ECHO | GPIO 19 **through a voltage divider**: ECHO – 1 kΩ – GPIO 19 – 2 kΩ – GND (5 V → 3.3 V) |

Then set `OBSTACLE_SENSOR_ENABLED 1` in `config.h` and upload. The Serial
Monitor prints `Ultrasonic obstacle sensor ON`, and `STATUS` reports the
distance. The robot stops forward motion by itself below `OBSTACLE_STOP_CM`
(25 cm) and allows it again above `OBSTACLE_CLEAR_CM` (35 cm). Test: hold your
hand 15 cm in front of the sensor and run `status` in the robot console.

## 8. Firmware structure

| File | Purpose |
|---|---|
| `sentinel_robot.ino` | Wi-Fi, TCP server, GPIO/PWM, status LED (hardware glue only) |
| `robot_core.h/.cpp` | protocol parsing + all safety logic, plain C++ (also compiled on the PC for tests) |
| `config.h` | every setting |
| `../host_test/test_robot_core.cpp` | 204 unit checks of the core, run on a PC with g++ |
| `../host_test/core_replay.cpp` | replays scripts through the core; used to prove the Python simulator is identical |
| `../host_test/robot_sim.cpp` | the core behind a TCP server = robot simulator (Linux/macOS) |

On Windows without g++, use the Python simulator: `python -m robot.sim_robot`.
