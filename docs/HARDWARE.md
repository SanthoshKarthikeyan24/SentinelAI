# Hardware: pins, wiring, power, calibration

Status: **not yet verified on your robot.** Nothing below has been tested with real motors,
battery, buzzer or LED. The firmware ships with `MOTORS_ENABLED 0`, so the motor pins are never
driven until you finish section 6 with the wheels lifted off the ground.

## 1. Parts assumed

| Part | Assumed | You must check |
|---|---|---|
| Controller | ESP32 DevKit V1 (DOIT, 30 pins), 3.3 V logic | board name printed on it |
| Motor driver | L298N module (dual H-bridge, red board with heat sink) | jumpers ENA/ENB, 5V-EN |
| Motors | 2 small DC gear motors ("TT" type, 3–6 V) | **voltage and stall current on the label** |
| Buzzer | 5 V active buzzer (has its own oscillator) | active or passive? current? |
| LED | red 5 mm LED | – |
| Battery | **unknown** | see section 4 – nothing is wired to a battery until you confirm it |
| Optional | HC-SR04 ultrasonic sensor (5 V) | only if you add one |

## 2. Pin map (firmware/sentinel_robot/config.h)

| Signal | ESP32 GPIO | Goes to | Note |
|---|---|---|---|
| Left motor speed (PWM) | 25 | L298N **ENA** | remove the ENA jumper so the pin controls speed |
| Left motor direction | 26, 27 | L298N **IN1, IN2** | |
| Right motor speed (PWM) | 13 | L298N **ENB** | remove the ENB jumper |
| Right motor direction | 33, 32 | L298N **IN3, IN4** | |
| Buzzer | 23 | transistor base (section 3.3) | never drive a 5 V buzzer straight from the pin |
| Red LED | 22 | LED via resistor | section 3.2 |
| Status LED | 2 | on-board blue LED | already on the board |
| HC-SR04 TRIG | 18 | sensor TRIG | only with `OBSTACLE_SENSOR_ENABLED 1` |
| HC-SR04 ECHO | 19 | sensor ECHO **through a voltage divider** | 5 V signal → 3.3 V |

Why these pins: they avoid the ESP32 "strapping" pins (0, 2, 5, 12, 15) that decide how the chip
boots, and pins 1, 3, 14 that pulse during boot – a motor must never twitch at power-on. The
firmware sets every motor pin LOW before Wi-Fi starts.

## 3. Wiring tables

### 3.1 ESP32 → L298N (signal side)

| ESP32 | L298N |
|---|---|
| GPIO25 | ENA |
| GPIO26 | IN1 |
| GPIO27 | IN2 |
| GPIO13 | ENB |
| GPIO33 | IN3 |
| GPIO32 | IN4 |
| **GND** | **GND** (the same screw terminal as the battery minus) |

The L298N accepts 3.3 V signals as "high" (its input threshold is about 2.3 V), so no level
shifter is needed. **The common ground wire is required**: without it the signals have no
reference and the motors may run randomly.

Motors: left motor to OUT1/OUT2, right motor to OUT3/OUT4. If a wheel turns the wrong way, do not
rewire – set `LEFT_MOTOR_INVERT` / `RIGHT_MOTOR_INVERT` to 1 (section 6).

### 3.2 Red LED

GPIO22 → resistor → LED long leg (anode); LED short leg (cathode, flat side) → GND.

Resistor: current = (3.3 V − 2.0 V LED drop) / R. With **330 Ω** → about 4 mA (bright enough,
safe for the pin). With 220 Ω → about 6 mA. Do not go below 150 Ω.

### 3.3 Buzzer (through a transistor)

An ESP32 pin should supply at most about 12 mA; a 5 V buzzer typically needs 20–30 mA. So the pin
only switches a small NPN transistor (2N2222, BC547 or S8050; pin order differs between
parts – see the table in docs/STAGE1_COMM_TEST.md §9.3):

| From | To |
|---|---|
| GPIO23 | **1 kΩ** resistor → transistor **base** |
| transistor **emitter** | GND |
| transistor **collector** | buzzer **minus (−)** |
| buzzer **plus (+)** | 5 V (or 3.3 V for a 3 V buzzer) |
| 1N4148 diode (optional for an active buzzer, **required for a passive/magnetic one**) | across the buzzer, stripe towards + |

Active buzzer: keep `BUZZER_PASSIVE 0`. Passive buzzer (no sound with plain DC): set
`BUZZER_PASSIVE 1`; the firmware then plays a `BUZZER_TONE_HZ` tone.

### 3.4 Optional HC-SR04

VCC → 5 V, GND → GND, TRIG → GPIO18, ECHO → **1 kΩ → GPIO19, and GPIO19 → 2 kΩ → GND**
(divider: 5 V × 2/3 = 3.3 V). Never connect ECHO directly: 5 V can damage the ESP32 pin.
It is an optional extra: the camera-based obstacle check is a heuristic and is reported
separately (`obstacle_source` = `camera` / `sensor` / `both`).

## 4. Power plan – battery details received, BMS rating still needed

What you told me: **3 × 18650 Li-ion cells, 3.7 V nominal each, with a BMS**; motors: either
**two 3–6 V gear motors** or **two 12 V motors**.

Facts that decide the plan:
* One Li-ion cell: 3.0 V (empty) – 3.7 V (nominal) – 4.2 V (full).
  2 in series ("2S") = 6.0 – 7.4 – 8.4 V. 3 in series ("3S") = 9.0 – 11.1 – 12.6 V.
* A BMS is made for a fixed number of cells in series – it is printed on the board, e.g. "2S 10A"
  or "3S 20A". Cells must be connected exactly as that BMS expects, and its current rating must
  be above the motors' combined stall current.
* The L298N loses about 1.5–2.5 V, so the motors get roughly the pack voltage minus 2 V.

| Option | Pack | Motors | Motor voltage after L298N | Verdict |
|---|---|---|---|---|
| **1 (recommended)** | 2S (7.4 V, 8.4 V full) + 2S BMS | 3–6 V gear motors | ≈ 5.4–6.4 V at full PWM; firmware caps speed (`MAX_SPEED_PCT`) | lighter, lower current, matches small chassis; 3rd cell spare |
| 2 | 3S (11.1 V, 12.6 V full) + 3S BMS | 12 V motors | ≈ 9–10.5 V | works; heavier/faster motors; needs a separate 5 V buck module for the ESP32 |
| ✗ | 3S | 3–6 V motors | ≈ 10 V – motors overheat unless PWM is strictly limited | avoid |
| ✗ | 1S (3.7 V) | any | ≈ 2 V | too low for the L298N |

**Stage 1 (now) uses no battery at all:** the ESP32 runs from USB (laptop for upload, then a
power bank or phone charger). Never connect a Li-ion cell directly to the ESP32 3V3 pin
(4.2 V > 3.6 V maximum).

Before the motor stage, please tell me:
1. the **S rating and current printed on your BMS** (e.g. "2S 10A"),
2. whether the cells are in a holder / spot-welded pack, and how you charge them (a proper
   Li-ion charger or a BMS with a charging input – never improvise charging),
3. which motors you choose (my default recommendation: option 1).

Then I will give the final battery → switch → fuse → L298N → ESP32 wiring. Always: a power
switch in the battery + line, a fuse sized for the motors, all grounds joined, and nothing on
the ESP32 3V3 pin except 3.3 V.

## 5. Grounding and noise

* Run the two thick motor-current wires (battery → L298N) separately from the thin ESP32 signal
  wires; join grounds at the L298N GND terminal ("star" point).
* A 100 nF ceramic capacitor across each motor's terminals reduces electrical noise that can
  reset the ESP32 or disturb Wi-Fi.
* If the ESP32 resets when the motors start, the supply voltage is dipping: stop and check the
  battery and wiring before continuing.

## 6. Calibration procedure (wheels lifted!)

Put the robot on a box so the wheels spin freely. Keep a hand near the power switch / USB cable.

1. Flash with `MOTORS_ENABLED 1`, all other settings default. Open the Serial Monitor (115200).
2. On the laptop: `python -m robot.robot_console --host <robot IP>`.
3. **Direction:** `motor 25 25 500` – both wheels must turn "forward". If one turns backwards,
   set its `*_MOTOR_INVERT 1`, re-flash, repeat.
4. **Dead zone:** `motor 10 10 800`, then 15, 20, 25 … note the smallest value where both wheels
   turn reliably. Put it in `MIN_DUTY_PCT` (the firmware then maps 1–100 % onto that range).
5. **Straight line (on the floor, low speed, open space):** `motor 25 25 1500`. If it drifts to
   one side, reduce the faster wheel's `*_TRIM_PCT` (e.g. 95) and repeat.
6. **Stop checks:** `motor 25 25 2000` then `stop` – wheels stop at once. Run
   `python -m robot.comm_test --allow-motion` (wheels lifted): watchdog and disconnect steps must
   stop the wheels within about half a second.
7. **Speed limit:** keep `MAX_SPEED_PCT` (firmware) and `robot.max_speed_pct` (laptop) low
   (≤ 40) until following has been tested in an empty room.
8. Write your values into `config.h` and note them in reports/test_plan.md (H-tests).

## 7. Never

* Never connect 5 V to an ESP32 GPIO pin.
* Never let motor current flow through the ESP32 or its USB cable.
* Never test following or motion near stairs, people who could trip, or pets.
* Never leave a Li-ion/LiPo battery charging unattended.
