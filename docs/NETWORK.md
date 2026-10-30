# Network arrangement (phone camera + laptop + ESP32)

The laptop must reach **two** devices at the same time:

```text
 phone (IP Webcam) --video--> laptop --commands--> ESP32 robot
```

A laptop has one Wi-Fi card and joins one network at a time. If it joins the
ESP32's own network it may lose the phone camera, and the other way round.

## Option A — phone hotspot (recommended, works anywhere)

```text
           Phone: hotspot ON + IP Webcam running (camera URL http://<phone-IP>:8080/video)
            /                     \
   laptop joins hotspot     ESP32 joins hotspot (station mode)
```

* Everything is on one network, no router needed, nothing else for the robot to carry.
* `config.h`: `SENTINEL_WIFI_MODE SENTINEL_MODE_STATION`, `WIFI_SSID` / `WIFI_PASSWORD` = the hotspot.
  Set the hotspot to **2.4 GHz** (or "compatibility" mode): the ESP32 cannot use 5 GHz.
* The phone's own IP on its hotspot is often `192.168.43.1` (varies by phone). In IP Webcam the
  address it shows is the one to use.
* The robot's IP is printed on the Serial Monitor at start-up. Optionally fix it with `USE_STATIC_IP`.
* Some phones block traffic between hotspot clients. The network check below detects this.

## Option B — home / lab router (station mode)

Same as A but phone, laptop and ESP32 all join the router's 2.4 GHz network.
College or guest networks often use **client isolation** (devices cannot see each
other) — then use option A.

## Option C — ESP32 access point

* `config.h`: `SENTINEL_WIFI_MODE SENTINEL_MODE_ACCESS_POINT`; the robot creates
  `SentinelRobot` (password in `config.h`), robot IP `192.168.4.1`.
* Laptop **and phone** must both join `SentinelRobot`, otherwise the laptop loses the camera.
* Concerns (must be measured before relying on it): the ESP32 soft-AP has limited
  bandwidth for the video stream, and whether its clients can reach each other
  has to be tested with your firmware/phone combination.
* Useful as a fallback when no hotspot or router is available.

## How to check your arrangement

1. Start IP Webcam on the phone; start the robot; connect the laptop.
2. Put the camera URL in `camera.source` and the robot IP in `robot.host`
   (`config/settings.yaml`).
3. Run:

   ```powershell
   python -m robot.network_check --seconds 15
   ```

   It reads camera frames **and** pings the robot at the same time and prints
   `camera and robot both reachable` or which one failed. The result is saved in
   `evaluation/communication/`.
4. Then run the full communication test (`python -m robot.comm_test`).

Write the arrangement you finally used, the IP addresses and the measured
results into `docs/TESTING.md` (Phase 2 table).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Serial Monitor: `still not connected, retrying` | wrong SSID/password, 5 GHz network | check `config.h`; switch hotspot to 2.4 GHz |
| `cannot connect ... timed out` from the laptop | laptop on a different network, wrong IP, client isolation | compare the IP on the Serial Monitor; use option A |
| `Connection refused` | IP reachable but firmware not running / wrong port | check `TCP_PORT` = `robot.port` |
| Camera works only when laptop is on home Wi-Fi | robot on its own AP (option C) | use option A or B |
| Robot keeps reconnecting, many timeouts | weak signal | move closer; the robot stops itself whenever the link drops |
| Windows asks "allow access" for Python | Windows firewall | outgoing connections need no rule; if prompted, allow on *private* networks |
