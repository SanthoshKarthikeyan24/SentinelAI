# Communication test results

| Folder / file | What it is |
|---|---|
| `simulators/firmware_core_sim/` | `robot.comm_test` against the real firmware core (`robot_core.cpp`) compiled for a PC, over **loopback** (no Wi-Fi) |
| `simulators/python_sim/` | the same test against `robot/sim_robot.py` |
| `comm_test_*.json` (this folder) | your runs against the **real ESP32** over Wi-Fi - created by `python -m robot.comm_test` |
| `network_check_*.json` | your runs of `python -m robot.network_check` (camera + robot together) |

The simulator latencies (well below 1 ms) only show that the software adds
almost no delay. Real Wi-Fi latency must be measured with the ESP32
(typically a few ms to a few tens of ms) and recorded in docs/TESTING.md.
