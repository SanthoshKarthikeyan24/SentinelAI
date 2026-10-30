# Known limitations

Plain list of what SentinelAI does **not** do, or does only partly. Each point says how you would
notice it and what reduces the risk.

## Safety and scope

1. **Not a medical device, cannot call for help.** It shows alerts on the laptop and sounds the
   robot buzzer. Nobody is notified if no one is watching.
2. **"Fall confirmed by model" can be wrong.** Measured on staged falls only (below). Missed
   falls and false alarms both happen; a person must always check.
3. **No authentication on the robot link.** Anyone on the same Wi-Fi can send commands. Use a
   private hotspot with a strong password; never expose port 3333 to the internet.
4. **The robot cannot sense wheel motion.** "Motors stopped" in the ESP32 status means the
   firmware set the driver pins to 0, not that the wheels have stopped (a jammed or coasting
   wheel is not detected).

## AI accuracy (all numbers from evaluation/ and reports/experiments.md)

5. **One small dataset.** GMDCSA-24: 4 young adult actors, 159 short clips (≈21 minutes), 79
   staged falls, 3 rooms. Other datasets (UR Fall, Le2i, CAUCAFall, OmniFall …) could not be
   downloaded in the build environment. Results may not transfer to elderly people, real falls,
   other homes or a robot-height camera.
6. **Fall detection (nested leave-one-subject-out):** FALL_SUSPECTED F1 0.890 (precision 0.859,
   recall 0.924), FALL_CONFIRMED F1 0.819 (recall 0.772). False alarms per hour (44 suspected / 33
   confirmed) are extrapolated from only ≈16 minutes of non-fall video and are therefore very
   uncertain – expect them to be lower in normal daily life but higher during exercise, quick
   sitting down or lying on a bed.
7. **FALL_CONFIRMED needs the person to stay down** (1.5 s after "suspected"). Many clips end too
   soon after the fall, which lowers measured confirmed recall.
8. **Night scenes: too little data to judge lying.** The dataset's "Night" clips contain only 19
   lying frames (all recognised), but 37 sitting frames there were mistaken for LYING, so the
   night LYING F1 is only 0.51 (daylight ≈ 0.97; experiment H, reports/error_analysis.md §1).
   A sitting person wrongly seen as lying does not by itself raise a fall alarm (a fall also
   needs fast downward motion), but more night-time data is needed.
9. **Posture classes:** only STANDING, SITTING and LYING are learned; BENDING and KNEELING come
   from simple rules. "LYING" was learned from people lying on a bed.
10. **No keypoint, box or tracking ground truth** exists in the dataset, so pose accuracy, person
    detection precision and tracker ID switches are not measured – only detection coverage
    (92.7 % of frames).
11. **One person at a time** was labelled. Several people in view work technically (tracking,
    per-person states) but were not evaluated.
12. **Camera resolution:** results come from 1280×720 videos. Lower camera resolutions reduce
    detection quality (experiment A).

## Following and obstacles

13. **Distance is not measured.** Following uses the person's box height as a rough "nearer /
    farther" cue; tall/short people, sitting or partial views change it.
14. **Camera obstacle detection is a heuristic.** It only knows the detector's 80 object classes,
    cannot see low, thin, transparent or dark objects, and has no distance. The optional HC-SR04
    sensor is not installed or confirmed.
15. **Following assumes the phone is mounted on the robot facing forward.** It has only been
    tested against simulated robots and recorded video, never on a moving robot.

## Hardware (requires verification)

16. **Nothing has run on your ESP32 yet.** The firmware compiles for ESP32 DevKit V1 with Arduino
    cores 3.3.12 and 2.0.17 and its logic passes 264 PC checks, but Wi-Fi, PWM, pins, the L298N,
    motors, buzzer, LED and battery are untested.
17. **Power plan is not final** until you confirm the battery and motor details (docs/HARDWARE.md §4).
18. **Motors ship disabled** (`MOTORS_ENABLED 0`). Calibration (dead zone, trim, direction) is a
    manual procedure (docs/HARDWARE.md §6).

## Laptop and software

19. **Not yet run on your laptop in this version.** The previous version ran there (≈12 FPS,
    71 ms inference); webcam index 0 showed a static logo (probably a virtual camera). Run
    `tools\check_system.bat` to see which camera index is real.
20. **IP Webcam stream from Python is not yet verified** (its URL is not in settings.yaml yet).
21. **Optional alert clips** (off by default) store video of people in logs/clips; keep them private
    and delete them when not needed.
22. **Speed depends on the laptop.** In the build environment the pose model takes ≈54 ms per
    frame on 2 CPU cores; the dashboard shows live FPS and latency.
