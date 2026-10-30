"""Compile and run the ESP32 firmware core unit tests on the PC (needs g++), and check that
firmware and laptop settings agree and that no Wi-Fi password is kept in published source."""
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from src.config import DEFAULT_CONFIG_PATH, validate

FW = Path(__file__).resolve().parent.parent / "firmware"


@pytest.mark.skipif(shutil.which("g++") is None or sys.platform.startswith("win"),
                    reason="g++ not available")
def test_firmware_core_unit_tests(tmp_path):
    exe = tmp_path / "test_robot_core"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{FW / 'sentinel_robot'}",
                    str(FW / "host_test" / "test_robot_core.cpp"), str(FW / "sentinel_robot" / "robot_core.cpp"),
                    "-o", str(exe)], check=True, capture_output=True, text=True)
    r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stdout
    assert "0 failed" in r.stdout


def test_firmware_and_laptop_settings_agree():
    """The laptop heartbeat must be faster than the robot watchdog, ports must match."""
    cfg = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    h = (FW / "sentinel_robot" / "config.h").read_text()

    def define(name):
        for line in h.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[0] == "#define" and parts[1] == name:
                return parts[2]
        raise AssertionError(name)
    assert int(define("TCP_PORT")) == cfg["robot"]["port"]
    assert cfg["robot"]["heartbeat_s"] * 1000 * 2 < int(define("WATCHDOG_MS"))
    assert int(define("MIN_MOVE_MS")) <= cfg["robot"]["manual_hold_ms"] <= int(define("MAX_MOVE_MS"))
    # the laptop treats the link as faulty when status is older than status_stale_s
    assert cfg["robot"]["status_stale_s"] * 1000 >= 2 * int(define("STATUS_INTERVAL_MS"))
    assert float(define("FOLLOW_MIN_CONFIDENCE")) <= cfg["robot"]["follow"]["min_detection_score"]
    assert define("MOTORS_ENABLED") == "0", "Phase 2 must ship with motor outputs disabled"
    # obstacle reports must be repeated well before the robot's hold expires
    assert cfg["robot"]["follow"]["obstacle"]["report_interval_s"] * 1000 * 2 <= int(define("OBSTACLE_HOLD_MS"))
    # following commands must outlive one frame gap but stay short
    assert int(define("MIN_MOVE_MS")) <= cfg["robot"]["follow"]["command_ms"] <= 1000
    assert cfg["robot"]["follow"]["max_speed_pct"] <= int(define("MAX_SPEED_PCT"))
    assert define("OBSTACLE_SENSOR_ENABLED") == "0", "sensor is optional extra hardware: off by default"


def test_robot_config_validation():
    cfg = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    cfg["robot"]["heartbeat_s"] = 0.6
    cfg["robot"]["max_speed_pct"] = 150
    cfg["robot"]["manual_hold_ms"] = 5000
    cfg["robot"]["status_stale_s"] = 0.5
    problems = validate(cfg)
    assert any("heartbeat_s" in p for p in problems)
    assert any("max_speed_pct" in p for p in problems)
    assert any("manual_hold_ms" in p for p in problems)
    assert any("status_stale_s" in p for p in problems)


def test_no_wifi_password_in_published_firmware_source():
    root = FW.parent
    assert "firmware/*/secrets.h" in (root / ".gitignore").read_text().split()
    placeholders = {"YOUR_WIFI_PASSWORD", "change-me-123", "YOUR_WIFI_NAME"}
    for f in list((FW / "sentinel_robot").glob("*")) + list((FW / "esp32_test").glob("*")):
        if f.name == "secrets.h" or f.suffix not in (".h", ".ino", ".cpp"):
            continue
        for name, value in re.findall(r'#define\s+(WIFI_PASSWORD|AP_PASSWORD|WIFI_SSID)\s+"([^"]*)"', f.read_text()):
            assert value in placeholders, f"{f.name}: {name} must stay a placeholder (put real values in secrets.h)"
    for sketch in ("sentinel_robot/sentinel_robot.ino", "esp32_test/esp32_test.ino"):
        for line in (FW / sketch).read_text().splitlines():
            if "Serial.print" in line:
                assert "PASSWORD" not in line, "never print Wi-Fi passwords on the serial monitor"


def test_communication_test_firmware_has_no_motor_code():
    """Stage 1: the test sketch must not configure or write any motor pin, and no setting can enable one."""
    ino = (FW / "esp32_test" / "esp32_test.ino").read_text()
    cfg = (FW / "esp32_test" / "config.h").read_text()
    for word in ("PIN_ENA", "PIN_ENB", "PIN_IN1", "PIN_IN2", "PIN_IN3", "PIN_IN4", "MOTORS_ENABLED", "driveOne"):
        assert word not in ino and word not in cfg, word
    assert "c.motorsEnabled = false" in ino
    # setMotors only records the request
    body = ino.split("void setMotors(int left, int right) override {")[1].split("}")[0]
    assert "digitalWrite" not in body and "ledc" not in body


def test_communication_test_firmware_uses_the_same_core():
    for f in ("robot_core.h", "robot_core.cpp", "secrets.example.h"):
        assert (FW / "esp32_test" / f).read_bytes() == (FW / "sentinel_robot" / f).read_bytes(), (
            f"{f} differs - run: python tools/sync_firmware_core.py")
