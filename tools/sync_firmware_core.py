"""Copy the shared protocol/safety core from firmware/sentinel_robot/ into firmware/esp32_test/.

The Arduino IDE only compiles files inside a sketch's own folder, so the communication-test
sketch keeps a copy of robot_core.h / robot_core.cpp. Edit the files in firmware/sentinel_robot/
only, then run:
    python tools/sync_firmware_core.py          # copy
    python tools/sync_firmware_core.py --check  # exit 1 if the copies differ (used by the tests)
"""
import filecmp
import shutil
import sys
from pathlib import Path

FW = Path(__file__).resolve().parent.parent / "firmware"
FILES = ("robot_core.h", "robot_core.cpp", "secrets.example.h")


def main() -> int:
    check = "--check" in sys.argv
    differ = [f for f in FILES if not filecmp.cmp(FW / "sentinel_robot" / f, FW / "esp32_test" / f, shallow=False)]
    if check:
        print("identical" if not differ else "DIFFERENT: " + ", ".join(differ))
        return 1 if differ else 0
    for f in differ:
        shutil.copy2(FW / "sentinel_robot" / f, FW / "esp32_test" / f)
        print("copied", f)
    print("esp32_test core is up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
