#!/usr/bin/env python3
"""Print the uniquely connected OpenRB-150 USB CDC port.

PlatformIO's automatic selection can choose a different SAMD CDC device when
more than one is connected. The OpenRB-150 board definition identifies the
board as USB VID:PID 2F5D:2202, which is stable across its changing modem name.
"""

from __future__ import annotations

import json
import subprocess
import sys


OPENRB_HWID = "VID:PID=2F5D:2202"


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <platformio-executable>", file=sys.stderr)
        return 2

    result = subprocess.run(
        [sys.argv[1], "device", "list", "--json-output"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        print(result.stderr.strip() or "PlatformIO device discovery failed.", file=sys.stderr)
        return result.returncode

    try:
        devices = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        print(f"Could not parse PlatformIO device list: {error}", file=sys.stderr)
        return 1

    ports = [
        device["port"]
        for device in devices
        if OPENRB_HWID in device.get("hwid", "").upper()
    ]
    if len(ports) == 1:
        print(ports[0])
        return 0

    if not ports:
        print(
            f"No OpenRB-150 found (expected USB {OPENRB_HWID}). "
            "Connect the board and retry.",
            file=sys.stderr,
        )
        return 1

    print(
        f"Found multiple OpenRB-150 devices ({', '.join(ports)}). "
        "Disconnect all but the target board and retry.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())