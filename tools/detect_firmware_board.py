#!/usr/bin/env python3
"""Detect the connected firmware target board and print "<env> <port>".

Identifies which supported controller is on USB so the just recipes can pick
the matching PlatformIO environment automatically:

  * ROBOTIS OpenRB-150 (SAMD21): USB VID:PID 2F5D:2202 -> env "openrb150"
  * SparkFun MicroMod Teensy (IMXRT1062): PJRC VID 16C0 -> env "teensymm"

USB IDs are stable across the changing macOS modem device names. A Teensy
sitting in its HalfKay bootloader is a HID device with no serial port and is
NOT detectable here; pass the environment explicitly in that case
(e.g. `just firmware-upload teensymm`).
"""

from __future__ import annotations

import json
import subprocess
import sys

BOARDS = {
    "2F5D:2202": "openrb150",
    "16C0:": "teensymm",  # PJRC vendor ID; 0483 = Teensy USB Serial
}


def main() -> int:
    args = sys.argv[1:]
    field = "both"
    if args and args[0] in ("--env-only", "--port-only"):
        field = args.pop(0)[2:].split("-")[0]
    if len(args) != 1:
        print(
            f"usage: {sys.argv[0]} [--env-only|--port-only] <platformio-executable>",
            file=sys.stderr,
        )
        return 2

    result = subprocess.run(
        [args[0], "device", "list", "--json-output"],
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

    found: list[tuple[str, str]] = []
    for device in devices:
        hwid = device.get("hwid", "").upper()
        for marker, env in BOARDS.items():
            if f"VID:PID={marker}" in hwid:
                found.append((env, device["port"]))
                break

    if len(found) == 1:
        env, port = found[0]
        print({"env": env, "port": port, "both": f"{env} {port}"}[field])
        return 0

    if not found:
        print(
            "No supported board found (OpenRB-150 2F5D:2202 or Teensy 16C0:xxxx). "
            "Connect one board and retry, or pass the env explicitly. A Teensy "
            "stuck in its bootloader exposes no serial port; press its PROGRAM "
            "button and use `just firmware-upload teensymm`.",
            file=sys.stderr,
        )
        return 1

    listing = ", ".join(f"{env} on {port}" for env, port in found)
    print(
        f"Found multiple supported boards ({listing}). "
        "Disconnect all but the target board or pass the env explicitly.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
