#!/usr/bin/env python3
"""Phase 1 hardware-in-loop (HIL) smoke test for the OpenRB-150 firmware.

Drives the USB API v0 over the board's USB CDC serial port and checks the
boot-safety invariants that can be observed without commanding any motion:

  * USB enumerates and the firmware answers HELLO with the expected protocol
    and firmware version.
  * GET_STATUS reports a safe boot state: DXL power OFF and no watchdog misses.
  * HEARTBEAT uptime advances (the RTOS scheduler is running).
  * GET_CAPABILITIES decodes (feature bitmap / device identity).

Deeper DXL / I2C / EEPROM / CRSF checks need either physical interaction or
Phase 2 API commands; those are covered by the companion checklist in
``docs/hil_smoke_phase1.md`` and printed as guided manual steps at the end.

Usage:
    # Run from the companion uv environment -- it is the only env that provides
    # both pyserial and the hexapod_protocol package. Bare system Python fails
    # because pyserial is not installed, and there is no root uv project.
    cd companion && uv run python ../tools/hil_smoke.py --port /dev/tty.usbmodemXXXX

    # Without uv: install pyserial into the active interpreter first
    # (pip install pyserial), then put hexapod_protocol on the path:
    PYTHONPATH=protocol/python python tools/hil_smoke.py --port <PORT>

Exit code 0 = all automated checks passed, 1 = a check failed, 2 = setup error.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:  # pragma: no cover - environment guard
    print(
        "ERROR: pyserial is required. Run from the companion uv env "
        "(cd companion && uv run python ../tools/hil_smoke.py --port <PORT>), "
        "or 'pip install pyserial' into the active interpreter.",
        file=sys.stderr,
    )
    sys.exit(2)

from hexapod_protocol import api
from hexapod_protocol.framing import VERSION_MAJOR, VERSION_MINOR

# Expected firmware identity (keep in sync with src/app/tasks.cpp initDeviceInfo).
EXPECTED_DEVICE_NAME = "OpenRB150-Hex"

READ_TIMEOUT_S = 1.0
INTER_CMD_PAUSE_S = 0.05


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


class SmokeRunner:
    def __init__(self, port: serial.Serial) -> None:
        self._port = port
        self._seq = 0
        self.checks: list[Check] = []

    def _record(self, name: str, passed: bool, detail: str) -> bool:
        self.checks.append(Check(name, passed, detail))
        mark = "PASS" if passed else "FAIL"
        print(f"  [{mark}] {name}: {detail}")
        return passed

    def _exchange(
        self, msg_id: int, payload: bytes = b""
    ) -> tuple[object, bytes] | None:
        """Send a command and read one framed response. Returns (header, payload)."""
        self._seq = (self._seq + 1) & 0xFFFF
        frame = api.build_command(msg_id, seq=self._seq, payload=payload)
        self._port.reset_input_buffer()
        self._port.write(frame)
        self._port.flush()

        # Read until we have a complete delimited frame (0x00 ... 0x00) or time out.
        deadline = time.monotonic() + READ_TIMEOUT_S
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._port.read(64)
            if chunk:
                buf.extend(chunk)
                # A complete frame has at least two delimiters with data between.
                if buf.count(0x00) >= 2:
                    start = buf.find(0x00)
                    end = buf.find(0x00, start + 1)
                    if end > start:
                        wire = bytes(buf[start : end + 1])
                        try:
                            return api.parse_response(wire)
                        except ValueError:
                            buf = bytearray(buf[end:])
            else:
                time.sleep(0.01)
        return None

    def check_hello(self) -> None:
        result = self._exchange(api.MSG_HELLO)
        if result is None:
            self._record("USB HELLO", False, "no response (timeout)")
            return
        header, payload = result
        try:
            info = api.parse_hello(payload)
        except Exception as exc:  # noqa: BLE001
            self._record("USB HELLO", False, f"undecodable payload: {exc}")
            return

        proto_ok = (info.proto_major, info.proto_minor) == (
            VERSION_MAJOR,
            VERSION_MINOR,
        )
        self._record(
            "USB HELLO protocol",
            proto_ok,
            f"v{info.proto_major}.{info.proto_minor} "
            f"(expected v{VERSION_MAJOR}.{VERSION_MINOR})",
        )
        self._record(
            "USB HELLO device identity",
            info.device_name == EXPECTED_DEVICE_NAME,
            f"name='{info.device_name}', fw {info.fw_major}.{info.fw_minor}.{info.fw_patch}",
        )

    def check_status(self) -> None:
        result = self._exchange(api.MSG_GET_STATUS)
        if result is None:
            self._record("USB GET_STATUS", False, "no response (timeout)")
            return
        _, payload = result
        try:
            st = api.parse_status(payload)
        except Exception as exc:  # noqa: BLE001
            self._record("USB GET_STATUS", False, f"undecodable payload: {exc}")
            return

        # Boot-safety invariant: DXL power must be OFF until explicitly commanded.
        self._record(
            "DXL power OFF at boot",
            st.dxl_power is False,
            f"dxl_power={st.dxl_power}, control={st.dxl_power_control}",
        )
        # No task should have missed its watchdog deadline at idle.
        self._record(
            "No watchdog misses",
            st.watchdog_missed == 0,
            f"watchdog_missed=0x{st.watchdog_missed:08X}",
        )
        # Battery telemetry should be plausible if a pack is connected; just report.
        self._record(
            "Status telemetry decodes",
            True,
            f"state={st.state}, battery={st.battery_mv} mV, uptime={st.uptime_ms} ms",
        )

    def check_heartbeat_advances(self) -> None:
        first = self._exchange(api.MSG_HEARTBEAT)
        if first is None:
            self._record("Heartbeat uptime advances", False, "no first response")
            return
        time.sleep(0.2)
        second = self._exchange(api.MSG_HEARTBEAT)
        if second is None:
            self._record("Heartbeat uptime advances", False, "no second response")
            return
        t0, _ = api.parse_heartbeat(first[1])
        t1, _ = api.parse_heartbeat(second[1])
        self._record(
            "Heartbeat uptime advances",
            t1 > t0,
            f"{t0} ms -> {t1} ms (scheduler running)",
        )

    def check_capabilities(self) -> None:
        result = self._exchange(api.MSG_GET_CAPABILITIES)
        if result is None:
            self._record("USB GET_CAPABILITIES", False, "no response (timeout)")
            return
        _, payload = result
        try:
            caps = api.parse_capabilities(payload)
        except Exception as exc:  # noqa: BLE001
            self._record("USB GET_CAPABILITIES", False, f"undecodable payload: {exc}")
            return
        self._record(
            "USB GET_CAPABILITIES",
            True,
            f"feature_bits=0x{caps.feature_bits:08X}, name='{caps.device_name}'",
        )

    def run(self) -> bool:
        print("Automated USB API checks:")
        self.check_hello()
        self.check_status()
        self.check_heartbeat_advances()
        self.check_capabilities()
        return all(c.passed for c in self.checks)


MANUAL_CHECKLIST = """
Manual hardware observations (see docs/hil_smoke_phase1.md for full detail):

  BOOT   - Power on with USB only. USER LED blinks; DXL bus LED is RED (power
           OFF) and stays off until an explicit enable command.
  USB    - Board enumerates as a USB CDC serial port; automated checks above
           cover the handshake.
  DXL    - With 12 V DXL power supplied and a servo on the bus, run a Phase 2
           DXL_SCAN (or the bench scan sketch) and confirm the servo is found
           with the correct model/protocol and torque OFF.
  I2C    - Confirm boot scan found the TCA9548A (0x70) and 24LC32 (0x50), and
           each populated foot channel (0..5) reports its VCNL4040 + LPS25HB.
  EEPROM - Commit a config, power-cycle, reload, and confirm it persists;
           corrupt a slot and confirm fallback to the other slot.
  CRSF   - Bind the ELRS link; confirm channels move and that powering the TX
           off raises the failsafe flag within ~250 ms.
  FAULT  - Trigger an estop / unsafe condition and confirm the firmware reports
           the fault and refuses motion.
"""


def resolve_port(requested: str | None) -> str | None:
    if requested:
        return requested
    candidates = [
        p.device
        for p in list_ports.comports()
        if "usbmodem" in p.device or "ttyACM" in p.device or "usbserial" in p.device
    ]
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        print(
            "No USB serial ports auto-detected. Pass --port explicitly.",
            file=sys.stderr,
        )
    else:
        print(
            f"Multiple ports found: {candidates}. Pass --port explicitly.",
            file=sys.stderr,
        )
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Phase 1 OpenRB-150 HIL smoke test")
    parser.add_argument("--port", help="USB CDC serial port (auto-detect if omitted)")
    parser.add_argument(
        "--baud", type=int, default=115200, help="baud (CDC ignores it)"
    )
    parser.add_argument(
        "--list", action="store_true", help="list candidate serial ports and exit"
    )
    args = parser.parse_args(argv)

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return 0

    port_name = resolve_port(args.port)
    if not port_name:
        return 2

    print(f"Opening {port_name} ...")
    try:
        port = serial.Serial(port_name, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {port_name}: {exc}", file=sys.stderr)
        return 2

    # Give the board a moment after the port opens (CDC reset on some hosts).
    time.sleep(0.3)

    with port:
        runner = SmokeRunner(port)
        ok = runner.run()

    total = len(runner.checks)
    passed = sum(1 for c in runner.checks if c.passed)
    print(f"\nAutomated result: {passed}/{total} checks passed.")
    print(MANUAL_CHECKLIST)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
