"""Interactive servo direction validation for all 18 hexapod joints.

Walks every leg/joint, centers all servos to 180 deg, drives one joint to
200 deg, prints the movement direction the *firmware model* expects (URDF
forward kinematics + the per-servo sign from the default config), and asks
whether the physical robot matched. Results are stored and, at the end, the
script prints exactly which servos need a sign flip in the firmware.

Run it (robot connected, legs free to move, from the companion/ dir):

    uv run python validate_dir.py

Optional overrides:

    uv run python validate_dir.py --port /dev/cu.usbmodem2101 --baud 115200
    uv run python validate_dir.py --only 2,3,4      # legs to test (1-based)

Per-joint prompt:
    y = moved the expected way        n = moved the wrong way
    r = repeat this joint             s = skip (record as unknown)
    q = quit early (summary is still printed)
"""

from __future__ import annotations

import math
import sys
import time

from hexapod_protocol import telemetry as tlm

from cli import _connect
from data.urdf_fk import UrdfForwardKinematics
from data.urdf_model import find_hexnav_description, load_hexnav
from hil import _MaintLock, _wait_state, STATE_MAC_MAINTENANCE

ROLE_NAMES = ["coxa", "femur", "tibia"]
TICKS_PER_DEG = 4096 / 360.0

# The servo is driven to a physical 200 deg (a +20 deg move from the 180 deg
# center). The firmware applies the per-leg sign, so the command magnitude is
# 2000 centidegrees and left/right legs get opposite command signs.
MOVE_DEG = 20.0


# ---------------------------------------------------------------------------
# Firmware model: per-servo sign + predicted foot motion direction.
# ---------------------------------------------------------------------------


def leg_sign(leg0: int) -> int:
    """Config sign: all legs use +1 (hardware-validated)."""
    return 1


class Predictor:
    """Predicts the physical foot-motion direction for a 180->200 servo move.

    Uses the flattened HexNav URDF forward kinematics (the same model the URDF
    viewer renders) plus the config sign, so predictions match the firmware.
    """

    def __init__(self) -> None:
        assert (
            find_hexnav_description() is not None
        ), "HexNav_description not found; cannot predict directions"
        self._fk = UrdfForwardKinematics(load_hexnav())
        self._home = self._fk.link_positions({})

    def _tip(self, leg0: int) -> tuple:
        return self._home[f"leg_{leg0 + 1}_tibia"]

    def _nearest_leg(self, leg0: int, dx: float, dy: float) -> int:
        """Which other leg's tip direction best matches a horizontal move."""
        hx, hy, _ = self._tip(leg0)
        mv = math.hypot(dx, dy)
        best, best_dot = leg0, -2.0
        for other in range(6):
            if other == leg0:
                continue
            ox, oy, _ = self._tip(other)
            vx, vy = ox - hx, oy - hy
            vn = math.hypot(vx, vy)
            if vn < 1e-9 or mv < 1e-9:
                continue
            dot = (dx * vx + dy * vy) / (mv * vn)
            if dot > best_dot:
                best_dot, best = dot, other
        return best

    def predict(self, leg0: int, joint: int) -> str:
        """Human-readable expected physical motion for this servo move."""
        q = leg_sign(leg0) * math.radians(MOVE_DEG)
        name = f"leg_{leg0 + 1}_{ROLE_NAMES[joint]}_joint"
        moved = self._fk.link_positions({name: q})[f"leg_{leg0 + 1}_tibia"]
        home = self._tip(leg0)
        dx = (moved[0] - home[0]) * 1000.0
        dy = (moved[1] - home[1]) * 1000.0
        dz = (moved[2] - home[2]) * 1000.0
        if joint == 0:  # coxa: horizontal swing toward a neighbor leg
            nb = self._nearest_leg(leg0, dx, dy)
            return f"coxa swings the foot TOWARD leg {nb + 1}"
        verb = "RAISES the foot (UP)" if dz > 0 else "LOWERS the foot (DOWN)"
        return f"{ROLE_NAMES[joint]} {verb}"


# ---------------------------------------------------------------------------
# Live present-position telemetry.
# ---------------------------------------------------------------------------

_latest: dict[int, int] = {}


def _on_tel(stream_id, record, header) -> None:
    if stream_id == int(tlm.StreamId.SERVO_STATUS):
        for s in record.servos:
            _latest[s.id] = s.position


def snapshot(ids) -> dict:
    """Latest present ticks for the given servo ids (waits briefly for data)."""
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline and not all(i in _latest for i in ids):
        time.sleep(0.05)
    return {i: _latest.get(i, 0) for i in ids}


# ---------------------------------------------------------------------------
# CLI arg parsing (tiny; avoids a Typer app for a one-off tool).
# ---------------------------------------------------------------------------


def _parse_args(argv: list[str]) -> tuple[str | None, int, list[int]]:
    port: str | None = None
    baud = 115200
    legs = list(range(6))
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--port":
            port = argv[i + 1]
            i += 2
        elif a == "--baud":
            baud = int(argv[i + 1])
            i += 2
        elif a == "--only":
            legs = [int(x) - 1 for x in argv[i + 1].split(",") if x.strip()]
            i += 2
        else:
            print(f"unknown arg: {a}")
            i += 1
    return port, baud, legs


def _ask(prompt: str) -> str:
    while True:
        ans = input(prompt).strip().lower()
        if ans in ("y", "n", "s", "r", "q"):
            return ans
        print("  please answer y / n / r / s / q")


# ---------------------------------------------------------------------------
# Main interactive validation loop.
# ---------------------------------------------------------------------------


def main() -> int:
    port, baud, legs = _parse_args(sys.argv[1:])
    predictor = Predictor()

    client = _connect(port, baud)
    client.on_telemetry(_on_tel)
    lock = _MaintLock(client)

    # results[(leg0, joint)] = "correct" | "wrong" | "skipped"
    results: dict[tuple[int, int], str] = {}

    try:
        if not lock.acquire():
            print("FAILED: could not acquire maintenance lock")
            return 1
        _wait_state(client, lambda s: s == STATE_MAC_MAINTENANCE)

        pw = client.dxl_power(True)
        pr = pw.power() if pw and pw.done else None
        if pr is None or not pr.power_on:
            print("FAILED: DXL power on")
            return 1

        # MX-28s need ~1 s after power-on to answer; scan 1..18 to avoid the
        # out-of-range scan watchdog trip.
        found = []
        for _ in range(4):
            time.sleep(1.0)
            scan = client.dxl_scan(1, 18)
            found = scan.servos() if scan and scan.done else []
            if found:
                break
        print(f"scanned {len(found)} servos\n")
        client.subscribe(int(tlm.StreamId.SERVO_STATUS), 10)

        print("=" * 64)
        print("Servo direction validation - 180 deg center, drive one to 200 deg")
        print("Answer per joint: y=correct  n=wrong  r=repeat  s=skip  q=quit")
        print("=" * 64)

        quit_early = False
        for leg0 in legs:
            if quit_early:
                break
            for joint in range(3):
                sid = leg0 * 3 + joint + 1
                expected = predictor.predict(leg0, joint)

                while True:  # allows 'r' repeat
                    # Recenter every joint so only the joint under test moves.
                    client.set_all_joint_targets([0] * 18)
                    time.sleep(1.2)
                    _latest.clear()
                    before = snapshot([sid])

                    cmd = leg_sign(leg0) * int(MOVE_DEG * 100)
                    res = client.set_joint_target(leg0, joint, cmd)
                    ok = res is not None and getattr(res, "ok", False)
                    time.sleep(1.2)
                    _latest.clear()
                    after = snapshot([sid])
                    delta = after.get(sid, 0) - before.get(sid, 0)

                    print(
                        f"\n[leg {leg0 + 1} {ROLE_NAMES[joint]}] servo id {sid}, "
                        f"sign {leg_sign(leg0):+d}"
                    )
                    print(
                        f"  command : {cmd:+d} cdeg -> 200 deg "
                        f"({'accepted' if ok else 'REJECTED'})"
                    )
                    print(
                        f"  measured: {before.get(sid, 0)} -> {after.get(sid, 0)} "
                        f"ticks ({delta:+d}, ~{delta / TICKS_PER_DEG:+.1f} deg)"
                    )
                    print(f"  EXPECTED: {expected}")

                    ans = _ask("  Did it move as expected? [y/n/r/s/q] ")
                    if ans == "r":
                        continue
                    if ans == "y":
                        results[(leg0, joint)] = "correct"
                    elif ans == "n":
                        results[(leg0, joint)] = "wrong"
                    elif ans == "s":
                        results[(leg0, joint)] = "skipped"
                    elif ans == "q":
                        quit_early = True
                    break

        # Return to a safe centered pose before dropping torque.
        client.set_all_joint_targets([0] * 18)
        time.sleep(1.0)
        client.unsubscribe(int(tlm.StreamId.SERVO_STATUS))

        _print_summary(results)
        return 0
    finally:
        lock.release()
        client.stop()


def _print_summary(results: dict[tuple[int, int], str]) -> None:
    print("\n" + "=" * 64)
    print("SUMMARY")
    print("=" * 64)
    correct = [k for k, v in results.items() if v == "correct"]
    wrong = sorted(k for k, v in results.items() if v == "wrong")
    skipped = sorted(k for k, v in results.items() if v == "skipped")

    print(
        f"  correct: {len(correct)}   wrong: {len(wrong)}   "
        f"skipped: {len(skipped)}   tested: {len(results)}/18"
    )

    if skipped:
        names = ", ".join(f"leg{l + 1} {ROLE_NAMES[j]}" for l, j in skipped)
        print(f"\n  skipped (not evaluated): {names}")

    print("\nFirmware changes needed:")
    if not wrong:
        print("  NONE - every tested joint moved the way the firmware predicts.")
        print("  The config signs in config_schema.cpp are correct.")
        return

    print("  The following servos moved the WRONG way. Flip each servo's `sign`")
    print("  in firmware/openrb150/src/config/config_schema.cpp (default servo")
    print("  map) - and the matching sign in protocol/python/hexapod_protocol/")
    print("  config.py - then re-flash and re-run this script to confirm.\n")
    print(
        f"    {'servo id':>8}  {'leg':>4}  {'joint':<6}  {'sign now':>8}  "
        f"{'-> new':>6}"
    )
    for leg0, joint in wrong:
        sid = leg0 * 3 + joint + 1
        cur = leg_sign(leg0)
        print(
            f"    {sid:>8}  {leg0 + 1:>4}  {ROLE_NAMES[joint]:<6}  "
            f"{cur:>+8d}  {-cur:>+6d}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
