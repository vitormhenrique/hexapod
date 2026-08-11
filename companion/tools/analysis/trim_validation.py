#!/usr/bin/env python3
"""Trim-aware apex validation (robot on stand).

Reads the live config trims, commands neutral + the 8 walk-phase poses as
joint targets, and checks present ticks against angle*11.377 + 2048 +
sign*trim per servo. Validates the entire chain: config -> servo map ->
goals -> physical servo, including the new coxa trim.
"""
import json
import math
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from hexapod_protocol import telemetry as tlm  # noqa: E402
from transport import list_serial_ports, open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

TICKS_PER_DEG = 4096.0 / 360.0
CENTER = 2048
TOL_TICKS = 20  # pass threshold per joint (static, incl. gravity droop)

phases = json.load(open("data/sessions/walk_phases.json"))

latest = {}
lock = threading.Lock()


def on_telemetry(stream_id, decoded, header):
    if tlm.STREAM_NAMES.get(stream_id) == "servo_status":
        servos = getattr(decoded, "servos", None)
        if servos:
            with lock:
                for s in servos:
                    latest[s.id] = s.position


def find_port():
    for p in list_serial_ports():
        if "openrb" in f"{p.device} {p.description}".lower():
            return p.device
    raise RuntimeError("OpenRB port not found")


def main():
    link = open_transport(find_port(), baud=115200)
    c = ProtocolClient(link)
    c.start()
    c.on_telemetry(on_telemetry)
    cfg = c.read_config()
    trims = {}   # (leg, joint) -> (sign, trim)
    sid_of = {}  # (leg, joint) -> id
    for s in cfg.servos:
        trims[(s.leg, s.joint)] = (s.sign, s.trim_ticks)
        sid_of[(s.leg, s.joint)] = s.id
    coxa_trims = sorted({t for (lg, j), (sg, t) in trims.items() if j == 0})
    print("coxa trims:", coxa_trims)

    m = c.enter_maintenance()
    if not (m and m.ok and m.token):
        print("maintenance rejected:", m)
        sys.exit(1)
    tok = m.token
    stop_hb = threading.Event()

    def hb():
        while not stop_hb.is_set():
            c.maint_heartbeat(tok)
            time.sleep(0.25)

    threading.Thread(target=hb, daemon=True).start()
    failures = 0
    try:
        print("power:", c.dxl_power(True).code)
        time.sleep(4.0)
        for attempt in range(8):
            scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
            if scan and len(scan.servos()) == 18:
                break
            time.sleep(2.0)
        print("torque:", c.dxl_torque(True).code)
        time.sleep(0.5)
        c.subscribe(int(tlm.StreamId.SERVO_STATUS), 50)

        tests = [("neutral", {str(sid_of[(lg, j)]): [2048, lg, j]
                              for lg in range(6) for j in range(3)})]
        tests += [(f"phase{e['phase']}", e["ticks"]) for e in phases]

        print("\npose      worst_id  cmd_tick exp_tick present  err")
        for name, ticks in tests:
            angles = [0] * 18
            expect = {}
            for sid_s, (tick, leg, joint) in ticks.items():
                cdeg = round((tick - CENTER) / TICKS_PER_DEG * 100.0)
                angles[leg * 3 + joint] = cdeg
                sign, trim = trims[(leg, joint)]
                exp = round(CENTER + trim + sign * (cdeg / 100.0) * TICKS_PER_DEG)
                expect[int(sid_s)] = (tick, exp)
            c.set_all_joint_targets(angles)
            time.sleep(1.8)
            with lock:
                snap = dict(latest)
            worst = (0, None, None, None, None)
            for sid, (cmd, exp) in sorted(expect.items()):
                pos = snap.get(sid)
                if pos is None:
                    continue
                err = pos - exp
                if abs(err) > abs(worst[0]):
                    worst = (err, sid, cmd, exp, pos)
            err, sid, cmd, exp, pos = worst
            status = "OK " if abs(err) <= TOL_TICKS else "FAIL"
            if abs(err) > TOL_TICKS:
                failures += 1
            print(f"{name:9s}  {sid:2d}       {cmd}     {exp}     {pos}   "
                  f"{err:+4d}  {status}")
    finally:
        try:
            c.dxl_torque(False)
            c.dxl_power(False)
            c.exit_maintenance(tok)
        except Exception as e:  # noqa: BLE001
            print("cleanup:", e)
        stop_hb.set()
        c.stop()
    print(f"\n{'ALL POSES PASS' if failures == 0 else f'{failures} POSES FAIL'}"
          f" (tolerance {TOL_TICKS} ticks)")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
