#!/usr/bin/env python3
"""Static apex-pose accuracy test: command each of the 8 walk-cycle phases as
frozen joint targets (maintenance JointTargets mode), let the servos settle,
and compare present positions to the commanded ticks, per servo."""
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

phases = json.load(open("data/sessions/walk_phases.json"))

latest_status = {}
status_lock = threading.Lock()


def on_telemetry(stream_id, decoded, header):
    if tlm.STREAM_NAMES.get(stream_id) == "servo_status":
        servos = getattr(decoded, "servos", None) or (
            decoded.get("servos") if isinstance(decoded, dict) else None)
        if servos:
            with status_lock:
                for s in servos:
                    sid = s["id"] if isinstance(s, dict) else s.id
                    pos = s["position"] if isinstance(s, dict) else s.position
                    latest_status[sid] = pos


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
    results = []
    try:
        print("power:", c.dxl_power(True).code)
        time.sleep(2.5)
        for attempt in range(4):
            scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
            if scan and len(scan.servos()) == 18:
                break
            time.sleep(1.5)
        print("scan:", len(scan.servos()))
        # JointTargets is the default control mode after session start.
        print("torque:", c.dxl_torque(True).code)
        time.sleep(0.5)
        c.subscribe(int(tlm.StreamId.SERVO_STATUS), 50)

        for entry in phases:
            ticks = entry["ticks"]  # id -> [tick, leg, joint]
            # leg-major angle list
            angles = [0] * 18
            expect = {}
            for sid_s, (tick, leg, joint) in ticks.items():
                cdeg = round((tick - CENTER) / TICKS_PER_DEG * 100.0)
                angles[leg * 3 + joint] = cdeg
                expect[int(sid_s)] = tick
            r = c.set_all_joint_targets(angles)
            time.sleep(1.8)  # settle
            with status_lock:
                snap = dict(latest_status)
            row = {"phase": entry["phase"], "deltas": {}}
            for sid, tick in sorted(expect.items()):
                pos = snap.get(sid)
                row["deltas"][sid] = (tick, pos,
                                      (pos - tick) if pos is not None else None)
            results.append(row)
            worst = max((abs(d[2]) for d in row["deltas"].values()
                         if d[2] is not None), default=-1)
            print(f"phase {entry['phase']}: worst |delta| = {worst} ticks")
    finally:
        try:
            c.dxl_torque(False)
            c.dxl_power(False)
            c.exit_maintenance(tok)
        except Exception as e:  # noqa: BLE001
            print("cleanup:", e)
        stop_hb.set()
        c.stop()

    print("\nper-servo tracking error (present - commanded, ticks):")
    print("id : " + "  ".join(f"ph{p['phase']}" for p in results) + "   mean")
    for sid in range(1, 19):
        vals = [p["deltas"][sid][2] for p in results if p["deltas"].get(sid)]
        vals = [v for v in vals if v is not None]
        mean = sum(vals) / len(vals) if vals else float("nan")
        print(f"{sid:2d} : " + "  ".join(f"{v:+4d}" for v in vals) +
              f"   {mean:+6.1f}")
    with open("data/sessions/apex_results.json", "w") as f:
        json.dump(results, f, indent=1)


if __name__ == "__main__":
    main()
