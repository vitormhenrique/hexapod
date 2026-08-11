#!/usr/bin/env python3
"""Serial-driven crouch-walk bench test (no RC needed).

MacMaintenance lock + GaitPipeline control mode drive the full gait stack over
USB. Robot MUST be suspended. Records servo goals vs present positions at
body height 132 (baseline) and 65 (repro), then reports per-servo tracking.
"""
import json
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from hexapod_protocol import telemetry as tlm  # noqa: E402
from transport import open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

PORT = "/dev/cu.usbmodem1101"
GAIT_TRIPOD = 2  # wire gait id used by SET_GAIT (matches config::GaitId order)

records = []  # (t_host, stream_name, decoded)
records_lock = threading.Lock()


def on_telemetry(stream_id, decoded, header):
    name = tlm.STREAM_NAMES.get(stream_id, str(stream_id))
    if name in ("servo_goals", "servo_status", "control_state"):
        with records_lock:
            records.append((time.time(), name, decoded))


def require(name, r, ok=lambda r: r is not None):
    if not ok(r):
        raise RuntimeError(f"{name} failed: {r!r}")
    return r


def main():
    link = open_transport(PORT, baud=115200)
    client = ProtocolClient(link)
    client.start()
    client.on_telemetry(on_telemetry)

    m = require("ENTER_MAINTENANCE", client.enter_maintenance(),
                lambda r: r is not None and r.ok and r.token)
    token = m.token
    print(f"maintenance token={token}")

    stop_hb = threading.Event()

    def heartbeat():
        while not stop_hb.is_set():
            client.maint_heartbeat(token)
            time.sleep(0.25)

    hb = threading.Thread(target=heartbeat, daemon=True)
    hb.start()

    try:
        p = require("DXL power on", client.dxl_power(True))
        pr = p.power()
        print(f"power job: code={p.code} slot={p.slot} power={pr}")
        time.sleep(2.5)  # servo boot
        scan = require(
            "DXL scan",
            client.dxl_run(api.build_dxl_scan(1, 18), timeout=15.0))
        print(f"scan job: code={scan.code} slot={scan.slot}")
        servos = scan.servos()
        print(f"scan: {len(servos)} servos")
        if len(servos) != 18:
            raise RuntimeError("expected 18 servos")

        # Session edge resets control mode once; set it after the edge settles.
        time.sleep(0.5)
        r = client.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        print("control mode ->", r)
        time.sleep(0.2)
        r2 = client.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        print("control mode (confirm) ->", r2)

        r3 = require("torque on", client.dxl_torque(True))
        print(f"torque job: code={r3.code}")
        time.sleep(0.5)

        # Subscribe telemetry.
        for sid, rate in ((tlm.StreamId.SERVO_GOALS, 50),
                          (tlm.StreamId.SERVO_STATUS, 50),
                          (tlm.StreamId.CONTROL_STATE, 20)):
            require(f"subscribe {sid}", client.subscribe(int(sid), rate))

        require("gait", client.set_gait(GAIT_TRIPOD))

        def phase(name, height, walk_s):
            # STOP_MOTION resets gait to Stand; re-select the gait per phase.
            rg = client.set_gait(GAIT_TRIPOD)
            print(f"  gait -> result={rg.result if rg else None}")
            rp = client.set_gait_params(height, 60, 30, 159, 128)
            print(f"  params({height}) -> result={rp.result if rp else None}")
            time.sleep(3.0)  # let the height filter settle
            with records_lock:
                records.append((time.time(), "marker", {"phase": name}))
            rt = client.set_body_twist(0.0, 0.6, 0.0)
            print(f"  twist -> result={rt.result if rt else None} "
                  f"state={rt.state if rt else None} "
                  f"allowed={rt.motion_allowed if rt else None}")
            t0 = time.time()
            while time.time() - t0 < walk_s:
                client.set_body_twist(0.0, 0.6, 0.0)  # refresh TTL
                time.sleep(0.2)
            require("stop", client.stop_motion())
            with records_lock:
                records.append((time.time(), "marker", {"phase": name + "_end"}))
            time.sleep(1.0)

        for nm, h in (("h132", 132), ("h100", 100), ("h80", 80), ("h70", 70),
                      ("h65", 65), ("h132b", 132)):
            print(f"phase: {nm}")
            phase(nm, h, 8.0)

    finally:
        try:
            client.stop_motion()
            client.dxl_torque(False)
            client.dxl_power(False)
            client.exit_maintenance(token)
        except Exception as e:  # noqa: BLE001
            print("cleanup error:", e)
        stop_hb.set()
        client.stop()

    out = "data/sessions/serial_crouch_test.jsonl"
    with open(out, "w") as f:
        for t, name, decoded in records:
            if name == "marker":
                f.write(json.dumps({"t": t, "stream": name, "data": decoded}) + "\n")
            else:
                d = decoded if isinstance(decoded, dict) else getattr(
                    decoded, "__dict__", None)
                if d is None:
                    d = json.loads(json.dumps(decoded, default=lambda o: o.__dict__))
                f.write(json.dumps({"t": t, "stream": name, "data": d},
                                   default=lambda o: o.__dict__) + "\n")
    print(f"wrote {len(records)} records to {out}")


if __name__ == "__main__":
    main()
