#!/usr/bin/env python3
"""Serial straight-walk drift test: tripod forward walk at 132 mm, recording
servo goals + present positions. Robot suspended or on a slick mat; this
measures COMMANDED vs ACHIEVED foot motion, not ground truth odometry."""
import json
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from hexapod_protocol import telemetry as tlm  # noqa: E402
from transport import list_serial_ports, open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402


def find_port() -> str:
    for p in list_serial_ports():
        hay = f"{p.device} {p.description}".lower()
        if "openrb" in hay:
            return p.device
    raise RuntimeError("OpenRB port not found")


PORT = find_port()
GAIT_TRIPOD = 2
WALK_S = 60.0

records = []
lock = threading.Lock()


def on_telemetry(stream_id, decoded, header):
    name = tlm.STREAM_NAMES.get(stream_id, str(stream_id))
    if name in ("servo_goals", "servo_status"):
        with lock:
            records.append((time.time(), name, decoded))


def main():
    link = open_transport(PORT, baud=115200)
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
    try:
        print("power:", c.dxl_power(True).code)
        time.sleep(4.0)
        scan = None
        for attempt in range(6):
            scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
            found = len(scan.servos()) if scan else 0
            print(f"scan attempt {attempt + 1}: {found}")
            if found == 18:
                break
            time.sleep(2.0)
        if not scan or len(scan.servos()) != 18:
            raise RuntimeError("scan failed")
        time.sleep(0.5)
        c.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        time.sleep(0.2)
        c.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        print("torque:", c.dxl_torque(True).code)
        time.sleep(0.5)
        for sid, rate in ((tlm.StreamId.SERVO_GOALS, 25),
                          (tlm.StreamId.SERVO_STATUS, 100)):
            c.subscribe(int(sid), rate)
        c.set_gait(GAIT_TRIPOD)
        c.set_gait_params(132, 60, 30, 159, 128)
        time.sleep(2.0)
        # vx = FORWARD in the command frame (vy would be a left strafe).
        print("walking forward for", WALK_S, "s")
        t0 = time.time()
        while time.time() - t0 < WALK_S:
            c.set_body_twist(0.6, 0.0, 0.0)
            time.sleep(0.2)
        c.stop_motion()
        time.sleep(0.5)
    finally:
        try:
            c.stop_motion()
            c.dxl_torque(False)
            c.dxl_power(False)
            c.exit_maintenance(tok)
        except Exception as e:  # noqa: BLE001
            print("cleanup:", e)
        stop_hb.set()
        c.stop()

    out = "data/sessions/straight_walk.jsonl"
    with open(out, "w") as f:
        for t, name, decoded in records:
            f.write(json.dumps({"t": t, "stream": name, "data": decoded},
                               default=lambda o: o.__dict__) + "\n")
    print(f"wrote {len(records)} records to {out}")


if __name__ == "__main__":
    main()
