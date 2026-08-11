#!/usr/bin/env python3
"""Floor discrimination test: forward, backward, strafe-left, strafe-right,
15 s each with 5 s pauses. Operator observes drift direction per segment.
Robot must be ON THE FLOOR with ~2 m clearance."""
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from transport import list_serial_ports, open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

GAIT_TRIPOD = 2
SEGMENTS = [
    ("FORWARD", (0.6, 0.0, 0.0)),
    ("BACKWARD", (-0.6, 0.0, 0.0)),
    ("YAW CCW in place", (0.0, 0.0, 0.5)),
    ("YAW CW in place", (0.0, 0.0, -0.5)),
]
SEG_S = 30.0
PAUSE_S = 5.0


def find_port():
    for p in list_serial_ports():
        if "openrb" in f"{p.device} {p.description}".lower():
            return p.device
    raise RuntimeError("OpenRB port not found")


def main():
    link = open_transport(find_port(), baud=115200)
    c = ProtocolClient(link)
    c.start()
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
        found = 0
        for attempt in range(8):
            scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
            found = len(scan.servos()) if scan else 0
            print(f"scan attempt {attempt + 1}: {found}")
            if found == 18:
                break
            time.sleep(2.0)
        if found != 18:
            raise RuntimeError("scan failed")
        time.sleep(0.5)
        c.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        time.sleep(0.2)
        c.set_maint_control_mode(api.MAINT_CONTROL_GAIT_PIPELINE)
        print("torque:", c.dxl_torque(True).code)
        time.sleep(1.0)
        c.set_gait_params(132, 60, 30, 159, 128)
        time.sleep(2.0)
        for name, (vx, vy, wz) in SEGMENTS:
            print(f"\n>>> {name} for {SEG_S:.0f} s -- observe drift <<<")
            c.set_gait(GAIT_TRIPOD)
            t0 = time.time()
            while time.time() - t0 < SEG_S:
                c.set_body_twist(vx, vy, wz)
                time.sleep(0.2)
            c.stop_motion()
            print(f"    {name} done; pausing {PAUSE_S:.0f} s")
            time.sleep(PAUSE_S)
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
    print("\ndone")


if __name__ == "__main__":
    main()
