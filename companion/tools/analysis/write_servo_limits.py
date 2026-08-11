#!/usr/bin/env python3
"""Write CW/CCW EEPROM angle limits 1024..3072 to all 18 servos, verified."""
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from transport import open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

MIN_TICK = 1024
MAX_TICK = 3072

link = open_transport("/dev/cu.usbmodem1101", baud=115200)
c = ProtocolClient(link)
c.start()
m = c.enter_maintenance()
if not (m and m.ok and m.token):
    print("maintenance rejected:", m)
    c.stop()
    sys.exit(1)
tok = m.token
stop_hb = threading.Event()


def heartbeat():
    while not stop_hb.is_set():
        c.maint_heartbeat(tok)
        time.sleep(0.25)


threading.Thread(target=heartbeat, daemon=True).start()

failures = []
try:
    p = c.dxl_power(True)
    print("power:", p.code if p else None)
    time.sleep(2.5)
    scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
    print("scan servos:", len(scan.servos()) if scan else None)

    for sid in range(1, 19):
        r = c.dxl_set_servo_limits(sid, MIN_TICK, MAX_TICK)
        sl = r.servo_limits() if r else None
        ok = sl is not None and sl.verified
        print(f"servo {sid:2d}: code={r.code if r else None} "
              f"limits={sl} verified={ok}")
        if not ok:
            failures.append(sid)

    print("\nread-back check:")
    for sid in range(1, 19):
        cw = c.dxl_get_param(sid, api.DXL_PARAM_CW_ANGLE_LIMIT)
        ccw = c.dxl_get_param(sid, api.DXL_PARAM_CCW_ANGLE_LIMIT)
        cwv = cw.param().value if cw and cw.param() else None
        ccwv = ccw.param().value if ccw and ccw.param() else None
        mark = "OK" if (cwv == MIN_TICK and ccwv == MAX_TICK) else "MISMATCH"
        print(f"servo {sid:2d}: cw={cwv} ccw={ccwv}  {mark}")
        if mark != "OK":
            failures.append(sid)
finally:
    c.dxl_power(False)
    c.exit_maintenance(tok)
    stop_hb.set()
    c.stop()

if failures:
    print("FAILURES:", sorted(set(failures)))
    sys.exit(1)
print("all 18 servos verified at 1024..3072")
