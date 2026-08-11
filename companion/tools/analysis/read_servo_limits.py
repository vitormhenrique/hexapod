#!/usr/bin/env python3
"""Read CW/CCW EEPROM angle limits from all 18 servos (maintenance session)."""
import sys
import threading
import time

sys.path.insert(0, "src")

from hexapod_protocol import api  # noqa: E402
from transport import open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

link = open_transport("/dev/cu.usbmodem1101", baud=115200)
c = ProtocolClient(link)
c.start()
m = c.enter_maintenance()
print("maint:", m)
if not (m and m.ok and m.token):
    c.stop()
    sys.exit(1)
tok = m.token
stop_hb = threading.Event()


def heartbeat():
    while not stop_hb.is_set():
        c.maint_heartbeat(tok)
        time.sleep(0.25)


threading.Thread(target=heartbeat, daemon=True).start()

try:
    p = c.dxl_power(True)
    print("power:", p.code if p else None)
    time.sleep(2.5)
    scan = c.dxl_run(api.build_dxl_scan(1, 18), timeout=20.0)
    servos = scan.servos() if scan else []
    print("scan servos:", len(servos))
    if servos:
        print("first profile:", servos[0])
    print("\nid  cw_limit ccw_limit")
    for sid in range(1, 19):
        cw = c.dxl_get_param(sid, api.DXL_PARAM_CW_ANGLE_LIMIT)
        ccw = c.dxl_get_param(sid, api.DXL_PARAM_CCW_ANGLE_LIMIT)
        cwv = cw.param().value if cw and cw.param() else None
        ccwv = ccw.param().value if ccw and ccw.param() else None
        print(f"{sid:2d}  {cwv}     {ccwv}")
finally:
    c.dxl_power(False)
    c.exit_maintenance(tok)
    stop_hb.set()
    c.stop()
