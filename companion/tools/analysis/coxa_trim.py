#!/usr/bin/env python3
"""Apply a stride-axis yaw correction: set trim_ticks on all six COXA servos.
Usage: coxa_trim.py [trim_ticks]   (default -33 = rotate stride axis CW 2.9deg)
Writes the config shadow and commits to OpenLog."""
import sys
import time

sys.path.insert(0, "src")

from transport import list_serial_ports, open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402

TRIM = int(sys.argv[1]) if len(sys.argv) > 1 else -33


def find_port():
    for p in list_serial_ports():
        if "openrb" in f"{p.device} {p.description}".lower():
            return p.device
    raise RuntimeError("OpenRB port not found")


link = open_transport(find_port(), baud=115200)
c = ProtocolClient(link)
c.start()
m = c.enter_maintenance()
if not (m and m.ok and m.token):
    print("maintenance rejected:", m)
    sys.exit(1)
tok = m.token
import threading  # noqa: E402

stop_hb = threading.Event()


def hb():
    while not stop_hb.is_set():
        c.maint_heartbeat(tok)
        time.sleep(0.25)


threading.Thread(target=hb, daemon=True).start()
try:
    cfg = c.read_config()
    if cfg is None:
        raise RuntimeError("read_config failed")
    changed = []
    for s in cfg.servos:
        if s.joint == 0:  # coxa
            changed.append((s.id, s.trim_ticks, TRIM))
            s.trim_ticks = TRIM
    for sid, old, new in changed:
        print(f"servo {sid}: trim {old} -> {new}")
    ok = c.write_config(cfg)
    print("stage:", ok)
    v = c.cfg_validate()
    print("validate:", v)
    r = c.cfg_commit()
    print("commit:", r)
    time.sleep(0.5)
    back = c.read_config()
    verify = all(s.trim_ticks == TRIM for s in back.servos if s.joint == 0)
    print("verified:", verify)
    if not (ok and verify):
        sys.exit(1)
finally:
    stop_hb.set()
    c.exit_maintenance(tok)
    c.stop()
