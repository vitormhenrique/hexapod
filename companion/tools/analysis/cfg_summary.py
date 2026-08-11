#!/usr/bin/env python3
"""Print CFG summary flags (persistent/staged-valid)."""
import sys

sys.path.insert(0, "src")

from transport import list_serial_ports, open_transport  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402


def find_port():
    for p in list_serial_ports():
        if "openrb" in f"{p.device} {p.description}".lower():
            return p.device
    raise RuntimeError("OpenRB port not found")


link = open_transport(find_port(), baud=115200)
c = ProtocolClient(link)
c.start()
try:
    s = c.cfg_get_summary()
    print(s)
finally:
    c.stop()
