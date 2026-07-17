#!/usr/bin/env python3
"""Manual driver: send companion motion commands to a simulated endpoint."""

from __future__ import annotations

import sys
import time

from hexapod_protocol import api
from transport.protocol_client import ProtocolClient
from transport.tcp_proxy import connect_tcp_proxy


def main() -> int:
    endpoint = sys.argv[1] if len(sys.argv) > 1 else "tcp://127.0.0.1:5560?token=hexapod-sim"
    client = ProtocolClient(connect_tcp_proxy(endpoint), response_timeout=1.0)
    client.start()
    try:
        hello = client.hello()
        print("hello:", hello.device_name if hello else None, flush=True)
        print("set_arming:", client.set_arming(True), flush=True)
        print("set_gait ripple:", client.set_gait(api.GAIT_RIPPLE), flush=True)
        print("set_gait_params:", client.set_gait_params(60, 40, 30, 200, 220), flush=True)
        print("set_body_twist:", client.set_body_twist(0.8, 0.0, 0.0), flush=True)
        print("set_body_pose:", client.set_body_pose(0, 0, 20, 10, 0, 0), flush=True)
        time.sleep(1.0)
    finally:
        client.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
