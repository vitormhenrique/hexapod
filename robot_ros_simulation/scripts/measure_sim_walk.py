#!/usr/bin/env python3
"""Measure simulated walking: joint sweep per joint class and odometry drift."""

from __future__ import annotations

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from hexapod_protocol import api
from transport.protocol_client import ProtocolClient
from transport.tcp_proxy import connect_tcp_proxy


def main() -> int:
    endpoint = sys.argv[1]
    gait = getattr(api, f"GAIT_{sys.argv[2].upper()}") if len(sys.argv) > 2 else api.GAIT_RIPPLE
    vx, vy, wz = (float(v) for v in sys.argv[3:6]) if len(sys.argv) > 5 else (1.0, 0.0, 0.0)

    rclpy.init()
    node = Node("walk_probe")
    samples: list[list[float]] = []
    odom: list[tuple[float, float, float]] = []
    node.create_subscription(
        Float64MultiArray, "/position_controller/commands",
        lambda m: samples.append(list(m.data)), 50)

    def on_tf(msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id == "base_footprint":
                odom.append((t.transform.translation.x, t.transform.translation.y,
                             t.transform.rotation.z))

    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    client = ProtocolClient(connect_tcp_proxy(endpoint), response_timeout=1.0)
    client.start()
    try:
        print("arm:", client.set_arming(True).state, flush=True)
        print("gait ok:", client.set_gait(gait).ok, flush=True)
        print("params ok:", client.set_gait_params(40, 60, 30, 128, 200).ok, flush=True)
        print("twist ok:", client.set_body_twist(vx, vy, wz).ok, flush=True)
        time.sleep(8.0)
        client.set_body_twist(0.0, 0.0, 0.0)
        client.stop_motion()
    finally:
        client.stop()

    rclpy.shutdown()
    spinner.join(timeout=2.0)

    if not samples:
        print("NO JOINT COMMANDS RECEIVED")
        return 1
    per_joint = [
        (max(s[j] for s in samples) - min(s[j] for s in samples))
        for j in range(len(samples[0]))
    ]
    names = ["coxa", "femur", "tibia"]
    print(f"samples: {len(samples)}")
    for leg in range(6):
        sweep = ", ".join(
            f"{names[j]}={per_joint[leg * 3 + j]:.3f}" for j in range(3)
        )
        print(f"leg{leg + 1}: {sweep}")
    if odom:
        x, y, qz = odom[-1]
        print(f"odom: x={x:.3f} y={y:.3f} yaw={math.degrees(2 * math.asin(max(-1, min(1, qz)))):.1f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
