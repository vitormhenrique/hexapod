"""Hardware-free tests for the Jetson bridge's shared-client boundary."""

from __future__ import annotations

import struct
import threading

from hexapod_jetson_bridge import JetsonBridge
from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import Header, MsgType, decode_frame_body, encode_frame

from transport import FrameExtractor
from transport.protocol_client import ProtocolClient


class RespondingStream:
    """In-memory stream that exercises the real shared ProtocolClient."""

    def __init__(self, handlers) -> None:
        self._handlers = handlers
        self._inbound = bytearray()
        self._lock = threading.Lock()
        self._extractor = FrameExtractor()
        self.request_ids: list[int] = []
        self.closed = False

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            if not self._inbound:
                return b""
            count = min(size, len(self._inbound))
            data = bytes(self._inbound[:count])
            del self._inbound[:count]
            return data

    def write(self, data: bytes) -> int:
        for frame in self._extractor.push(data):
            self._respond(frame)
        return len(data)

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._inbound)

    def close(self) -> None:
        self.closed = True

    def push_telemetry(self, stream_id: int, payload: bytes) -> None:
        header = Header(
            msg_type=int(MsgType.TELEMETRY), msg_id=api.MSG_TELEMETRY_BASE + stream_id
        )
        with self._lock:
            self._inbound.extend(encode_frame(header, payload))

    def _respond(self, frame: bytes) -> None:
        header, payload = decode_frame_body(frame[1:-1])
        self.request_ids.append(header.msg_id)
        response_payload, error = self._handlers[header.msg_id](payload)
        response = Header(
            msg_type=int(MsgType.RESPONSE),
            msg_id=header.msg_id,
            seq=header.seq,
            flags=api.FLAG_ERROR if error else 0,
        )
        with self._lock:
            self._inbound.extend(encode_frame(response, response_payload))


def _hello(_payload: bytes) -> tuple[bytes, bool]:
    return bytes([0, 3, 1, 2, 3]) + b"jetson-test".ljust(16, b"\0"), False


def _heartbeat(_payload: bytes) -> tuple[bytes, bool]:
    return struct.pack("<IB", 0x12345678, 7), False


def _subscribe(payload: bytes) -> tuple[bytes, bool]:
    stream_id, rate_hz = struct.unpack("<BH", payload)
    return struct.pack("<BBH", api.SUB_OK, stream_id, rate_hz), False


def _motion_ok(_payload: bytes) -> tuple[bytes, bool]:
    return bytes([api.MOTION_OK, 7, 1]), False


def _make_bridge() -> tuple[JetsonBridge, ProtocolClient, RespondingStream]:
    stream = RespondingStream(
        {
            api.MSG_HELLO: _hello,
            api.MSG_JETSON_HEARTBEAT: _heartbeat,
            api.MSG_SUBSCRIBE: _subscribe,
            api.MSG_SET_GAIT: _motion_ok,
            api.MSG_SET_BODY_TWIST: _motion_ok,
            api.MSG_SET_BODY_POSE: _motion_ok,
            api.MSG_STOP_MOTION: _motion_ok,
        }
    )
    client = ProtocolClient(stream, response_timeout=0.5)
    bridge = JetsonBridge(client, heartbeat_interval_s=60.0, heartbeat_timeout_s=61.0)
    return bridge, client, stream


def test_bridge_forwards_heartbeat_motion_and_typed_telemetry():
    bridge, _, stream = _make_bridge()
    received = []
    arrived = threading.Event()
    bridge.on_telemetry(lambda stream_id, record, _header: (received.append((stream_id, record)), arrived.set()))

    try:
        hello = bridge.start()
        assert hello is not None
        assert hello.device_name == "jetson-test"
        assert bridge.last_heartbeat is not None
        assert bridge.last_heartbeat.uptime_ms == 0x12345678
        assert bridge.heartbeat_fresh

        twist = bridge.set_body_twist(0.25, -0.5, 1.0)
        gait = bridge.set_gait(3)
        pose = bridge.set_body_pose(10.0, -20.0, 5.0, 1.0, -2.0, 3.0)
        assert twist is not None and twist.ok
        assert gait is not None and gait.ok
        assert pose is not None and pose.ok

        health = struct.pack("<IBBIH", 1234, 7, 0, 0, 12000)
        stream.push_telemetry(int(tlm.StreamId.HEALTH), health)
        assert arrived.wait(timeout=0.5)
        assert received[0][0] == int(tlm.StreamId.HEALTH)
        assert received[0][1].uptime_ms == 1234

        assert api.MSG_JETSON_HEARTBEAT in stream.request_ids
        assert api.MSG_HEARTBEAT not in stream.request_ids
        assert stream.request_ids.count(api.MSG_SUBSCRIBE) == 2
        assert api.MSG_SET_BODY_TWIST in stream.request_ids
        assert api.MSG_SET_GAIT in stream.request_ids
        assert api.MSG_SET_BODY_POSE in stream.request_ids
    finally:
        bridge.close()
    assert stream.closed


def test_bridge_blocks_stale_motion_but_always_forwards_stop():
    bridge, client, stream = _make_bridge()
    client.start()
    try:
        assert bridge.set_body_twist(0.2, 0.0, 0.0) is None
        stop = bridge.stop_motion()
        assert stop is not None and stop.ok
        assert stream.request_ids == [api.MSG_STOP_MOTION]
    finally:
        bridge.close()


def test_bridge_does_not_forward_motion_after_subscription_startup_failure():
    def rejected_subscribe(payload: bytes) -> tuple[bytes, bool]:
        stream_id, _ = struct.unpack("<BH", payload)
        return struct.pack("<BBH", api.SUB_BAD_STREAM, stream_id, 0), False

    stream = RespondingStream(
        {
            api.MSG_HELLO: _hello,
            api.MSG_JETSON_HEARTBEAT: _heartbeat,
            api.MSG_SUBSCRIBE: rejected_subscribe,
            api.MSG_SET_BODY_TWIST: _motion_ok,
        }
    )
    bridge = JetsonBridge(
        ProtocolClient(stream, response_timeout=0.5),
        heartbeat_interval_s=60.0,
        heartbeat_timeout_s=61.0,
    )
    try:
        assert bridge.start() is None
        assert bridge.heartbeat_fresh
        assert bridge.set_body_twist(0.2, 0.0, 0.0) is None
        assert api.MSG_SET_BODY_TWIST not in stream.request_ids
    finally:
        bridge.close()


def test_bridge_surface_excludes_actuator_and_arming_controls():
    bridge, _, _ = _make_bridge()
    for name in (
        "set_arming",
        "set_joint_target",
        "set_all_joint_targets",
        "dxl_torque",
        "dxl_write_register",
    ):
        assert not hasattr(bridge, name)