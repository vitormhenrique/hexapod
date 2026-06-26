"""Loopback tests for the threaded :class:`ProtocolClient`.

A :class:`RespondingStream` answers each command frame with a canned response
that echoes the request seq, so we exercise the real reader thread, the
seq-matched request/response correlation, and the typed control-group methods
without any serial hardware.
"""

from __future__ import annotations

import threading

from hexapod_protocol import api
from hexapod_protocol.framing import Header, MsgType, decode_frame_body, encode_frame

from transport import FrameExtractor
from transport.protocol_client import ProtocolClient


class RespondingStream:
    """In-memory ByteStream that auto-answers commands via a handler table.

    ``handlers`` maps msg_id -> a function (req_payload) -> (resp_payload, error).
    Unmapped msg_ids get an UNKNOWN_MSG error response.
    """

    def __init__(self, handlers) -> None:
        self._handlers = handlers
        self._inbound = bytearray()
        self._lock = threading.Lock()
        self._extractor = FrameExtractor()
        self.closed = False
        self.tx_count = 0

    # ByteStream protocol -------------------------------------------------
    def read(self, size: int = 1) -> bytes:
        with self._lock:
            if not self._inbound:
                return b""
            n = min(size, len(self._inbound))
            chunk = bytes(self._inbound[:n])
            del self._inbound[:n]
            return chunk

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

    # internals -----------------------------------------------------------
    def _respond(self, frame: bytes) -> None:
        header, payload = decode_frame_body(frame[1:-1])
        self.tx_count += 1
        handler = self._handlers.get(header.msg_id)
        if handler is None:
            resp_payload, error = bytes([api.ERR_UNKNOWN_MSG]), True
        else:
            resp_payload, error = handler(payload)
        resp = Header(
            msg_type=int(MsgType.RESPONSE),
            msg_id=header.msg_id,
            seq=header.seq,
            flags=api.FLAG_ERROR if error else 0,
        )
        wire = encode_frame(resp, resp_payload)
        with self._lock:
            self._inbound.extend(wire)

    def push_telemetry(self, stream_id: int, payload: bytes) -> None:
        header = Header(
            msg_type=int(MsgType.TELEMETRY),
            msg_id=api.MSG_TELEMETRY_BASE + stream_id,
        )
        with self._lock:
            self._inbound.extend(encode_frame(header, payload))


def _ctrl_ok(state: int = 2, fault: int = 0):
    return lambda _p: (bytes([api.CTRL_OK, state, fault]), False)


def _motion_ok(state: int = 5):
    return lambda _p: (bytes([api.MOTION_OK, state, 1]), False)


def _make_client(handlers):
    stream = RespondingStream(handlers)
    client = ProtocolClient(stream, response_timeout=1.0)
    client.start()
    return client, stream


def test_estop_roundtrip():
    client, stream = _make_client({api.MSG_ESTOP: _ctrl_ok(state=12)})
    try:
        res = client.estop()
        assert res is not None
        assert res.ok and res.state == 12
        assert stream.tx_count == 1
    finally:
        client.stop()


def test_arm_disarm_and_clear_fault():
    handlers = {
        api.MSG_SET_ARMING: _ctrl_ok(state=4),
        api.MSG_CLEAR_FAULT: _ctrl_ok(state=2),
    }
    client, _ = _make_client(handlers)
    try:
        arm = client.set_arming(True)
        assert arm is not None and arm.ok and arm.state == 4
        disarm = client.set_arming(False)
        assert disarm is not None and disarm.ok
        cf = client.clear_fault()
        assert cf is not None and cf.ok and cf.state == 2
    finally:
        client.stop()


def test_set_mode_echoes_arming_payload():
    seen = {}

    def handler(payload):
        seen["mode"] = payload[0]
        return bytes([api.CTRL_OK, payload[0], 0]), False

    client, _ = _make_client({api.MSG_SET_MODE: handler})
    try:
        res = client.set_mode(8)
        assert res is not None and res.state == 8
        assert seen["mode"] == 8
    finally:
        client.stop()


def test_motion_commands():
    handlers = {
        api.MSG_SET_GAIT: _motion_ok(),
        api.MSG_STOP_MOTION: _motion_ok(state=3),
    }
    client, _ = _make_client(handlers)
    try:
        g = client.set_gait(api.GAIT_TRIPOD)
        assert g is not None and g.ok
        s = client.stop_motion()
        assert s is not None and s.ok and s.state == 3
    finally:
        client.stop()


def test_feature_set_roundtrip():
    def handler(payload):
        feature, enable = payload[0], payload[1]
        # [result, state, id, avail, enabled, reason]
        return bytes([api.FEATURE_OK, 2, feature, 1, enable, 0]), False

    client, _ = _make_client({api.MSG_FEATURE_SET: handler})
    try:
        res = client.feature_set(api.FEATURE_SENSOR_POLLING, True)
        assert res is not None
        assert res.feature == api.FEATURE_SENSOR_POLLING
        assert res.enabled is True
    finally:
        client.stop()


def test_passive_enter_exit():
    handlers = {
        api.MSG_PASSIVE_ENTER: lambda _p: (bytes([api.PASSIVE_OK, 9]), False),
        api.MSG_PASSIVE_EXIT: lambda _p: (bytes([api.PASSIVE_OK, 2]), False),
    }
    client, _ = _make_client(handlers)
    try:
        enter = client.passive_enter()
        assert enter is not None and enter.ok and enter.state == 9
        exit_ = client.passive_exit()
        assert exit_ is not None and exit_.ok and exit_.state == 2
    finally:
        client.stop()


def test_dxl_run_polls_until_done():
    # A single-slot job queue: submit accepts, GET_RESULT first returns RUNNING
    # then DONE. dxl_run must poll past the RUNNING slot.
    state = {"polls": 0}
    job_id = 7

    def submit(_p):
        return bytes([api.DXL_SUBMIT_ACCEPTED, job_id, api.DXL_SLOT_PENDING]), False

    def get_result(_p):
        state["polls"] += 1
        if state["polls"] < 2:
            return bytes([api.DXL_SLOT_RUNNING, api.DXL_CODE_OK, 0]), False
        # DONE with a 2-byte data blob.
        return (
            bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, 2, 0xAB, 0xCD]),
            False,
        )

    handlers = {
        api.MSG_DXL_GET_PARAM: submit,
        api.MSG_DXL_GET_RESULT: get_result,
    }
    client, _ = _make_client(handlers)
    try:
        res = client.dxl_get_param(servo_id=1, param=api.DXL_PARAM_CW_ANGLE_LIMIT)
        assert res is not None
        assert res.slot == api.DXL_SLOT_DONE
        assert res.data == bytes([0xAB, 0xCD])
        assert state["polls"] >= 2
    finally:
        client.stop()


def test_telemetry_dispatch_during_requests():
    received = []
    client, stream = _make_client({api.MSG_ESTOP: _ctrl_ok()})
    client.on_telemetry(lambda sid, rec, hdr: received.append((sid, rec)))
    try:
        # A health telemetry frame is dispatched without disturbing requests.
        # 12-byte health payload: uptime(4), state, fault, watchdog(4), batt(2).
        stream.push_telemetry(int(tlm_stream()), bytes(12))
        res = client.estop()
        assert res is not None and res.ok
        # Give the reader a moment to drain the telemetry frame.
        _wait_for(lambda: received, timeout=1.0)
        assert received and received[0][0] == int(tlm_stream())
    finally:
        client.stop()


def test_request_timeout_returns_none():
    # No handler writes a response for an unmapped-but-silent stream: simulate by
    # a handler that drops the frame (returns nothing to inbound).
    class SilentStream(RespondingStream):
        def _respond(self, frame: bytes) -> None:  # swallow, never answer
            return

    stream = SilentStream({})
    client = ProtocolClient(stream, response_timeout=0.1)
    client.start()
    try:
        assert client.estop() is None
    finally:
        client.stop()


# --- helpers --------------------------------------------------------------
def tlm_stream():
    from hexapod_protocol import telemetry as tlm

    return tlm.StreamId.HEALTH


def _wait_for(pred, timeout: float = 1.0) -> None:
    import time

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return
        time.sleep(0.005)
