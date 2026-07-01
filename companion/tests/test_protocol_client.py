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


# --- EEPROM config round-trip ---------------------------------------------
class _FakeConfigFirmware:
    """Stateful CFG_* handler set backed by a serialized RobotConfig payload.

    Serves CFG_GET_SUMMARY / CFG_GET_BLOCK from ``payload``, accepts windowed
    CFG_SET_BLOCK writes into a staging buffer, and answers validate/commit/reset.
    """

    def __init__(self) -> None:
        import struct

        from hexapod_protocol import config as cfg

        self._cfg = cfg
        self._struct = struct
        self.config = cfg.default_robot_config()
        self.payload = bytearray(cfg.encode_robot_config(self.config))
        self.staged = bytearray(len(self.payload))
        self.block_max = cfg.CFG_BLOCK_MAX
        self.committed = False

    def handlers(self) -> dict:
        return {
            api.MSG_CFG_GET_SUMMARY: self._summary,
            api.MSG_CFG_GET_BLOCK: self._get_block,
            api.MSG_CFG_SET_BLOCK: self._set_block,
            api.MSG_CFG_VALIDATE: lambda _p: (bytes([api.CFG_OK]), False),
            api.MSG_CFG_COMMIT: self._commit,
            api.MSG_CFG_RESET_DEFAULTS: lambda _p: (bytes([api.CFG_OK]), False),
        }

    def _summary(self, _payload):
        s = self._struct
        out = s.pack("<HHH", self.config.schema_version, len(self.payload),
                     self.block_max)
        out += bytes([0x03])  # persistent + staged valid
        out += s.pack("<I", self.config.feature_defaults)
        name = self.config.robot_name.encode("utf-8")[:16]
        out += name + bytes(16 - len(name))
        return out, False

    def _get_block(self, payload):
        s = self._struct
        offset, length = s.unpack("<HH", payload[:4])
        data = bytes(self.payload[offset : offset + length])
        return s.pack("<HH", offset, len(data)) + data, False

    def _set_block(self, payload):
        s = self._struct
        offset, length = s.unpack("<HH", payload[:4])
        data = payload[4 : 4 + length]
        self.staged[offset : offset + length] = data
        return s.pack("<HH", offset, len(data)), False

    def _commit(self, _payload):
        self.committed = True
        self.payload = bytearray(self.staged)
        return bytes([api.CFG_OK]), False


def test_read_config_assembles_full_payload():
    fw = _FakeConfigFirmware()
    client, _ = _make_client(fw.handlers())
    try:
        cfg = client.read_config()
        assert cfg is not None
        assert cfg.robot_name == fw.config.robot_name
        assert len(cfg.servos) == len(fw.config.servos)
        assert cfg.servos[0].id == fw.config.servos[0].id
    finally:
        client.stop()


def test_write_config_stages_every_block():
    from hexapod_protocol import config as cfg_mod

    fw = _FakeConfigFirmware()
    client, _ = _make_client(fw.handlers())
    try:
        edited = client.read_config()
        assert edited is not None
        edited.robot_name = "Tweaked"
        edited.servos[0].trim_ticks = 17
        assert client.write_config(edited) is True
        # The staged buffer must now decode back to the edited config.
        staged = cfg_mod.decode_robot_config(bytes(fw.staged))
        assert staged.robot_name == "Tweaked"
        assert staged.servos[0].trim_ticks == 17
    finally:
        client.stop()


def test_cfg_validate_commit_reset():
    fw = _FakeConfigFirmware()
    client, _ = _make_client(fw.handlers())
    try:
        assert client.cfg_validate().ok
        assert client.cfg_commit().ok
        assert fw.committed is True
        assert client.cfg_reset_defaults().ok
    finally:
        client.stop()



# --- diagnostics: raw-frame capture + counters ----------------------------
def test_raw_capture_records_telemetry_frame():
    from hexapod_protocol import telemetry as tlm

    client, stream = _make_client({})
    try:
        client.set_raw_capture(True)
        # Health payload: uptime(u32)=1000, state=2, fault=0, wd(u32)=0, batt(u16)=12000
        import struct

        payload = struct.pack("<IBBIH", 1000, 2, 0, 0, 12000)
        stream.push_telemetry(int(tlm.StreamId.HEALTH), payload)
        _wait_for(lambda: client.rx_frames >= 1)
        frames = client.drain_raw_frames()
        assert len(frames) == 1
        rec = frames[0]
        assert rec.ok and rec.msg_type == int(MsgType.TELEMETRY)
        assert rec.payload_len == len(payload)
        # Draining clears the buffer.
        assert client.drain_raw_frames() == []
    finally:
        client.stop()


def test_raw_capture_disabled_by_default():
    from hexapod_protocol import telemetry as tlm

    client, stream = _make_client({})
    try:
        import struct

        payload = struct.pack("<IBBIH", 1, 2, 0, 0, 12000)
        stream.push_telemetry(int(tlm.StreamId.HEALTH), payload)
        _wait_for(lambda: client.rx_frames >= 1)
        assert client.drain_raw_frames() == []
    finally:
        client.stop()


def test_set_raw_capture_false_clears_buffer():
    from hexapod_protocol import telemetry as tlm

    client, stream = _make_client({})
    try:
        import struct

        client.set_raw_capture(True)
        payload = struct.pack("<IBBIH", 1, 2, 0, 0, 12000)
        stream.push_telemetry(int(tlm.StreamId.HEALTH), payload)
        _wait_for(lambda: client.rx_frames >= 1)
        client.set_raw_capture(False)
        assert client.drain_raw_frames() == []
    finally:
        client.stop()


def test_diagnostics_snapshot_counts_frames():
    client, _ = _make_client({api.MSG_ESTOP: _ctrl_ok(state=12)})
    try:
        client.estop()
        _wait_for(lambda: client.rx_frames >= 1)
        snap = client.diagnostics_snapshot()
        assert snap.tx_frames >= 1
        assert snap.rx_frames >= 1
        assert snap.decode_errors == 0
        assert snap.capture_enabled is False
    finally:
        client.stop()


# --- I2C topology / sensor status / poll rate -----------------------------
def _topology_payload() -> bytes:
    import struct

    # mux=1, eeprom=1, n=3 channels; ch0 present, ch1 missing, ch2 fault.
    body = bytes([1, 1, 3])
    body += bytes([1, 1, 1, 2, 1])  # scanned,vcnl,lps,count,state=present
    body += bytes([1, 0, 0, 0, 0])  # missing
    body += bytes([1, 1, 0, 1, 2])  # fault
    return body


def _sensor_status_payload() -> bytes:
    import struct

    # n=2 feet, present_mask=0b01, polling=1.
    body = bytes([2, 0b01, 1])
    body += struct.pack("<BBHhB", 3, 200, 1234, 40, 0x04)  # LOADED
    body += struct.pack("<BBHhB", 0, 10, 5, 0, 0x00)  # AIR
    return body


def test_i2c_get_topology_decodes_channels():
    client, _ = _make_client(
        {api.MSG_I2C_GET_TOPOLOGY: lambda _p: (_topology_payload(), False)}
    )
    try:
        topo = client.i2c_get_topology()
        assert topo is not None
        assert topo.mux_present and topo.eeprom_present
        assert len(topo.channels) == 3
        assert topo.channels[0].state == 1
        assert topo.channels[2].state == 2
    finally:
        client.stop()


def test_i2c_scan_returns_result():
    import struct

    client, _ = _make_client(
        {api.MSG_I2C_SCAN: lambda _p: (bytes([api.SENSOR_OK]) + struct.pack("<H", 7), False)}
    )
    try:
        res = client.i2c_scan()
        assert res is not None and res.ok and res.scan_seq == 7
    finally:
        client.stop()


def test_sensor_get_status_decodes_feet():
    client, _ = _make_client(
        {api.MSG_SENSOR_GET_STATUS: lambda _p: (_sensor_status_payload(), False)}
    )
    try:
        st = client.sensor_get_status()
        assert st is not None
        assert st.present_mask == 0b01 and st.polling_enabled
        assert len(st.feet) == 2
        assert st.feet[0].loaded and st.feet[0].proximity == 1234
    finally:
        client.stop()


def test_sensor_set_rate_roundtrip():
    import struct

    seen = {}

    def handler(payload):
        seen["rate"] = struct.unpack_from("<H", payload, 0)[0]
        return bytes([api.SENSOR_OK]) + struct.pack("<H", seen["rate"]), False

    client, _ = _make_client({api.MSG_SENSOR_SET_RATE: handler})
    try:
        res = client.sensor_set_rate(75)
        assert res is not None and res.ok and res.rate_hz == 75
        assert seen["rate"] == 75
    finally:
        client.stop()
