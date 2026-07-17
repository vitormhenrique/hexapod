#!/usr/bin/env python3
"""Loopback simulated-firmware protocol for the ROS SIL graph.

The companion remains a normal binary-protocol client.  This module implements
only the high-level portion of that protocol and exposes its latest validated
motion intent to a ROS wrapper.  It intentionally has no ``rclpy`` dependency
so the wire contract can be tested without a ROS executor.
"""

from __future__ import annotations

import hmac
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Final

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import (
    DecodeError,
    Header,
    MsgType,
    VERSION_MAJOR,
    VERSION_MINOR,
    decode_frame_body,
    encode_frame,
)


SIMULATION_DEVICE_NAME: Final[str] = "HexNav ROS SIM"
DEFAULT_HOST: Final[str] = "127.0.0.1"
DEFAULT_PORT: Final[int] = 5560
DEFAULT_TOKEN: Final[str] = "hexapod-sim"
DEFAULT_VALID_FOR_MS: Final[int] = 250
_MAX_AUTH_LINE_BYTES: Final[int] = 512
_AUTH_PREFIX: Final[bytes] = b"HEXAPOD_RELAY/1 "
_UNAVAILABLE_REASON: Final[int] = api.FEATURE_REASON_NOT_IMPLEMENTED
_SUPPORTED_STREAMS: Final[frozenset[int]] = frozenset(
    (
        int(tlm.StreamId.HEALTH),
        int(tlm.StreamId.CONTROL_STATE),
        int(tlm.StreamId.RC_INPUT),
    )
)


@dataclass(frozen=True)
class SimulatedMotionCommand:
    """Latest safe high-level request expressed in ROS message units."""

    sequence: int = 0
    gait: int = api.GAIT_STAND
    body_height_m: float = 0.040
    stride_length_m: float = 0.060
    step_height_m: float = 0.030
    duty_factor: float = 128.0 / 255.0
    speed_scale: float = 128.0 / 255.0
    normalized_vx: float = 0.0
    normalized_vy: float = 0.0
    normalized_wz: float = 0.0
    body_x_m: float = 0.0
    body_y_m: float = 0.0
    body_z_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    valid_for_ms: int = DEFAULT_VALID_FOR_MS


@dataclass(frozen=True)
class SimulatedSafetyState:
    """SIL-only authority snapshot published by the ROS wrapper."""

    state: int
    armed: bool
    host_estop: bool


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SimulatedFirmware:
    """Subset of the public firmware protocol that is meaningful in SIL.

    Real hardware ownership is deliberately absent: DYNAMIXEL, maintenance,
    passive pose, I2C, and feature configuration commands are not emulated.
    The server accepts only high-level motion intent and mirrors the firmware's
    validation and clamping before publishing it to the ROS-side consumer.
    """

    def __init__(
        self,
        motion_sink: Callable[[SimulatedMotionCommand], None] | None = None,
        *,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self._motion_sink = motion_sink
        self._clock_ms = clock_ms or self._monotonic_ms
        self._lock = threading.RLock()
        self._motion = SimulatedMotionCommand()
        # Boot disarmed: ControllerCore requires an rc_armed false->true edge
        # before it will arm, so the client must arm explicitly (as the
        # companion Gait Lab does). Booting already-armed would race the
        # controller's first safety-input sample and deadlock it in Disarmed.
        self._state = int(tlm.SafetyState.DISARMED)
        self._subscriptions: dict[int, tuple[int, int, int]] = {}
        self._started_ms = self._clock_ms()

    @staticmethod
    def _monotonic_ms() -> int:
        return int(time.monotonic() * 1000)

    def uptime_ms(self) -> int:
        return max(0, self._clock_ms() - self._started_ms) & 0xFFFFFFFF

    def motion_command(self) -> SimulatedMotionCommand:
        with self._lock:
            return self._motion

    def safety_state(self) -> SimulatedSafetyState:
        with self._lock:
            state = self._state
        return SimulatedSafetyState(
            state=state,
            armed=state == int(tlm.SafetyState.RC_MANUAL),
            host_estop=state == int(tlm.SafetyState.ESTOP),
        )

    def motion_allowed(self) -> bool:
        return self.safety_state().armed

    def handle_wire(self, wire: bytes) -> bytes:
        """Decode one delimited command frame and return its response frame."""
        if len(wire) < 3 or wire[0] != 0 or wire[-1] != 0:
            raise DecodeError("frame missing 0x00 delimiters")
        header, payload = decode_frame_body(wire[1:-1])
        if header.msg_type != int(MsgType.COMMAND):
            return self._response(header, bytes([api.ERR_BAD_REQUEST]), api.FLAG_ERROR)

        response_payload, response_flags, publish = self._handle_command(
            header.msg_id, payload
        )
        if publish is not None and self._motion_sink is not None:
            self._motion_sink(publish)
        return self._response(header, response_payload, response_flags)

    def telemetry_frames(self) -> list[bytes]:
        """Return rate-limited telemetry frames for accepted subscriptions."""
        now = self.uptime_ms()
        frames: list[bytes] = []
        with self._lock:
            subscriptions = list(self._subscriptions.items())
            for stream_id, (rate_hz, last_sent_ms, emitted) in subscriptions:
                interval_ms = max(1, 1000 // max(1, rate_hz))
                if now - last_sent_ms < interval_ms:
                    continue
                payload = self._telemetry_payload(stream_id, now)
                if payload is None:
                    continue
                self._subscriptions[stream_id] = (rate_hz, now, emitted + 1)
                header = Header(
                    msg_type=int(MsgType.TELEMETRY),
                    msg_id=api.MSG_TELEMETRY_BASE + stream_id,
                    timestamp_ms=now,
                )
                frames.append(encode_frame(header, payload))
        return frames

    def _handle_command(
        self, msg_id: int, payload: bytes
    ) -> tuple[bytes, int, SimulatedMotionCommand | None]:
        if msg_id == api.MSG_HELLO:
            return self._hello_payload(), 0, None
        if msg_id in (api.MSG_HEARTBEAT, api.MSG_JETSON_HEARTBEAT):
            return struct.pack("<IB", self.uptime_ms(), self._state), 0, None
        if msg_id == api.MSG_GET_STATUS:
            return self._status_payload(), 0, None
        if msg_id == api.MSG_GET_CAPABILITIES:
            return self._capabilities_payload(), 0, None
        if msg_id in (api.MSG_SUBSCRIBE, api.MSG_SET_STREAM_RATE):
            return self._handle_subscription(payload), 0, None
        if msg_id == api.MSG_UNSUBSCRIBE:
            return self._handle_unsubscribe(payload), 0, None
        if msg_id == api.MSG_GET_STREAM_STATS:
            return self._stream_stats_payload(), 0, None
        if msg_id in (
            api.MSG_ESTOP,
            api.MSG_CLEAR_FAULT,
            api.MSG_SET_ARMING,
            api.MSG_SET_MODE,
        ):
            return self._handle_control(msg_id, payload), 0, None
        if api.MSG_SET_GAIT <= msg_id <= api.MSG_STOP_MOTION:
            return self._handle_motion(msg_id, payload)
        if msg_id == api.MSG_FEATURE_GET:
            return self._feature_list_payload(), 0, None
        if msg_id == api.MSG_FEATURE_GET_REASONS:
            return self._feature_reasons_payload(), 0, None
        if msg_id == api.MSG_FEATURE_RESET_DEFAULTS:
            return bytes([api.FEATURE_OK]) + self._feature_list_payload(), 0, None
        if msg_id == api.MSG_FEATURE_SET:
            return self._handle_feature_set(payload), 0, None
        return bytes([api.ERR_UNKNOWN_MSG]), api.FLAG_ERROR, None

    def _hello_payload(self) -> bytes:
        return bytes(
            [VERSION_MAJOR, VERSION_MINOR, 0, 1, 0]
        ) + self._device_name_bytes()

    def _capabilities_payload(self) -> bytes:
        return (
            bytes([VERSION_MAJOR, VERSION_MINOR, 0, 1, 0])
            + struct.pack("<I", 0)
            + self._device_name_bytes()
            + b"\x00"
        )

    @staticmethod
    def _device_name_bytes() -> bytes:
        return SIMULATION_DEVICE_NAME.encode("ascii").ljust(api.DEVICE_NAME_LEN, b"\x00")

    def _status_payload(self) -> bytes:
        state = self.safety_state()
        return struct.pack("<IBBHI", self.uptime_ms(), state.state, 0, 12000, 0)

    def _handle_subscription(self, payload: bytes) -> bytes:
        if len(payload) != 3:
            return bytes([api.SUB_BAD_REQUEST, 0, 0, 0])
        stream_id = payload[0]
        rate_hz = struct.unpack_from("<H", payload, 1)[0]
        if stream_id not in _SUPPORTED_STREAMS or rate_hz == 0:
            return struct.pack("<BBH", api.SUB_BAD_STREAM, stream_id, 0)
        with self._lock:
            previous = self._subscriptions.get(stream_id, (rate_hz, 0, 0))
            self._subscriptions[stream_id] = (rate_hz, 0, previous[2])
        return struct.pack("<BBH", api.SUB_OK, stream_id, rate_hz)

    def _handle_unsubscribe(self, payload: bytes) -> bytes:
        if len(payload) != 1:
            return bytes([api.SUB_BAD_REQUEST, 0, 0, 0])
        with self._lock:
            self._subscriptions.pop(payload[0], None)
        return struct.pack("<BBH", api.SUB_OK, payload[0], 0)

    def _stream_stats_payload(self) -> bytes:
        with self._lock:
            subscriptions = list(self._subscriptions.items())
        payload = bytearray(struct.pack("<BI", len(subscriptions), 0))
        for stream_id, (rate_hz, _last_sent, emitted) in subscriptions:
            payload.extend(struct.pack("<BBHII", stream_id, 1, rate_hz, emitted, 0))
        return bytes(payload)

    def _handle_control(self, msg_id: int, payload: bytes) -> bytes:
        with self._lock:
            result = api.CTRL_OK
            if msg_id == api.MSG_ESTOP:
                self._state = int(tlm.SafetyState.ESTOP)
            elif msg_id == api.MSG_CLEAR_FAULT:
                self._state = int(tlm.SafetyState.DISARMED)
            elif msg_id == api.MSG_SET_ARMING:
                if len(payload) != 1 or payload[0] not in (
                    api.ARMING_DISARM,
                    api.ARMING_ARM,
                ):
                    result = api.CTRL_BAD_REQUEST
                else:
                    self._state = (
                        int(tlm.SafetyState.RC_MANUAL)
                        if payload[0] == api.ARMING_ARM
                        else int(tlm.SafetyState.DISARMED)
                    )
            elif msg_id == api.MSG_SET_MODE:
                if len(payload) != 1:
                    result = api.CTRL_BAD_REQUEST
                elif payload[0] == int(tlm.SafetyState.DISARMED):
                    self._state = int(tlm.SafetyState.DISARMED)
                elif payload[0] == int(tlm.SafetyState.ESTOP):
                    self._state = int(tlm.SafetyState.ESTOP)
                else:
                    result = api.CTRL_REJECTED
            fault = 2 if self._state == int(tlm.SafetyState.ESTOP) else 0
            return bytes([result, self._state, fault])

    def _handle_motion(
        self, msg_id: int, payload: bytes
    ) -> tuple[bytes, int, SimulatedMotionCommand | None]:
        with self._lock:
            motion = self._motion
            result = api.MOTION_OK
            if msg_id == api.MSG_SET_GAIT:
                if len(payload) < 1:
                    result = api.MOTION_BAD_REQUEST
                elif payload[0] >= api.GAIT_CRAWL + 1:
                    result = api.MOTION_REJECTED
                else:
                    motion = replace(motion, gait=payload[0])
            elif msg_id == api.MSG_SET_GAIT_PARAMS:
                if len(payload) < 8:
                    result = api.MOTION_BAD_REQUEST
                else:
                    height, stride, step, duty, speed = struct.unpack_from(
                        "<HHHBB", payload
                    )
                    motion = replace(
                        motion,
                        body_height_m=min(height, 120) / 1000.0,
                        stride_length_m=min(stride, 80) / 1000.0,
                        step_height_m=min(step, 50) / 1000.0,
                        duty_factor=duty / 255.0,
                        speed_scale=speed / 255.0,
                    )
            elif msg_id == api.MSG_SET_BODY_TWIST:
                if len(payload) < 6:
                    result = api.MOTION_BAD_REQUEST
                else:
                    vx, vy, wz = struct.unpack_from("<hhh", payload)
                    motion = replace(
                        motion,
                        normalized_vx=_clamp(vx / 1000.0, -1.0, 1.0),
                        normalized_vy=_clamp(vy / 1000.0, -1.0, 1.0),
                        normalized_wz=_clamp(wz / 1000.0, -1.0, 1.0),
                    )
            elif msg_id == api.MSG_SET_BODY_POSE:
                if len(payload) < 12:
                    result = api.MOTION_BAD_REQUEST
                else:
                    x, y, z, roll, pitch, yaw = struct.unpack_from("<hhhhhh", payload)
                    motion = replace(
                        motion,
                        body_x_m=_clamp(float(x), -50.0, 50.0) / 1000.0,
                        body_y_m=_clamp(float(y), -50.0, 50.0) / 1000.0,
                        body_z_m=_clamp(float(z), -50.0, 50.0) / 1000.0,
                        roll_rad=_clamp(
                            math.radians(roll / 1000.0), -0.4363, 0.4363
                        ),
                        pitch_rad=_clamp(
                            math.radians(pitch / 1000.0), -0.4363, 0.4363
                        ),
                        yaw_rad=_clamp(
                            math.radians(yaw / 1000.0), -0.4363, 0.4363
                        ),
                    )
            elif msg_id == api.MSG_STOP_MOTION:
                motion = replace(
                    motion,
                    gait=api.GAIT_STAND,
                    normalized_vx=0.0,
                    normalized_vy=0.0,
                    normalized_wz=0.0,
                )

            if result != api.MOTION_OK:
                flags = api.FLAG_ERROR if result == api.MOTION_BAD_REQUEST else 0
                return (
                    bytes([result, self._state, int(self.motion_allowed())]),
                    flags,
                    None,
                )
            motion = replace(motion, sequence=(motion.sequence + 1) & 0xFFFFFFFF)
            self._motion = motion
            return bytes([result, self._state, int(self.motion_allowed())]), 0, motion

    def _feature_list_payload(self) -> bytes:
        payload = bytearray([self.safety_state().state, api.FEATURE_COUNT])
        for feature in range(api.FEATURE_COUNT):
            payload.extend(bytes([feature, 0, 0, _UNAVAILABLE_REASON]))
        return bytes(payload)

    def _feature_reasons_payload(self) -> bytes:
        payload = bytearray([self.safety_state().state, api.FEATURE_COUNT])
        for feature in range(api.FEATURE_COUNT):
            payload.extend(bytes([feature, _UNAVAILABLE_REASON]))
        return bytes(payload)

    def _handle_feature_set(self, payload: bytes) -> bytes:
        if len(payload) != 2 or payload[0] >= api.FEATURE_COUNT:
            return bytes([api.FEATURE_BAD_REQUEST, self.safety_state().state, 0, 0, 0, 0])
        feature, enable = payload
        result = api.FEATURE_REJECTED if enable else api.FEATURE_OK
        return bytes(
            [result, self.safety_state().state, feature, 0, 0, _UNAVAILABLE_REASON]
        )

    def _telemetry_payload(self, stream_id: int, uptime_ms: int) -> bytes | None:
        state = self.safety_state()
        if stream_id == int(tlm.StreamId.HEALTH):
            return struct.pack("<IBBIH", uptime_ms, state.state, 0, 0, 12000)
        if stream_id == int(tlm.StreamId.CONTROL_STATE):
            return bytes(
                [
                    int(tlm.CommandSource.RC),
                    int(state.armed),
                    int(state.host_estop),
                    state.state,
                    2 if state.host_estop else 0,
                    int(state.armed),
                ]
            )
        if stream_id == int(tlm.StreamId.RC_INPUT):
            flags = (0x01 if state.armed else 0) | (0x02 if state.host_estop else 0)
            return bytes([flags, self.motion_command().gait]) + struct.pack(
                "<16H", *([1500] * 16)
            )
        return None

    def _response(self, request: Header, payload: bytes, flags: int) -> bytes:
        response = Header(
            msg_type=int(MsgType.RESPONSE),
            msg_id=request.msg_id,
            flags=flags,
            seq=request.seq,
            timestamp_ms=self.uptime_ms(),
        )
        return encode_frame(response, payload)


class SimulatedFirmwareServer:
    """One-client, loopback-only TCP server for :class:`SimulatedFirmware`."""

    def __init__(
        self,
        firmware: SimulatedFirmware,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        token: str = DEFAULT_TOKEN,
    ) -> None:
        if host not in ("127.0.0.1", "::1", "localhost"):
            raise ValueError("the simulated firmware must bind to loopback only")
        if not token or len(token) > 256 or any(char.isspace() for char in token):
            raise ValueError("token must be a non-empty single token")
        self._firmware = firmware
        self._host = host
        self._port = port
        self._token = token.encode("ascii")
        self._listener: socket.socket | None = None
        self._connection: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connection_lock = threading.Lock()

    @property
    def port(self) -> int:
        listener = self._listener
        if listener is None:
            return self._port
        return int(listener.getsockname()[1])

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self._host, self._port))
        listener.listen(1)
        listener.settimeout(0.1)
        self._listener = listener
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._serve, name="hexapod-sim-firmware", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._connection_lock:
            connection = self._connection
            self._connection = None
        for endpoint in (connection, self._listener):
            if endpoint is None:
                continue
            try:
                endpoint.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            endpoint.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._listener = None
        self._thread = None

    def _serve(self) -> None:
        listener = self._listener
        if listener is None:
            return
        while not self._stop.is_set():
            try:
                connection, _address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._connection_lock:
                self._connection = connection
            try:
                self._serve_connection(connection)
            finally:
                with self._connection_lock:
                    self._connection = None
                try:
                    connection.close()
                except OSError:
                    pass

    def _serve_connection(self, connection: socket.socket) -> None:
        try:
            line, buffer = self._receive_auth_line(connection)
            if not line.startswith(_AUTH_PREFIX) or not hmac.compare_digest(
                line[len(_AUTH_PREFIX) :], self._token
            ):
                connection.sendall(b"UNAUTHORIZED\n")
                return
            connection.sendall(b"OK\n")
            connection.settimeout(0.05)
            while not self._stop.is_set():
                for frame in self._extract_frames(buffer):
                    try:
                        connection.sendall(self._firmware.handle_wire(frame))
                    except DecodeError:
                        continue
                for frame in self._firmware.telemetry_frames():
                    connection.sendall(frame)
                try:
                    chunk = connection.recv(512)
                except socket.timeout:
                    continue
                if not chunk:
                    return
                buffer.extend(chunk)
        except (ConnectionError, OSError):
            return

    @staticmethod
    def _receive_auth_line(connection: socket.socket) -> tuple[bytes, bytearray]:
        connection.settimeout(1.0)
        buffer = bytearray()
        while True:
            newline = buffer.find(b"\n")
            if newline >= 0:
                return bytes(buffer[:newline]), bytearray(buffer[newline + 1 :])
            if len(buffer) >= _MAX_AUTH_LINE_BYTES:
                raise ConnectionError("authentication preamble is too long")
            chunk = connection.recv(min(256, _MAX_AUTH_LINE_BYTES - len(buffer)))
            if not chunk:
                raise ConnectionError("connection closed during authentication")
            buffer.extend(chunk)

    @staticmethod
    def _extract_frames(buffer: bytearray) -> list[bytes]:
        frames: list[bytes] = []
        while True:
            start = buffer.find(b"\x00")
            if start < 0:
                buffer.clear()
                return frames
            if start > 0:
                del buffer[:start]
            end = buffer.find(b"\x00", 1)
            if end < 0:
                return frames
            frames.append(bytes(buffer[: end + 1]))
            del buffer[: end + 1]
