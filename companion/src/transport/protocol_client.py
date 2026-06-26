"""Typed, threaded protocol client.

Wraps a :class:`ByteStream` with a background reader thread. Commands are sent
synchronously (``request``) with seq-matched response correlation; telemetry
frames are dispatched to registered callbacks. UI-independent: the CLI uses it
directly and the PySide6 app wraps it in a Qt service.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Callable, Optional

from hexapod_protocol import api
from hexapod_protocol.framing import (
    Header,
    MsgType,
    DecodeError,
    decode_frame_body,
)
from hexapod_protocol import telemetry as tlm

from . import ByteStream, FrameExtractor


@dataclass
class Response:
    header: Header
    payload: bytes

    @property
    def is_error(self) -> bool:
        return api.is_error(self.header)


TelemetryCallback = Callable[[int, object, Header], None]
"""Called as ``cb(stream_id, decoded_record, header)`` for each telemetry frame."""

ConnectionCallback = Callable[[bool], None]


class ProtocolClient:
    """Send commands and receive responses / telemetry over a byte stream."""

    def __init__(self, stream: ByteStream, response_timeout: float = 1.0) -> None:
        self._stream = stream
        self._extractor = FrameExtractor()
        self._timeout = response_timeout
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._write_lock = threading.Lock()

        # seq -> single-slot queue for the matching response.
        self._pending: dict[int, "Queue[Response]"] = {}
        self._pending_lock = threading.Lock()

        self._telemetry_cbs: list[TelemetryCallback] = []
        self._connection_cbs: list[ConnectionCallback] = []

        self._reader: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._connected = False

        # Lightweight stats for the diagnostics page.
        self.rx_frames = 0
        self.tx_frames = 0
        self.decode_errors = 0

    # --- lifecycle --------------------------------------------------------

    def start(self) -> None:
        if self._reader and self._reader.is_alive():
            return
        self._running.set()
        self._reader = threading.Thread(
            target=self._read_loop, name="hexapod-reader", daemon=True
        )
        self._reader.start()
        self._set_connected(True)

    def stop(self) -> None:
        self._running.clear()
        if self._reader:
            self._reader.join(timeout=1.0)
        self._set_connected(False)
        try:
            self._stream.close()
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._connected

    # --- callbacks --------------------------------------------------------

    def on_telemetry(self, cb: TelemetryCallback) -> None:
        self._telemetry_cbs.append(cb)

    def on_connection(self, cb: ConnectionCallback) -> None:
        self._connection_cbs.append(cb)

    def _set_connected(self, value: bool) -> None:
        if value == self._connected:
            return
        self._connected = value
        for cb in list(self._connection_cbs):
            try:
                cb(value)
            except Exception:
                pass

    # --- sending ----------------------------------------------------------

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFF
            if self._seq == 0:
                self._seq = 1
            return self._seq

    def request(self, msg_id: int, payload: bytes = b"") -> Optional[Response]:
        """Send a command and wait for the seq-matched response.

        Returns ``None`` on timeout. Thread-safe.
        """
        seq = self._next_seq()
        slot: "Queue[Response]" = Queue(maxsize=1)
        with self._pending_lock:
            self._pending[seq] = slot
        try:
            frame = api.build_command(msg_id, seq=seq, payload=payload)
            self._write(frame)
            return slot.get(timeout=self._timeout)
        except Empty:
            return None
        finally:
            with self._pending_lock:
                self._pending.pop(seq, None)

    def send(self, msg_id: int, payload: bytes = b"") -> None:
        """Fire-and-forget command (no response wait)."""
        seq = self._next_seq()
        self._write(api.build_command(msg_id, seq=seq, payload=payload))

    def _write(self, frame: bytes) -> None:
        with self._write_lock:
            self._stream.write(frame)
            self.tx_frames += 1

    # --- high-level commands ---------------------------------------------

    def hello(self) -> Optional[api.HelloInfo]:
        r = self.request(api.MSG_HELLO)
        return api.parse_hello(r.payload) if r else None

    def get_status(self) -> Optional[api.StatusInfo]:
        r = self.request(api.MSG_GET_STATUS)
        return api.parse_status(r.payload) if r else None

    def get_capabilities(self) -> Optional[api.Capabilities]:
        r = self.request(api.MSG_GET_CAPABILITIES)
        return api.parse_capabilities(r.payload) if r else None

    def subscribe(self, stream_id: int, rate_hz: int) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_SUBSCRIBE, struct_subscribe(stream_id, rate_hz))
        return api.parse_subscribe_result(r.payload) if r else None

    def set_stream_rate(
        self, stream_id: int, rate_hz: int
    ) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_SET_STREAM_RATE, struct_subscribe(stream_id, rate_hz))
        return api.parse_subscribe_result(r.payload) if r else None

    def unsubscribe(self, stream_id: int) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_UNSUBSCRIBE, bytes([stream_id]))
        return api.parse_subscribe_result(r.payload) if r else None

    def get_stream_stats(self) -> Optional[api.StreamStats]:
        r = self.request(api.MSG_GET_STREAM_STATS)
        return api.parse_stream_stats(r.payload) if r else None

    # --- typed control-group commands ------------------------------------
    #
    # Each method reuses the golden ``api.build_*`` frame builders as the single
    # source of truth for the wire payload, then re-sends the payload through
    # ``request`` so the client owns seq allocation and response correlation.

    def _send_built(self, frame: bytes) -> Optional[Response]:
        """Re-send a pre-built command frame under a client-managed seq."""
        header, payload = decode_frame_body(frame[1:-1])
        return self.request(header.msg_id, payload)

    # Safety control.
    def estop(self) -> Optional[api.ControlResult]:
        r = self._send_built(api.build_estop())
        return api.parse_control_result(r.payload) if r else None

    def clear_fault(self) -> Optional[api.ControlResult]:
        r = self._send_built(api.build_clear_fault())
        return api.parse_control_result(r.payload) if r else None

    def set_arming(self, arm: bool) -> Optional[api.ControlResult]:
        r = self._send_built(api.build_set_arming(arm))
        return api.parse_control_result(r.payload) if r else None

    def set_mode(self, mode: int) -> Optional[api.ControlResult]:
        r = self._send_built(api.build_set_mode(mode))
        return api.parse_control_result(r.payload) if r else None

    # Motion.
    def set_gait(self, gait: int) -> Optional[api.MotionResultMsg]:
        r = self._send_built(api.build_set_gait(gait))
        return api.parse_motion_result(r.payload) if r else None

    def set_gait_params(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> Optional[api.MotionResultMsg]:
        r = self._send_built(
            api.build_set_gait_params(
                body_height_mm,
                stride_len_mm,
                step_height_mm,
                duty_x255,
                speed_x255,
            )
        )
        return api.parse_motion_result(r.payload) if r else None

    def set_body_twist(
        self, vx: float, vy: float, wz: float
    ) -> Optional[api.MotionResultMsg]:
        r = self._send_built(api.build_set_body_twist(vx, vy, wz))
        return api.parse_motion_result(r.payload) if r else None

    def set_body_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Optional[api.MotionResultMsg]:
        r = self._send_built(
            api.build_set_body_pose(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)
        )
        return api.parse_motion_result(r.payload) if r else None

    def stop_motion(self) -> Optional[api.MotionResultMsg]:
        r = self._send_built(api.build_stop_motion())
        return api.parse_motion_result(r.payload) if r else None

    # Feature flags.
    def feature_get(self) -> Optional[api.FeatureList]:
        r = self._send_built(api.build_feature_get())
        return api.parse_feature_list(r.payload) if r else None

    def feature_set(self, feature: int, enable: bool) -> Optional[api.FeatureSetResult]:
        r = self._send_built(api.build_feature_set(feature, enable))
        return api.parse_feature_set_result(r.payload) if r else None

    def feature_get_reasons(self) -> Optional[api.FeatureReasons]:
        r = self._send_built(api.build_feature_get_reasons())
        return api.parse_feature_reasons(r.payload) if r else None

    def feature_reset_defaults(self) -> Optional[api.FeatureList]:
        r = self._send_built(api.build_feature_reset_defaults())
        return api.parse_feature_list(r.payload) if r else None

    # Contact / leveling.
    def contact_enable(self, enable: bool) -> Optional[api.SensorFeatureResult]:
        frame = api.build_contact_enable() if enable else api.build_contact_disable()
        r = self._send_built(frame)
        return api.parse_sensor_feature_result(r.payload) if r else None

    def leveling_enable(self, enable: bool) -> Optional[api.SensorFeatureResult]:
        frame = api.build_leveling_enable() if enable else api.build_leveling_disable()
        r = self._send_built(frame)
        return api.parse_sensor_feature_result(r.payload) if r else None

    def contact_set_thresholds(
        self, foot: int, near: int, touch: int, load: int
    ) -> Optional[api.ContactThresholdResult]:
        r = self._send_built(
            api.build_contact_set_thresholds(foot, near, touch, load)
        )
        return api.parse_contact_threshold_result(r.payload) if r else None

    def leveling_set_params(
        self, max_tilt_mdeg: int, rate_mdeg_s: int, response_x255: int
    ) -> Optional[api.LevelingParamsResult]:
        r = self._send_built(
            api.build_leveling_set_params(max_tilt_mdeg, rate_mdeg_s, response_x255)
        )
        return api.parse_leveling_params_result(r.payload) if r else None

    def contact_calibrate(
        self, foot: int = api.SENSOR_CALIBRATE_ALL
    ) -> Optional[api.SensorCalibrateResult]:
        r = self._send_built(api.build_contact_calibrate(foot))
        return api.parse_sensor_calibrate_result(r.payload) if r else None

    def sensor_calibrate(self) -> Optional[api.SensorCalibrateResult]:
        r = self._send_built(api.build_sensor_calibrate())
        return api.parse_sensor_calibrate_result(r.payload) if r else None

    def sensor_set_rate(self, rate_hz: int) -> Optional[api.SensorRateResult]:
        r = self._send_built(api.build_sensor_set_rate(rate_hz))
        return api.parse_sensor_rate_result(r.payload) if r else None

    # Passive pose streaming.
    def passive_enter(self) -> Optional[api.PassiveResult]:
        r = self._send_built(api.build_passive_enter())
        return api.parse_passive_result(r.payload) if r else None

    def passive_exit(self) -> Optional[api.PassiveResult]:
        r = self._send_built(api.build_passive_exit())
        return api.parse_passive_result(r.payload) if r else None

    def passive_set_stream_rate(self, rate_hz: int) -> Optional[api.PassiveRateResult]:
        r = self._send_built(api.build_passive_set_stream_rate(rate_hz))
        return api.parse_passive_rate_result(r.payload) if r else None

    def passive_zero_reference(self) -> Optional[api.PassiveResult]:
        r = self._send_built(api.build_passive_zero_reference())
        return api.parse_passive_result(r.payload) if r else None

    # Maintenance lock.
    def enter_maintenance(self) -> Optional[api.MaintResultMsg]:
        r = self._send_built(api.build_enter_maintenance())
        return api.parse_maint_result(r.payload) if r else None

    def exit_maintenance(self, token: int) -> Optional[api.MaintResultMsg]:
        r = self._send_built(api.build_exit_maintenance(token))
        return api.parse_maint_result(r.payload) if r else None

    # --- DXL async job helpers -------------------------------------------
    #
    # DXL maintenance runs on a single-slot firmware job queue: submit a job,
    # then poll GET_RESULT until the slot is DONE. ``dxl_run`` wraps that loop.

    def dxl_submit(self, frame: bytes) -> Optional[api.DxlSubmitResult]:
        r = self._send_built(frame)
        return api.parse_dxl_submit(r.payload) if r else None

    def dxl_get_result(self, job_id: int) -> Optional[api.DxlJobResult]:
        r = self._send_built(api.build_dxl_get_result(job_id))
        return api.parse_dxl_result(r.payload) if r else None

    def dxl_run(
        self, frame: bytes, poll_interval: float = 0.02, timeout: float = 2.0
    ) -> Optional[api.DxlJobResult]:
        """Submit a DXL job frame and poll until it completes or times out."""
        sub = self.dxl_submit(frame)
        if sub is None or not sub.accepted:
            return None
        deadline = time.monotonic() + timeout
        result: Optional[api.DxlJobResult] = None
        while time.monotonic() < deadline:
            result = self.dxl_get_result(sub.job_id)
            if result is not None and result.slot == api.DXL_SLOT_DONE:
                return result
            time.sleep(poll_interval)
        return result

    def dxl_get_param(self, servo_id: int, param: int) -> Optional[api.DxlJobResult]:
        return self.dxl_run(api.build_dxl_get_param(servo_id, param))

    def dxl_set_param(
        self, servo_id: int, param: int, value: int
    ) -> Optional[api.DxlJobResult]:
        return self.dxl_run(api.build_dxl_set_param(servo_id, param, value))

    def dxl_set_servo_limits(
        self, servo_id: int, min_tick: int, max_tick: int
    ) -> Optional[api.DxlJobResult]:
        return self.dxl_run(
            api.build_dxl_set_servo_limits(servo_id, min_tick, max_tick)
        )

    def dxl_get_servo_profile(self, servo_id: int) -> Optional[api.DxlJobResult]:
        return self.dxl_run(api.build_dxl_get_servo_profile(servo_id))

    def dxl_read_register(
        self, servo_id: int, address: int, length: int
    ) -> Optional[api.DxlJobResult]:
        return self.dxl_run(api.build_dxl_read_register(servo_id, address, length))

    def dxl_write_register(
        self,
        servo_id: int,
        address: int,
        length: int,
        value: int,
        is_eeprom: bool = False,
    ) -> Optional[api.DxlJobResult]:
        return self.dxl_run(
            api.build_dxl_write_register(
                servo_id, address, length, value, is_eeprom=is_eeprom
            )
        )

    # --- reader loop ------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running.is_set():
            try:
                waiting = getattr(self._stream, "in_waiting", 0) or 0
                chunk = self._stream.read(max(1, min(waiting, 512)))
            except Exception:
                self._set_connected(False)
                break
            if not chunk:
                time.sleep(0.005)
                continue
            for frame in self._extractor.push(chunk):
                self._handle_frame(frame)

    def _handle_frame(self, frame: bytes) -> None:
        if len(frame) < 2 or frame[0] != 0x00 or frame[-1] != 0x00:
            return
        try:
            header, payload = decode_frame_body(frame[1:-1])
        except DecodeError:
            self.decode_errors += 1
            return
        self.rx_frames += 1

        if header.msg_type == int(MsgType.TELEMETRY):
            self._dispatch_telemetry(header, payload)
            return

        # Response/event: route to a waiting request by seq.
        with self._pending_lock:
            slot = self._pending.get(header.seq)
        if slot is not None:
            try:
                slot.put_nowait(Response(header, payload))
            except Exception:
                pass

    def _dispatch_telemetry(self, header: Header, payload: bytes) -> None:
        stream_id = header.msg_id - api.MSG_TELEMETRY_BASE
        record = tlm.decode_stream(stream_id, payload)
        if record is None:
            return
        for cb in list(self._telemetry_cbs):
            try:
                cb(stream_id, record, header)
            except Exception:
                pass


def struct_subscribe(stream_id: int, rate_hz: int) -> bytes:
    import struct

    return struct.pack("<BH", stream_id & 0xFF, rate_hz & 0xFFFF)
