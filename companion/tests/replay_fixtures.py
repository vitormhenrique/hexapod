"""Hardware-free replay fixtures for UI smoke tests.

Builds a small but representative recorded session on disk (via
:class:`data.session_logger.SessionLogger`) whose raw frames decode back through
the protocol stack, so any page can be driven exactly like a live link without a
serial port. See AGENTS.md 7.5 (session layout) and nzi.8.
"""

from __future__ import annotations

import struct
from pathlib import Path

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import Header, MsgType, encode_frame

NUM_FEET = tlm.NUM_FEET
NUM_CHANNELS = tlm.NUM_CHANNELS
NUM_STREAMS = tlm.NUM_STREAMS


def _frame(stream_id: int, payload: bytes, ts_ms: int) -> bytes:
    header = Header(
        msg_type=int(MsgType.TELEMETRY),
        msg_id=api.MSG_TELEMETRY_BASE + stream_id,
        timestamp_ms=ts_ms,
    )
    return encode_frame(header, payload)


# --- per-stream payload builders (mirror telemetry.decode_* layouts) -------


def health_payload(uptime_ms: int = 12345, batt_mv: int = 11800) -> bytes:
    # uptime(u32), state(u8), fault(u8), watchdog(u32), battery(u16)
    return struct.pack("<IBBIH", uptime_ms, 5, 0, 0, batt_mv)


def control_state_payload() -> bytes:
    # source, motion_authorized, kill, state, fault_reason, motion_gate
    return bytes([1, 1, 0, 5, 0, 1])


def servo_status_payload(count: int = 18) -> bytes:
    body = bytes([count])
    for i in range(count):
        body += struct.pack(
            "<BIhhHbBB",
            i + 1,  # id
            2048 + i,  # position
            0,  # velocity
            0,  # load
            12000,  # voltage_mv
            30,  # temperature_c
            0,  # hardware_error
            1,  # torque_enabled
        )
    return body


def contact_state_payload() -> bytes:
    body = b""
    for i in range(NUM_FEET):
        state = 3 if i % 2 == 0 else 0  # LOADED / AIR
        body += struct.pack("<BBh", state, 180, 40)
    return body


def i2c_sensors_raw_payload() -> bytes:
    body = b""
    for i in range(NUM_FEET):
        body += struct.pack("<Hi", 100 + i, 200 + i)
    return body


def rc_input_payload() -> bytes:
    body = bytes([0x01, 2])  # flags: armed; gait_index 2
    for _ in range(NUM_CHANNELS):
        body += struct.pack("<H", 1500)
    return body


def api_stats_payload() -> bytes:
    body = struct.pack("<I", 0)  # tx_backlog
    for _ in range(NUM_STREAMS):
        body += struct.pack("<I", 0)
    return body


def joint_state_payload() -> bytes:
    joints = [(leg, joint) for leg in range(NUM_FEET) for joint in range(3)]
    body = bytes([len(joints)])
    for leg, joint in joints:
        body += struct.pack("<BBh", leg, joint, 500)
    return body


# Ordered so timestamps increase; covers every stream the pages consume.
_STREAM_BUILDERS = [
    (tlm.StreamId.HEALTH, health_payload),
    (tlm.StreamId.CONTROL_STATE, control_state_payload),
    (tlm.StreamId.SERVO_STATUS, servo_status_payload),
    (tlm.StreamId.CONTACT_STATE, contact_state_payload),
    (tlm.StreamId.I2C_SENSORS_RAW, i2c_sensors_raw_payload),
    (tlm.StreamId.RC_INPUT, rc_input_payload),
    (tlm.StreamId.API_STATS, api_stats_payload),
    (tlm.StreamId.JOINT_STATE, joint_state_payload),
]


def build_sample_session(out_dir: Path, frames_per_stream: int = 3):
    """Write a recorded session covering every telemetry stream.

    Returns a :class:`data.session_replay.SessionReplay` over the new directory.
    """
    from data.session_logger import SessionLogger
    from data.session_replay import SessionReplay

    logger = SessionLogger(
        out_dir=out_dir,
        robot_name="fixture",
        firmware={"device_name": "hexapod", "fw": "0.2.0", "proto": "1.0"},
    )
    ts = 0
    for _ in range(frames_per_stream):
        for stream_id, builder in _STREAM_BUILDERS:
            payload = builder()
            ts += 10
            logger.log_raw_frame(_frame(int(stream_id), payload, ts))
            logger.log_record(tlm.STREAM_NAMES[stream_id], None, robot_time_ms=ts)
    logger.mark_event("connect", "fixture session")
    logger.close()
    return SessionReplay(logger.dir)


def replay_into_service(replay, service) -> int:
    """Feed a session's decoded telemetry into a ConnectionService.

    Emits each decoded telemetry frame on ``service.telemetry`` exactly like the
    live reader would. Returns the number of telemetry records emitted.
    """
    count = 0
    for df in replay.iter_decoded_frames():
        if df.stream is None or df.record is None:
            continue
        stream_id = df.msg_id - api.MSG_TELEMETRY_BASE
        service.telemetry.emit(stream_id, df.record)
        count += 1
    return count
