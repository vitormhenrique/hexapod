"""Offline behavior checks for the output-disabled board smoke flow."""

from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path

from hexapod_protocol import api
from hexapod_protocol.framing import Header, MsgType


def _load_hil_smoke():
    path = Path(__file__).resolve().parents[2] / "tools" / "hil_smoke.py"
    spec = importlib.util.spec_from_file_location("hil_smoke_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _response(msg_id: int, payload: bytes):
    return Header(msg_type=int(MsgType.RESPONSE), msg_id=msg_id), payload


def _status_payload(state: int, counters: dict[str, int]) -> bytes:
    payload = bytearray()
    payload += struct.pack("<I", 100)
    payload += bytes([state, 0x06])
    payload += struct.pack("<H", 11800)
    payload += struct.pack("<I", 0)
    payload += bytes([0])
    payload += struct.pack("<I", 0)
    payload += bytes([0])
    payload += struct.pack("<HH", 0, 0)
    payload += bytes([0, 0])
    payload += struct.pack("<I", 0)
    payload += bytes([0x1F])
    payload += struct.pack(
        "<IIIII",
        counters["power"],
        counters["torque"],
        counters["goal"],
        0,
        0,
    )
    return bytes(payload)


def test_output_disabled_smoke_checks_immutable_guard(monkeypatch):
    smoke = _load_hil_smoke()
    monkeypatch.setattr(smoke.time, "sleep", lambda _seconds: None)

    runner = smoke.SmokeRunner(object())
    state = {
        "safety": 2,
        "job": 0,
        "counters": {"power": 0, "torque": 0, "goal": 0},
    }

    def exchange(msg_id: int, payload: bytes = b""):
        if msg_id == api.MSG_GET_STATUS:
            return _response(
                msg_id, _status_payload(state["safety"], state["counters"])
            )
        if msg_id == api.MSG_GET_CAPABILITIES:
            return _response(
                msg_id,
                bytes([0, 2, 0, 1, 0])
                + struct.pack("<I", 0)
                + b"OpenRB150-Hex\0\0\0"
                + bytes([api.CAPABILITY_HIL_OUTPUT_DISABLED]),
            )
        if msg_id == api.MSG_ENTER_MAINTENANCE:
            state["safety"] = 8
            return _response(msg_id, struct.pack("<BBI", api.MAINT_OK, 8, 99))
        if msg_id == api.MSG_DXL_POWER:
            state["job"] = 1
            return _response(
                msg_id,
                bytes([api.DXL_SUBMIT_ACCEPTED, 1, api.DXL_SLOT_PENDING]),
            )
        if msg_id == api.MSG_DXL_TORQUE:
            state["job"] = 2
            return _response(
                msg_id,
                bytes([api.DXL_SUBMIT_ACCEPTED, 2, api.DXL_SLOT_PENDING]),
            )
        if msg_id == api.MSG_DXL_GET_RESULT:
            if state["job"] == 1:
                state["counters"]["power"] += 1
                data = bytes([0, 1])
            else:
                state["counters"]["torque"] += 1
                state["counters"]["goal"] += 1
                data = bytes([1, 0])
            return _response(
                msg_id,
                bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OUTPUT_DISABLED, len(data)])
                + data,
            )
        if msg_id == api.MSG_EXIT_MAINTENANCE:
            state["safety"] = 2
            return _response(msg_id, bytes([api.MAINT_OK, 2]))
        raise AssertionError(f"unexpected request {msg_id:#x} {payload.hex()}")

    monkeypatch.setattr(runner, "_exchange", exchange)
    runner.check_hil_output_guard()

    assert runner.checks
    assert all(check.passed for check in runner.checks)