"""ConnectionService tests: the Qt bridge must run commands on a worker thread
and re-emit firmware acks as Qt signals.

A :class:`RespondingStream` (from ``test_protocol_client``) answers commands, and
a :class:`ProtocolClient` is injected directly so no serial hardware is needed.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api

pytest.importorskip("PySide6")

from transport.protocol_client import ProtocolClient

from .test_protocol_client import RespondingStream, _ctrl_ok


def _service_with_client(handlers):
    from services import ConnectionService

    service = ConnectionService()
    stream = RespondingStream(handlers)
    client = ProtocolClient(stream, response_timeout=1.0)
    client.start()
    service._client = client  # inject; bypasses real serial connect
    return service, client


def test_emergency_stop_sends_real_estop(qtbot) -> None:
    service, client = _service_with_client({api.MSG_ESTOP: _ctrl_ok(state=12)})
    try:
        with qtbot.waitSignal(service.control_result, timeout=2000) as blocker:
            service.emergency_stop()
        kind, result = blocker.args
        assert kind == "estop"
        assert result.ok and result.state == 12
    finally:
        client.stop()


def test_set_arming_emits_control_result(qtbot) -> None:
    service, client = _service_with_client({api.MSG_SET_ARMING: _ctrl_ok(state=4)})
    try:
        with qtbot.waitSignal(service.control_result, timeout=2000) as blocker:
            service.set_arming(True)
        kind, result = blocker.args
        assert kind == "arm"
        assert result.ok and result.state == 4
    finally:
        client.stop()


def test_set_feature_emits_feature_result(qtbot) -> None:
    def handler(payload):
        feature, enable = payload[0], payload[1]
        return bytes([api.FEATURE_OK, 2, feature, 1, enable, 0]), False

    service, client = _service_with_client({api.MSG_FEATURE_SET: handler})
    try:
        with qtbot.waitSignal(service.feature_result, timeout=2000) as blocker:
            service.set_feature(api.FEATURE_SENSOR_POLLING, True)
        (result,) = blocker.args
        assert result.feature == api.FEATURE_SENSOR_POLLING
        assert result.enabled is True
    finally:
        client.stop()


def test_command_when_disconnected_emits_error(qtbot) -> None:
    from services import ConnectionService

    service = ConnectionService()
    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        service.clear_fault()
    (msg,) = blocker.args
    assert "not connected" in msg


def test_dxl_get_param_emits_decoded_result(qtbot) -> None:
    import struct

    def submit(_p):
        return bytes([api.DXL_SUBMIT_ACCEPTED, 5, api.DXL_SLOT_PENDING]), False

    def get_result(_p):
        blob = bytes([api.DXL_PARAM_TORQUE_LIMIT, 1, 4]) + struct.pack("<i", 700)
        return bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(blob)]) + blob, False

    service, client = _service_with_client(
        {api.MSG_DXL_GET_PARAM: submit, api.MSG_DXL_GET_RESULT: get_result}
    )
    try:
        with qtbot.waitSignal(service.dxl_result, timeout=2000) as blocker:
            service.dxl_get_param(1, api.DXL_PARAM_TORQUE_LIMIT)
        kind, res = blocker.args
        assert kind == "get_param"
        assert res is not None and res.param().value == 700
    finally:
        client.stop()


def _handshake_handlers():
    name = b"HexNav".ljust(api.DEVICE_NAME_LEN, b"\x00")

    def hello(_p):
        return bytes([0, 1, 0, 2, 0]) + name, False

    def caps(_p):
        import struct

        return bytes([0, 1, 0, 2, 0]) + struct.pack("<I", 0x7) + name, False

    def status(_p):
        import struct

        return (
            struct.pack("<I", 1234)
            + bytes([2, 0])
            + struct.pack("<H", 12000)
            + struct.pack("<I", 0)
        ), False

    return {
        api.MSG_HELLO: hello,
        api.MSG_GET_CAPABILITIES: caps,
        api.MSG_GET_STATUS: status,
    }


def test_connect_disconnect_cycle_against_fakestream(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    stream = RespondingStream(_handshake_handlers())
    monkeypatch.setattr(services_mod, "open_serial", lambda port, baud=115200: stream)

    service = ConnectionService()
    # Handshake completes on the worker thread -> connected(True) is emitted.
    with qtbot.waitSignal(service.connected, timeout=2000) as blocker:
        service.connect_to("/dev/fake", baud=115200)
    assert blocker.args == [True]
    assert service.is_connected

    # Disconnect closes the link and reports connected(False).
    with qtbot.waitSignal(service.connected, timeout=2000) as blocker:
        service.disconnect()
    assert blocker.args == [False]
    assert not service.is_connected
    assert stream.closed


def test_connect_open_failure_emits_error(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    monkeypatch.setattr(services_mod, "open_serial", lambda port, baud=115200: None)

    service = ConnectionService()
    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        service.connect_to("/dev/nope")
    (msg,) = blocker.args
    assert "/dev/nope" in msg
    assert not service.is_connected

