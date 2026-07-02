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


def test_connect_version_mismatch_emits_event(qtbot, monkeypatch) -> None:
    # 4sa.5: a handshake against firmware on an incompatible MAJOR protocol
    # version must surface a diagnostic event rather than silently connecting.
    from services import ConnectionService
    import services as services_mod

    handlers = _handshake_handlers()
    name = b"HexNav".ljust(api.DEVICE_NAME_LEN, b"\x00")

    def hello_mismatch(_p):
        # proto_major=9 (host is 0) -> incompatible wire layout.
        return bytes([9, 0, 0, 2, 0]) + name, False

    handlers[api.MSG_HELLO] = hello_mismatch
    stream = RespondingStream(handlers)
    monkeypatch.setattr(services_mod, "open_serial", lambda port, baud=115200: stream)

    service = ConnectionService()
    events: list[tuple[str, str]] = []
    service.event.connect(lambda kind, msg: events.append((kind, msg)))

    # Wait for the handshake worker to finish (connected emitted last); its
    # queued event emissions are delivered before it.
    with qtbot.waitSignal(service.connected, timeout=2000):
        service.connect_to("/dev/fake", baud=115200)
    qtbot.waitUntil(lambda: any(k == "version" for k, _ in events), timeout=2000)

    version_msgs = [m for k, m in events if k == "version"]
    assert version_msgs
    assert "mismatch" in version_msgs[0] and "v9.0" in version_msgs[0]
    service.disconnect()


def test_connect_open_failure_emits_error(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    monkeypatch.setattr(services_mod, "open_serial", lambda port, baud=115200: None)

    service = ConnectionService()
    service._connect_max_attempts = 1
    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        service.connect_to("/dev/nope")
    (msg,) = blocker.args
    assert "/dev/nope" in msg
    assert not service.is_connected


def test_connect_write_failure_does_not_report_connected(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    class FailingWriteStream(RespondingStream):
        def write(self, data: bytes) -> int:
            raise RuntimeError("device not configured")

    stream = FailingWriteStream(_handshake_handlers())
    monkeypatch.setattr(services_mod, "open_serial", lambda port, baud=115200: stream)

    service = ConnectionService()
    service._connect_max_attempts = 1
    connection_states: list[bool] = []
    service.connected.connect(connection_states.append)

    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        service.connect_to("/dev/fake")

    (msg,) = blocker.args
    assert "no HELLO response" in msg
    assert True not in connection_states
    assert not service.is_connected
    assert stream.closed


def test_connect_retries_after_startup_serial_failure(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    class FailingWriteStream(RespondingStream):
        def write(self, data: bytes) -> int:
            raise RuntimeError("device not configured")

    streams = [
        FailingWriteStream(_handshake_handlers()),
        RespondingStream(_handshake_handlers()),
    ]
    opened: list[RespondingStream] = []

    def fake_open(port, baud=115200):
        stream = streams[len(opened)]
        opened.append(stream)
        return stream

    monkeypatch.setattr(services_mod, "open_serial", fake_open)

    service = ConnectionService()
    service._connect_retry_delay_s = 0

    with qtbot.waitSignal(service.connected, timeout=3000) as blocker:
        service.connect_to("/dev/fake")

    assert blocker.args == [True]
    assert service.is_connected
    assert len(opened) == 2
    assert opened[0].closed
    service.disconnect()


def test_available_ports_discovers_without_hardware(monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod
    from transport import PortInfo

    ports = [PortInfo("/dev/cu.usbmodem1", "OpenRB-150", "hwid-1")]
    monkeypatch.setattr(services_mod, "list_serial_ports", lambda: ports)

    service = ConnectionService()
    found = service.available_ports()
    assert [p.device for p in found] == ["/dev/cu.usbmodem1"]


def test_reconnect_cycle_against_fakestream(qtbot, monkeypatch) -> None:
    """connect -> disconnect -> connect again re-handshakes on a fresh link."""
    from services import ConnectionService
    import services as services_mod

    streams: list[RespondingStream] = []

    def fake_open(port, baud=115200):
        stream = RespondingStream(_handshake_handlers())
        streams.append(stream)
        return stream

    monkeypatch.setattr(services_mod, "open_serial", fake_open)

    service = ConnectionService()
    with qtbot.waitSignal(service.connected, timeout=2000) as blocker:
        service.connect_to("/dev/fake")
    assert blocker.args == [True]

    with qtbot.waitSignal(service.connected, timeout=2000):
        service.disconnect()
    assert streams[0].closed

    # A second connect opens a new link and handshakes again.
    with qtbot.waitSignal(service.connected, timeout=2000) as blocker:
        service.connect_to("/dev/fake")
    assert blocker.args == [True]
    assert service.is_connected
    assert len(streams) == 2
    service.disconnect()
