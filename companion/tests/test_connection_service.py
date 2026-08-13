"""ConnectionService tests: the Qt bridge must run commands on a worker thread
and re-emit firmware acks as Qt signals.

A :class:`RespondingStream` (from ``test_protocol_client``) answers commands, and
a :class:`ProtocolClient` is injected directly so no serial hardware is needed.
"""

from __future__ import annotations

import os
import struct
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api, config as cfg, telemetry as tlm

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


def test_center_all_inverts_active_map_to_physical_2048(qtbot) -> None:
    from services import ConnectionService

    class FakeClient:
        def __init__(self) -> None:
            self.config = cfg.default_robot_config()
            self.angles = None
            self.calls = []

        def read_config(self):
            return self.config

        def set_maint_control_mode(self, mode):
            self.calls.append(("mode", mode))
            return api.MaintControlModeResult(api.MAINT_TARGET_OK, mode, 1)

        def dxl_torque(self, on):
            self.calls.append(("torque", on))
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([1, 18])
            )

        def set_all_joint_targets(self, angles):
            self.calls.append(("targets",))
            self.angles = list(angles)
            return api.AllJointTargetResult(api.MAINT_TARGET_OK, 8, 18, 0)

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    torque_states = []
    service.servo_torque_changed.connect(
        lambda servo_id, enabled, verified: torque_states.append(
            (servo_id, enabled, verified)
        )
    )
    with qtbot.waitSignal(service.joint_target_result, timeout=2000):
        service.center_all_joints()

    assert client.calls == [
        ("mode", api.MAINT_CONTROL_JOINT_TARGETS),
        ("torque", True),
        ("targets",),
    ]
    assert torque_states == [(servo.id, True, True) for servo in client.config.servos]
    assert client.angles is not None and len(client.angles) == 18
    servo_map = cfg.ServoMap(client.config)
    for index, angle_cdeg in enumerate(client.angles):
        leg = index // cfg.JOINTS_PER_LEG
        joint = index % cfg.JOINTS_PER_LEG
        command = servo_map.angle_to_tick(
            leg, joint, angle_cdeg / 100.0 * cfg.DEG_TO_RAD
        )
        assert abs(command.tick - cfg.SERVO_CENTER_TICK) <= 1
        assert not command.clamped_low and not command.clamped_high


def test_enter_maintenance_one_call_retries_gate_then_scans(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    class FakeClient:
        def __init__(self) -> None:
            self.enter_calls = 0
            self.power_calls = 0
            self.scan_calls = 0

        def enter_maintenance(self):
            self.enter_calls += 1
            return api.MaintResultMsg(api.MAINT_OK, 2, 77)

        def get_status(self):
            return SimpleNamespace(state=int(tlm.SafetyState.MAC_MAINTENANCE))

        def dxl_power(self, _on):
            self.power_calls += 1
            if self.power_calls == 1:
                return None
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([1, 1])
            )

        def dxl_scan(self, _first, _last):
            self.scan_calls += 1
            records = b"".join(
                bytes([sid, 29, 0, 1, 2, 1]) for sid in range(1, 19)
            )
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([18]) + records
            )

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._start_maint_heartbeat = lambda *_args: None  # type: ignore[method-assign]
    monkeypatch.setattr(services_mod.time, "sleep", lambda _seconds: None)

    setup_states = []
    service.maintenance_setup_changed.connect(
        lambda busy, ready, detail: setup_states.append((busy, ready, detail))
    )
    service.enter_maintenance()
    qtbot.waitUntil(
        lambda: bool(setup_states) and setup_states[-1][0] is False,
        timeout=2000,
    )
    assert setup_states[0][0] is True
    assert setup_states[-1][0] is False and setup_states[-1][1] is True
    assert client.enter_calls == 1
    assert client.power_calls == 2
    assert client.scan_calls == 1


def test_released_joint_stages_goal_before_enabling_torque(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    class FakeClient:
        def __init__(self) -> None:
            self.calls = []

        def set_joint_target(self, leg, joint, angle):
            self.calls.append(("target", leg, joint, angle))
            return api.JointTargetResult(api.MAINT_TARGET_OK, 8, False, False, 2048)

        def dxl_set_param(self, servo_id, param, value):
            self.calls.append(("torque", servo_id, param, value))
            data = bytes([param, 1]) + struct.pack("<iiB", value, value, 1)
            return api.DxlJobResult(api.DXL_SLOT_DONE, api.DXL_CODE_OK, data)

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    monkeypatch.setattr(services_mod.time, "sleep", lambda _seconds: None)

    with qtbot.waitSignal(service.servo_torque_changed, timeout=2000) as changed:
        service.set_joint_target_with_torque(1, 0, 0, 0)
    assert changed.args == [1, True, True]
    assert client.calls == [
        ("target", 0, 0, 0),
        ("torque", 1, api.DXL_PARAM_TORQUE_ENABLE, 1),
    ]


def test_gait_test_session_sets_up_and_unwinds_without_rc(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    class FakeClient:
        def __init__(self) -> None:
            self.calls = []

        def dxl_power(self, on):
            self.calls.append(("power", on))
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([int(on), 1])
            )

        def dxl_scan(self, first, last):
            self.calls.append(("scan", first, last))
            records = b"".join(
                bytes([sid, 29, 0, 1, 2, 1]) for sid in range(1, 19)
            )
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([18]) + records
            )

        def set_maint_control_mode(self, mode):
            self.calls.append(("mode", mode))
            return api.MaintControlModeResult(api.MAINT_TARGET_OK, 8, mode)

        def set_gait_params(self, *args):
            self.calls.append(("params", *args))

        def set_body_pose(self, *args):
            self.calls.append(("pose", *args))

        def set_body_twist(self, *args):
            self.calls.append(("twist", *args))

        def set_gait(self, gait):
            self.calls.append(("gait", gait))

        def stop_motion(self):
            self.calls.append(("stop",))

        def dxl_torque(self, on):
            self.calls.append(("torque", on))
            return api.DxlJobResult(
                api.DXL_SLOT_DONE,
                api.DXL_CODE_OK,
                    bytes([int(on), 18]),
            )

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._acquire_maint_lock = lambda _c: True  # type: ignore[method-assign]
    service._wait_state = lambda *_a, **_k: True  # type: ignore[method-assign]
    released = []
    service._release_maint_lock = lambda _c: released.append(True)  # type: ignore[method-assign]
    monkeypatch.setattr(services_mod.time, "sleep", lambda _s: None)

    service.start_gait_test(40, 60, 30, 128, 180)
    qtbot.waitUntil(lambda: service.gait_test_active, timeout=2000)
    assert ("mode", api.MAINT_CONTROL_GAIT_PIPELINE) in client.calls
    assert ("torque", True) in client.calls
    assert client.calls.index(("gait", api.GAIT_STAND)) < client.calls.index(
        ("torque", True)
    )

    service.stop_gait_test()
    qtbot.waitUntil(
        lambda: not service.gait_test_active and not service.gait_test_busy,
        timeout=2000,
    )
    assert ("torque", False) in client.calls
    assert ("mode", api.MAINT_CONTROL_JOINT_TARGETS) in client.calls
    assert ("power", False) in client.calls
    assert released == [True]


def test_gait_test_stop_requires_all_torque_off_acknowledgements(qtbot) -> None:
    from services import ConnectionService

    class FakeClient:
        def set_body_twist(self, *_args):
            pass

        def set_body_pose(self, *_args):
            pass

        def stop_motion(self):
            pass

        def dxl_torque(self, _on):
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([0, 17])
            )

    service = ConnectionService()
    service._client = FakeClient()  # type: ignore[assignment]
    service._gait_test_active = True

    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        service.stop_gait_test()
    (error_message,) = blocker.args
    qtbot.waitUntil(lambda: not service.gait_test_busy, timeout=2000)
    assert service.gait_test_active
    assert "torque off not confirmed" in error_message


def test_simulation_gait_session_never_uses_dxl(qtbot) -> None:
    from services import ConnectionService

    class FakeClient:
        def __init__(self) -> None:
            self.calls = []

        def _motion(self, name, *args):
            self.calls.append((name, *args))
            return api.MotionResultMsg(api.MOTION_OK, 5, True)

        def set_arming(self, arm):
            self.calls.append(("arm", arm))
            return api.ControlResult(api.CTRL_OK, 5 if arm else 2, 0)

        def set_gait_params(self, *args):
            return self._motion("params", *args)

        def set_body_pose(self, *args):
            return self._motion("pose", *args)

        def set_body_twist(self, *args):
            return self._motion("twist", *args)

        def set_gait(self, gait):
            return self._motion("gait", gait)

        def stop_motion(self):
            return self._motion("stop")

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._set_simulation_mode(True)

    service.start_gait_test(40, 60, 30, 128, 180)
    qtbot.waitUntil(lambda: service.gait_test_active, timeout=2000)
    assert ("arm", True) in client.calls
    assert ("params", 40, 60, 30, 128, 180) in client.calls
    assert ("gait", api.GAIT_STAND) in client.calls
    assert all(call[0] not in {"power", "scan", "torque", "mode"} for call in client.calls)

    service.stop_gait_test()
    qtbot.waitUntil(lambda: not service.gait_test_active, timeout=2000)
    assert ("stop",) in client.calls
    assert ("arm", False) in client.calls


def test_simulation_suppresses_hardware_background_requests(qtbot) -> None:
    from services import ConnectionService

    class FakeClient:
        def __init__(self) -> None:
            self.subscriptions = []

        def subscribe(self, stream_id, rate_hz):
            self.subscriptions.append((stream_id, rate_hz))

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._set_simulation_mode(True)

    with qtbot.waitSignal(service.config_summary, timeout=1000) as summary:
        service.load_config()
    with qtbot.waitSignal(service.config_loaded, timeout=1000) as loaded:
        service.load_config()
    with qtbot.waitSignal(service.i2c_topology, timeout=1000) as topology:
        service.refresh_i2c_topology()
    with qtbot.waitSignal(service.sensor_status, timeout=1000) as sensors:
        service.refresh_sensor_status()

    service.subscribe(int(tlm.StreamId.HEALTH), 5)
    service.subscribe(int(tlm.StreamId.SERVO_STATUS), 5)
    qtbot.waitUntil(lambda: client.subscriptions == [(int(tlm.StreamId.HEALTH), 5)])
    assert summary.args == [None]
    assert loaded.args == [None]
    assert topology.args == [None]
    assert sensors.args == [None]


def test_passive_enter_requires_all_torque_off_acknowledgements(
    qtbot, monkeypatch
) -> None:
    from services import ConnectionService
    import services as services_mod

    class FakeClient:
        def __init__(self) -> None:
            self.passive_calls = 0

        def dxl_power(self, _on):
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([1, 1])
            )

        def dxl_scan(self, _first, _last):
            records = b"".join(
                bytes([sid, 29, 0, 1, 2, 1]) for sid in range(1, 19)
            )
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([18]) + records
            )

        def dxl_torque(self, _on):
            return api.DxlJobResult(
                api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([0, 17])
            )

        def passive_enter(self):
            self.passive_calls += 1
            return api.PassiveResult(api.PASSIVE_OK, 9)

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._acquire_maint_lock = lambda _c: True  # type: ignore[method-assign]
    service._wait_state = lambda *_a, **_k: True  # type: ignore[method-assign]
    monkeypatch.setattr(services_mod.time, "sleep", lambda _s: None)
    errors = []
    service.error.connect(errors.append)

    service.passive_enter()
    qtbot.waitUntil(lambda: bool(errors), timeout=2000)
    assert "torque off not confirmed" in errors[-1]
    assert client.passive_calls == 0


def test_passive_enter_refreshes_idempotently_when_already_active(qtbot) -> None:
    from services import ConnectionService

    class FakeClient:
        def __init__(self) -> None:
            self.calls = []

        def passive_enter(self):
            self.calls.append(("enter",))
            return api.PassiveResult(api.PASSIVE_OK, 9)

        def subscribe(self, stream_id, rate):
            self.calls.append(("subscribe", stream_id, rate))

    service = ConnectionService()
    client = FakeClient()
    service._client = client  # type: ignore[assignment]
    service._robot_state = 9

    with qtbot.waitSignal(service.passive_result, timeout=2000):
        service.passive_enter()
    qtbot.waitUntil(lambda: len(client.calls) == 2, timeout=2000)
    assert client.calls == [
        ("enter",),
        ("subscribe", int(tlm.StreamId.JOINT_STATE), 50),
    ]


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


def test_connect_tcp_proxy_endpoint_uses_proxy_opener(qtbot, monkeypatch) -> None:
    from services import ConnectionService
    import services as services_mod

    stream = RespondingStream(_handshake_handlers())
    opened: list[str] = []

    def fake_proxy_open(endpoint: str):
        opened.append(endpoint)
        return stream

    monkeypatch.setattr(services_mod, "open_tcp_proxy", fake_proxy_open)

    service = ConnectionService()
    endpoint = "tcp://jetson.local:5555?token=example-token"
    with qtbot.waitSignal(service.connected, timeout=2000) as blocker:
        service.connect_to(endpoint)
    assert blocker.args == [True]
    assert opened == [endpoint]
    service.disconnect()


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
