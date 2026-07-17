"""Smoke + routing tests for the Typer ``hexapod-cli`` (qqi.5, kj8.3).

The base commands (qqi.5) are exercised with Typer's :class:`CliRunner` so we
prove they import and ``--help`` cleanly without hardware. The control/tuning
subcommands (kj8.3) are routed to a :class:`RespondingStream`-backed
``ProtocolClient`` (reused from the protocol-client tests) by monkeypatching
``cli._connect``, so we verify each subcommand actually sends its command and
prints the firmware's verdict.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import struct

import pytest
from typer.testing import CliRunner

from hexapod_protocol import api
from hexapod_protocol import hil
from hexapod_protocol.config import default_robot_config, encode_robot_config
from hexapod_protocol.crc16 import crc16

import cli
from data import SessionLogger
from .test_protocol_client import RespondingStream

runner = CliRunner()


# --- qqi.5: base CLI smoke -------------------------------------------------


def test_help_lists_all_command_groups():
    result = runner.invoke(cli.app, ["--help"])
    assert result.exit_code == 0
    for name in (
        "ports",
        "status",
        "stream",
        "log",
        "stream-stats",
        "export-csv",
        "export-report",
        "hil-decode",
        "hil-capture",
        "gui",
        "safety",
        "gait",
        "feature",
        "contact",
        "passive",
        "dxl",
    ):
        assert name in result.output


def test_ports_runs_without_hardware():
    result = runner.invoke(cli.app, ["ports"])
    assert result.exit_code == 0


def test_hil_decode_writes_offline_parity_artifact(monkeypatch, tmp_path):
    @dataclass(frozen=True)
    class _End:
        reason: hil.CaptureEndReason

    @dataclass(frozen=True)
    class _Trace:
        steps: tuple[int, ...]
        end: _End

    with SessionLogger(tmp_path, robot_name="hil") as logger:
        session_dir = logger.dir
    monkeypatch.setattr(
        cli.hil,
        "decode_trace_frames",
        lambda _frames, expected_session_id=None: _Trace((1, 2), _End(hil.CaptureEndReason.COMPLETE)),
    )
    output = tmp_path / "decoded.json"

    result = runner.invoke(
        cli.app,
        ["hil-decode", str(session_dir), "--out", str(output), "--session-id", "7"],
    )

    assert result.exit_code == 0
    assert "decoded 2 controller step(s)" in result.output
    assert json.loads(output.read_text()) == {
        "end": {"reason": 0},
        "steps": [1, 2],
    }


def test_hil_decode_encodes_valid_config_payload_as_hex(monkeypatch, tmp_path):
    @dataclass(frozen=True)
    class _End:
        reason: hil.CaptureEndReason

    @dataclass(frozen=True)
    class _Trace:
        config: hil.ConfigSnapshot
        steps: tuple[int, ...]
        end: _End

    robot_config = default_robot_config()
    payload = encode_robot_config(robot_config)
    trace = _Trace(
        config=hil.ConfigSnapshot(
            revision=9,
            valid=True,
            persistent=True,
            payload_crc16=crc16(payload),
            payload=payload,
            robot_config=robot_config,
        ),
        steps=(1,),
        end=_End(hil.CaptureEndReason.COMPLETE),
    )
    with SessionLogger(tmp_path, robot_name="hil") as logger:
        session_dir = logger.dir
    monkeypatch.setattr(cli.hil, "decode_trace_frames", lambda *_args, **_kwargs: trace)
    output = tmp_path / "decoded.json"

    result = runner.invoke(
        cli.app,
        ["hil-decode", str(session_dir), "--out", str(output)],
    )

    assert result.exit_code == 0
    artifact = json.loads(output.read_text())
    assert artifact["config"]["payload_hex"] == payload.hex()
    assert "payload" not in artifact["config"]


def test_log_registers_lossless_raw_frame_callback(monkeypatch, tmp_path):
    created = _patch_client(
        monkeypatch,
        {
            api.MSG_HELLO: lambda _p: (bytes([0, 2, 0, 0, 0]) + b"test\x00", False),
            api.MSG_SUBSCRIBE: lambda p: (bytes([api.SUB_OK, p[0], p[1], p[2]]), False),
            api.MSG_UNSUBSCRIBE: lambda p: (bytes([api.SUB_OK, p[0], 0, 0]), False),
        },
    )

    class _Logger:
        dir = tmp_path / "session"

        class _Meta:
            record_count = 0
            frame_count = 0

        _meta = _Meta()

        def log_raw_frame(self, _frame: bytes) -> None:
            pass

        def log_record(self, *_args, **_kwargs) -> None:
            pass

        def close(self) -> None:
            pass

    logger = _Logger()
    monkeypatch.setattr(cli, "SessionLogger", lambda **_kwargs: logger)
    result = runner.invoke(
        cli.app,
        ["log", "--seconds", "0", "--out", str(tmp_path), "--streams", "health"],
    )

    assert result.exit_code == 0
    assert logger.log_raw_frame in created["client"]._raw_frame_cbs


def test_export_csv_writes_selected_session_data(tmp_path):
    from .replay_fixtures import build_sample_session

    replay = build_sample_session(tmp_path, frames_per_stream=2)
    output = tmp_path / "signals.csv"

    result = runner.invoke(
        cli.app,
        [
            "export-csv",
            str(replay.dir),
            "--signals",
            "health.battery_mv,servo.1.position",
            "--out",
            str(output),
        ],
    )

    assert result.exit_code == 0, result.output
    assert "exported 4 CSV row(s)" in result.output
    assert "health.battery_mv" in output.read_text(encoding="utf-8")


def test_export_report_writes_human_readable_session_summary(tmp_path):
    from .replay_fixtures import build_sample_session

    replay = build_sample_session(tmp_path, frames_per_stream=1)
    output = tmp_path / "report.txt"

    result = runner.invoke(
        cli.app, ["export-report", str(replay.dir), "--out", str(output)]
    )

    assert result.exit_code == 0, result.output
    assert "wrote report for" in result.output
    text = output.read_text(encoding="utf-8")
    assert "Session report:" in text
    assert "Servo health:" in text


def test_hil_capture_performs_safe_handshake_retention_and_cleanup(
    monkeypatch, tmp_path
):
    maintenance_token = 0x11223344
    session_token = 0x55667788
    actions: list[str] = []

    @dataclass(frozen=True)
    class _End:
        reason: hil.CaptureEndReason
        emitted_fragment_count: int = 3

    @dataclass(frozen=True)
    class _Trace:
        steps: tuple[int, ...]
        end: _End

    class _Assembler:
        def __init__(self, expected_session_id: int) -> None:
            assert expected_session_id == 7

        def accept_event(self, event_id: int, payload: bytes):
            assert (event_id, payload) == (hil.TRACE_FRAGMENT_EVENT, b"done")
            return hil.TraceEnd(
                reason=hil.CaptureEndReason.COMPLETE,
                requested_steps=1,
                recorded_steps=1,
                queue_high_water=1,
                emitted_record_count=3,
                emitted_fragment_count=3,
                final_guard=hil.OutputGuardStatus(
                    True, True, True, True, True, 0, 0, 0, 0, 0, 0
                ),
            )

        def finalize(self):
            return _Trace((1,), _End(hil.CaptureEndReason.COMPLETE))

    monkeypatch.setattr(cli.hil, "TraceAssembler", _Assembler)
    created = {}

    def capabilities(_payload: bytes):
        actions.append("capabilities")
        return (
            bytes([0, 2, 0, 0, 0])
            + struct.pack("<I", 0)
            + b"OpenRB-150\x00".ljust(16, b"\x00")
            + bytes([api.CAPABILITY_HIL_OUTPUT_DISABLED]),
            False,
        )

    def observer_capability(_payload: bytes):
        actions.append("observer_capability")
        return (
            struct.pack(
                "<BBBBIIIHB", api.HIL_OK, 0, 0, 1, 0, 0, 0,
                hil.TRACE_SCHEMA_VERSION, 1,
            ),
            False,
        )

    def status(_payload: bytes):
        actions.append("status")
        return struct.pack("<IBBHI", 100, 2, 0x06, 12000, 0), False

    def enter(_payload: bytes):
        actions.append("enter")
        return struct.pack("<BBI", api.MAINT_OK, 8, maintenance_token), False

    def open_session(payload: bytes):
        assert payload == struct.pack("<I", maintenance_token)
        actions.append("open")
        return (
            struct.pack(
                "<BIIHB", api.HIL_OK, 7, session_token, hil.TRACE_SCHEMA_VERSION, 0x0F
            ),
            False,
        )

    def capture(payload: bytes):
        assert payload == struct.pack("<IB", session_token, 1)
        actions.append("capture")
        return struct.pack("<BI", api.HIL_OK, 9), False

    def observer_heartbeat(payload: bytes):
        assert payload == struct.pack("<I", session_token)
        actions.append("observer_heartbeat")
        created["stream"].push_event(hil.TRACE_FRAGMENT_EVENT, b"done")
        return bytes([api.HIL_OK]), False

    def maintenance_heartbeat(payload: bytes):
        assert payload == struct.pack("<I", maintenance_token)
        actions.append("maintenance_heartbeat")
        return bytes([api.MAINT_OK, 8]), False

    def close_session(payload: bytes):
        assert payload == struct.pack("<I", session_token)
        actions.append("close")
        return bytes([api.HIL_OK]), False

    def exit_maintenance(payload: bytes):
        assert payload == struct.pack("<I", maintenance_token)
        actions.append("exit")
        return bytes([api.MAINT_OK, 2]), False

    created = _patch_client(
        monkeypatch,
        {
            api.MSG_GET_CAPABILITIES: capabilities,
            api.MSG_HIL_GET_CAPABILITY: observer_capability,
            api.MSG_GET_STATUS: status,
            api.MSG_ENTER_MAINTENANCE: enter,
            api.MSG_HIL_OPEN_SESSION: open_session,
            api.MSG_HIL_CAPTURE: capture,
            api.MSG_HIL_HEARTBEAT: observer_heartbeat,
            api.MSG_MAINT_HEARTBEAT: maintenance_heartbeat,
            api.MSG_HIL_CLOSE_SESSION: close_session,
            api.MSG_EXIT_MAINTENANCE: exit_maintenance,
        },
    )

    result = runner.invoke(cli.app, ["hil-capture", "--out", str(tmp_path)])

    assert result.exit_code == 0, result.output
    assert actions == [
        "capabilities",
        "observer_capability",
        "status",
        "enter",
        "open",
        "capture",
        "observer_heartbeat",
        "maintenance_heartbeat",
        "close",
        "exit",
    ]
    session_dir = next(tmp_path.iterdir())
    artifact = json.loads((session_dir / "hil_trace.json").read_text())
    assert artifact == {
        "end": {"reason": 0, "emitted_fragment_count": 3},
        "steps": [1],
    }
    assert json.loads((session_dir / "session.json").read_text())["frame_count"] >= 1
    event_kinds = [json.loads(line)["kind"] for line in (session_dir / "events.jsonl").read_text().splitlines()]
    assert event_kinds == [
        "hil_session_opened",
        "hil_capture_requested",
        "hil_capture_complete",
    ]


@pytest.mark.parametrize(
    "group", ["safety", "gait", "feature", "contact", "passive", "dxl"]
)
def test_subgroup_help(group):
    result = runner.invoke(cli.app, [group, "--help"])
    assert result.exit_code == 0


# --- kj8.3: control/tuning routing ----------------------------------------


def _patch_client(monkeypatch, handlers):
    """Route cli._connect to a RespondingStream-backed ProtocolClient."""
    from transport.protocol_client import ProtocolClient

    created = {}

    def fake_connect(port, baud):
        stream = RespondingStream(handlers)
        client = ProtocolClient(stream, response_timeout=1.0)
        client.start()
        created["client"] = client
        created["stream"] = stream
        return client

    monkeypatch.setattr(cli, "_connect", fake_connect)
    return created


def test_safety_estop(monkeypatch):
    created = _patch_client(
        monkeypatch,
        {api.MSG_ESTOP: lambda _p: (bytes([api.CTRL_OK, 12, 0]), False)},
    )
    result = runner.invoke(cli.app, ["safety", "estop"])
    assert result.exit_code == 0
    assert "ok" in result.output
    assert created["stream"].tx_count == 1


def test_safety_arm_disarm(monkeypatch):
    _patch_client(
        monkeypatch,
        {api.MSG_SET_ARMING: lambda _p: (bytes([api.CTRL_OK, 4, 0]), False)},
    )
    assert runner.invoke(cli.app, ["safety", "arm"]).exit_code == 0
    assert runner.invoke(cli.app, ["safety", "disarm"]).exit_code == 0


def test_gait_set_known_and_unknown(monkeypatch):
    _patch_client(
        monkeypatch,
        {api.MSG_SET_GAIT: lambda _p: (bytes([api.MOTION_OK, 5, 1]), False)},
    )
    ok = runner.invoke(cli.app, ["gait", "set", "tripod"])
    assert ok.exit_code == 0
    assert "ok" in ok.output
    bad = runner.invoke(cli.app, ["gait", "set", "moonwalk"])
    assert bad.exit_code == 2


def test_gait_twist(monkeypatch):
    _patch_client(
        monkeypatch,
        {api.MSG_SET_BODY_TWIST: lambda _p: (bytes([api.MOTION_OK, 5, 1]), False)},
    )
    result = runner.invoke(cli.app, ["gait", "twist", "--vx", "0.5"])
    assert result.exit_code == 0


def test_feature_set_roundtrip(monkeypatch):
    def handler(payload):
        feature = payload[0]
        enable = payload[1]
        return bytes([api.FEATURE_OK, 2, feature, 1, enable, 0]), False

    _patch_client(monkeypatch, {api.MSG_FEATURE_SET: handler})
    result = runner.invoke(cli.app, ["feature", "set", "sensor_polling", "true"])
    assert result.exit_code == 0
    assert "enabled=" in result.output


def test_feature_set_unknown_name(monkeypatch):
    _patch_client(monkeypatch, {})
    result = runner.invoke(cli.app, ["feature", "set", "warp_drive", "true"])
    assert result.exit_code == 2


def test_passive_enter_exit(monkeypatch):
    _patch_client(
        monkeypatch,
        {
            api.MSG_PASSIVE_ENTER: lambda _p: (bytes([api.PASSIVE_OK, 9]), False),
            api.MSG_PASSIVE_EXIT: lambda _p: (bytes([api.PASSIVE_OK, 2]), False),
        },
    )
    assert runner.invoke(cli.app, ["passive", "enter"]).exit_code == 0
    assert runner.invoke(cli.app, ["passive", "exit"]).exit_code == 0


def test_dxl_scan(monkeypatch):
    def submit(_payload):
        return bytes([api.DXL_SUBMIT_ACCEPTED, 7, api.DXL_SLOT_PENDING]), False

    def get_result(_payload):
        # DONE, code OK, len=7: count=1 then one 6-byte record.
        data = bytes([1, 1, 29, 0, 42, 1, 0])
        return bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data, False

    _patch_client(
        monkeypatch,
        {api.MSG_DXL_SCAN: submit, api.MSG_DXL_GET_RESULT: get_result},
    )
    result = runner.invoke(cli.app, ["dxl", "scan"])
    assert result.exit_code == 0
    assert "1 servo(s) found" in result.output
    assert "id=1" in result.output


def test_dxl_power(monkeypatch):
    def submit(_payload):
        return bytes([api.DXL_SUBMIT_ACCEPTED, 4, api.DXL_SLOT_PENDING]), False

    def get_result(_payload):
        # DONE, code OK: [power_on=1, has_control=1].
        data = bytes([1, 1])
        return bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data, False

    _patch_client(
        monkeypatch,
        {api.MSG_DXL_POWER: submit, api.MSG_DXL_GET_RESULT: get_result},
    )
    result = runner.invoke(cli.app, ["dxl", "power", "true"])
    assert result.exit_code == 0
    assert "power_on=True" in result.output
    assert "has_control=True" in result.output


def test_dxl_limits_validation(monkeypatch):
    _patch_client(monkeypatch, {})
    # min >= max is rejected before connecting.
    result = runner.invoke(cli.app, ["dxl", "limits", "1", "3000", "1000"])
    assert result.exit_code == 2
