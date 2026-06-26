"""Smoke + routing tests for the Typer ``hexapod-cli`` (qqi.5, kj8.3).

The base commands (qqi.5) are exercised with Typer's :class:`CliRunner` so we
prove they import and ``--help`` cleanly without hardware. The control/tuning
subcommands (kj8.3) are routed to a :class:`RespondingStream`-backed
``ProtocolClient`` (reused from the protocol-client tests) by monkeypatching
``cli._connect``, so we verify each subcommand actually sends its command and
prints the firmware's verdict.
"""

from __future__ import annotations

import pytest
from typer.testing import CliRunner

from hexapod_protocol import api

import cli
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


def test_dxl_limits_validation(monkeypatch):
    _patch_client(monkeypatch, {})
    # min >= max is rejected before connecting.
    result = runner.invoke(cli.app, ["dxl", "limits", "1", "3000", "1000"])
    assert result.exit_code == 2
