"""Hardware-free tests for ROS high-level motion conversion and TTL handling."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pytest

from hexapod_jetson_bridge.motion_command_adapter import (
    MotionCommandAdapter,
    MotionCommandInput,
    map_motion_command,
)


@dataclass(frozen=True)
class MotionResult:
    ok: bool


class FakeBridge:
    def __init__(self, rejected_call: str | None = None) -> None:
        self.calls: list[tuple[str, tuple[object, ...]]] = []
        self._rejected_call = rejected_call

    def set_gait(self, gait: int) -> MotionResult:
        return self._record("set_gait", gait)

    def set_gait_params(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> MotionResult:
        return self._record(
            "set_gait_params",
            body_height_mm,
            stride_len_mm,
            step_height_mm,
            duty_x255,
            speed_x255,
        )

    def set_body_twist(self, vx: float, vy: float, wz: float) -> MotionResult:
        return self._record("set_body_twist", vx, vy, wz)

    def set_body_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> MotionResult:
        return self._record(
            "set_body_pose", x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg
        )

    def stop_motion(self) -> MotionResult:
        return self._record("stop_motion")

    def _record(self, name: str, *values: object) -> MotionResult:
        self.calls.append((name, values))
        return MotionResult(ok=name != self._rejected_call)


def make_command(**changes: object) -> MotionCommandInput:
    values: dict[str, object] = {
        "gait": 3,
        "body_height_m": 0.08,
        "stride_length_m": 0.04,
        "step_height_m": 0.02,
        "duty_factor": 0.5,
        "speed_scale": 0.5,
        "normalized_vx": 0.25,
        "normalized_vy": -0.5,
        "normalized_wz": 0.75,
        "body_translation_x_m": 0.01,
        "body_translation_y_m": -0.02,
        "body_translation_z_m": 0.03,
        "body_roll_rad": 0.1,
        "body_pitch_rad": -0.2,
        "body_yaw_rad": 0.3,
        "valid_for_sec": 0,
        "valid_for_nanosec": 500_000_000,
    }
    values.update(changes)
    return MotionCommandInput(**values)  # type: ignore[arg-type]


def test_mapping_matches_firmware_envelope_and_rounding():
    mapped = map_motion_command(
        make_command(
            gait=5,
            body_height_m=0.150,
            stride_length_m=-0.010,
            step_height_m=0.080,
            duty_factor=0.5,
            speed_scale=1.1,
            normalized_vx=2.0,
            normalized_vy=-2.0,
            normalized_wz=0.25,
            body_translation_x_m=0.060,
            body_translation_y_m=-0.060,
            body_translation_z_m=0.025,
            body_roll_rad=0.5,
            body_pitch_rad=-0.5,
            body_yaw_rad=0.1,
            valid_for_sec=2,
            valid_for_nanosec=0,
        )
    )

    assert mapped.gait == 5
    assert mapped.body_height_mm == 120
    assert mapped.stride_length_mm == 0
    assert mapped.step_height_mm == 50
    assert mapped.duty_x255 == 128
    assert mapped.speed_x255 == 255
    assert (mapped.normalized_vx, mapped.normalized_vy, mapped.normalized_wz) == (
        1.0,
        -1.0,
        0.25,
    )
    assert (mapped.pose_x_mm, mapped.pose_y_mm, mapped.pose_z_mm) == (
        50.0,
        -50.0,
        25.0,
    )
    assert mapped.pose_roll_deg == pytest.approx(25.0, abs=0.02)
    assert mapped.pose_pitch_deg == pytest.approx(-25.0, abs=0.02)
    assert mapped.pose_yaw_deg == pytest.approx(math.degrees(0.1))
    assert mapped.validity_s == 1.0


def test_adapter_forwards_only_high_level_commands_then_stops_at_expiry():
    bridge = FakeBridge()
    now = [10.0]
    adapter = MotionCommandAdapter(bridge, clock=lambda: now[0])

    assert adapter.accept(make_command())
    assert [name for name, _ in bridge.calls] == [
        "set_gait",
        "set_gait_params",
        "set_body_twist",
        "set_body_pose",
    ]
    assert bridge.calls[1][1] == (80, 40, 20, 128, 128)
    assert bridge.calls[2][1] == (0.25, -0.5, 0.75)

    now[0] = 10.5
    assert adapter.expire_if_needed()
    assert bridge.calls[-1][0] == "stop_motion"
    assert not adapter.expire_if_needed()


def test_adapter_rejects_invalid_input_without_forwarding():
    bridge = FakeBridge()
    adapter = MotionCommandAdapter(bridge)

    assert not adapter.accept(make_command(gait=6))
    assert adapter.last_error == "motion command has an unknown gait"
    assert bridge.calls == []

    assert not adapter.accept(make_command(body_height_m=math.nan))
    assert adapter.last_error == "motion command contains a non-finite value"
    assert bridge.calls == []

    assert not adapter.accept(make_command(valid_for_nanosec=1))
    assert adapter.last_error == "motion command validity must be positive"
    assert bridge.calls == []


def test_adapter_stops_when_one_forwarded_command_is_rejected():
    bridge = FakeBridge(rejected_call="set_body_twist")
    adapter = MotionCommandAdapter(bridge)

    assert not adapter.accept(make_command())
    assert adapter.last_error == "body twist command was not accepted"
    assert [name for name, _ in bridge.calls] == [
        "set_gait",
        "set_gait_params",
        "set_body_twist",
        "stop_motion",
    ]
    assert not adapter.expire_if_needed()