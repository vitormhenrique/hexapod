"""Tests for the ROS-message boundary that do not require a ROS installation."""

from __future__ import annotations

from dataclasses import dataclass

from hexapod_jetson_bridge.ros2_node import motion_command_from_ros


@dataclass(frozen=True)
class FakeDuration:
    sec: int
    nanosec: int


@dataclass(frozen=True)
class FakeVector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class FakeMotionCommand:
    gait: int = 3
    body_height_m: float = 0.08
    stride_length_m: float = 0.04
    step_height_m: float = 0.02
    duty_factor: float = 0.5
    speed_scale: float = 0.5
    normalized_vx: float = 0.25
    normalized_vy: float = -0.5
    normalized_wz: float = 0.75
    body_translation_m: FakeVector3 = FakeVector3(0.01, -0.02, 0.03)
    body_rpy_rad: FakeVector3 = FakeVector3(0.1, -0.2, 0.3)
    valid_for: FakeDuration = FakeDuration(0, 500_000_000)


def test_ros_message_conversion_does_not_require_rclpy():
    converted = motion_command_from_ros(FakeMotionCommand())

    assert converted.gait == 3
    assert converted.body_translation_x_m == 0.01
    assert converted.body_translation_y_m == -0.02
    assert converted.body_translation_z_m == 0.03
    assert converted.body_roll_rad == 0.1
    assert converted.body_pitch_rad == -0.2
    assert converted.body_yaw_rad == 0.3
    assert converted.valid_for_sec == 0
    assert converted.valid_for_nanosec == 500_000_000