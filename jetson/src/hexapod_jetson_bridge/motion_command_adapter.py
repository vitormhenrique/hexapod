"""ROS-independent conversion of high-level motion requests to Jetson commands."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol


GAIT_COUNT = 6
MAX_BODY_HEIGHT_MM = 120
MAX_STRIDE_MM = 80
MAX_STEP_HEIGHT_MM = 50
MAX_POSE_TRANSLATION_MM = 50.0
MAX_POSE_ROTATION_RAD = 0.4363
MAXIMUM_VALIDITY_MS = 1000


@dataclass(frozen=True)
class MotionCommandInput:
    """ROS-message-shaped high-level motion input without a ROS dependency."""

    gait: int
    body_height_m: float
    stride_length_m: float
    step_height_m: float
    duty_factor: float
    speed_scale: float
    normalized_vx: float
    normalized_vy: float
    normalized_wz: float
    body_translation_x_m: float
    body_translation_y_m: float
    body_translation_z_m: float
    body_roll_rad: float
    body_pitch_rad: float
    body_yaw_rad: float
    valid_for_sec: int
    valid_for_nanosec: int


@dataclass(frozen=True)
class MappedMotionCommand:
    """Serial-API values bounded to the firmware motion envelope."""

    gait: int
    body_height_mm: int
    stride_length_mm: int
    step_height_mm: int
    duty_x255: int
    speed_x255: int
    normalized_vx: float
    normalized_vy: float
    normalized_wz: float
    pose_x_mm: float
    pose_y_mm: float
    pose_z_mm: float
    pose_roll_deg: float
    pose_pitch_deg: float
    pose_yaw_deg: float
    validity_s: float


class HighLevelMotionBridge(Protocol):
    """The deliberately narrow, safety-preserving Jetson bridge surface."""

    def set_gait(self, gait: int) -> object: ...

    def set_gait_params(
        self,
        body_height_mm: int,
        stride_len_mm: int,
        step_height_mm: int,
        duty_x255: int,
        speed_x255: int,
    ) -> object: ...

    def set_body_twist(self, vx: float, vy: float, wz: float) -> object: ...

    def set_body_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> object: ...

    def stop_motion(self) -> object: ...


def map_motion_command(command: MotionCommandInput) -> MappedMotionCommand:
    """Validate and clamp one ROS high-level request to serial API values.

    The limits and rounding intentionally mirror the C++ SIL adapter. Firmware
    applies its own final validation after the request reaches the MCU.
    """
    if not 0 <= command.gait < GAIT_COUNT:
        raise ValueError("motion command has an unknown gait")

    values = (
        command.body_height_m,
        command.stride_length_m,
        command.step_height_m,
        command.duty_factor,
        command.speed_scale,
        command.normalized_vx,
        command.normalized_vy,
        command.normalized_wz,
        command.body_translation_x_m,
        command.body_translation_y_m,
        command.body_translation_z_m,
        command.body_roll_rad,
        command.body_pitch_rad,
        command.body_yaw_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("motion command contains a non-finite value")

    requested_ms = (
        command.valid_for_sec * 1000 + command.valid_for_nanosec // 1_000_000
    )
    if requested_ms <= 0:
        raise ValueError("motion command validity must be positive")

    return MappedMotionCommand(
        gait=command.gait,
        body_height_mm=_meters_to_millimeters(
            command.body_height_m, MAX_BODY_HEIGHT_MM
        ),
        stride_length_mm=_meters_to_millimeters(
            command.stride_length_m, MAX_STRIDE_MM
        ),
        step_height_mm=_meters_to_millimeters(
            command.step_height_m, MAX_STEP_HEIGHT_MM
        ),
        duty_x255=_unit_to_x255(command.duty_factor),
        speed_x255=_unit_to_x255(command.speed_scale),
        normalized_vx=_clamp(command.normalized_vx, -1.0, 1.0),
        normalized_vy=_clamp(command.normalized_vy, -1.0, 1.0),
        normalized_wz=_clamp(command.normalized_wz, -1.0, 1.0),
        pose_x_mm=_meters_to_bounded_millimeters(command.body_translation_x_m),
        pose_y_mm=_meters_to_bounded_millimeters(command.body_translation_y_m),
        pose_z_mm=_meters_to_bounded_millimeters(command.body_translation_z_m),
        pose_roll_deg=_radians_to_bounded_degrees(command.body_roll_rad),
        pose_pitch_deg=_radians_to_bounded_degrees(command.body_pitch_rad),
        pose_yaw_deg=_radians_to_bounded_degrees(command.body_yaw_rad),
        validity_s=min(requested_ms, MAXIMUM_VALIDITY_MS) / 1000.0,
    )


class MotionCommandAdapter:
    """Forward bounded high-level motion and stop it once its TTL elapses."""

    def __init__(
        self,
        bridge: HighLevelMotionBridge,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._bridge = bridge
        self._clock = clock
        self._expires_at: float | None = None
        self._last_error: str | None = None

    @property
    def last_error(self) -> str | None:
        """The most recent local validation or forwarding error."""
        return self._last_error

    def accept(self, command: MotionCommandInput) -> bool:
        """Validate, forward, and begin enforcing the command's validity TTL."""
        self.expire_if_needed()
        try:
            mapped = map_motion_command(command)
        except ValueError as error:
            self._last_error = str(error)
            return False

        self._last_error = None
        if not self._send("gait", self._bridge.set_gait(mapped.gait)):
            return False
        if not self._send(
            "gait parameters",
            self._bridge.set_gait_params(
                mapped.body_height_mm,
                mapped.stride_length_mm,
                mapped.step_height_mm,
                mapped.duty_x255,
                mapped.speed_x255,
            ),
        ):
            return False
        if not self._send(
            "body twist",
            self._bridge.set_body_twist(
                mapped.normalized_vx,
                mapped.normalized_vy,
                mapped.normalized_wz,
            ),
        ):
            return False
        if not self._send(
            "body pose",
            self._bridge.set_body_pose(
                mapped.pose_x_mm,
                mapped.pose_y_mm,
                mapped.pose_z_mm,
                mapped.pose_roll_deg,
                mapped.pose_pitch_deg,
                mapped.pose_yaw_deg,
            ),
        ):
            return False

        self._expires_at = self._clock() + mapped.validity_s
        return True

    def expire_if_needed(self) -> bool:
        """Stop exactly once when the most recently accepted command expires."""
        if self._expires_at is None or self._clock() < self._expires_at:
            return False
        self._expires_at = None
        self._bridge.stop_motion()
        return True

    def _send(self, label: str, response: object) -> bool:
        if response is not None and bool(getattr(response, "ok", False)):
            return True
        self._last_error = f"{label} command was not accepted"
        self._expires_at = None
        self._bridge.stop_motion()
        return False


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def _round_nonnegative(value: float) -> int:
    return int(math.floor(value + 0.5))


def _meters_to_millimeters(value: float, maximum: int) -> int:
    return _round_nonnegative(_clamp(value * 1000.0, 0.0, float(maximum)))


def _meters_to_bounded_millimeters(value: float) -> float:
    return _clamp(
        value * 1000.0,
        -MAX_POSE_TRANSLATION_MM,
        MAX_POSE_TRANSLATION_MM,
    )


def _radians_to_bounded_degrees(value: float) -> float:
    radians = _clamp(value, -MAX_POSE_ROTATION_RAD, MAX_POSE_ROTATION_RAD)
    return math.degrees(radians)


def _unit_to_x255(value: float) -> int:
    return _round_nonnegative(_clamp(value, 0.0, 1.0) * 255.0)