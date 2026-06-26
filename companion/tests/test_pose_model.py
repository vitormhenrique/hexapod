"""Unit tests for the UI-independent hexapod kinematic pose model."""

from __future__ import annotations

import math

from hexapod_protocol import config as cfg
from hexapod_protocol import telemetry as tlm

from models.pose_model import HexapodPoseModel, HOME_FOOT_Z_MM, HOME_RADIUS_MM

# Home foot positions in body frame B (mm), mirror test_leg_ik.cpp kHomeFootB.
HOME_FOOT_B = [
    (-155.4, -205.4, -40.0),
    (155.4, -205.4, -40.0),
    (196.8, 0.0, -40.0),
    (155.4, 205.4, -40.0),
    (-155.4, 205.4, -40.0),
    (-196.8, 0.0, -40.0),
]


def _model() -> HexapodPoseModel:
    return HexapodPoseModel(cfg.default_robot_config())


def test_zero_angles_place_foot_at_home_all_legs() -> None:
    m = _model()
    for i, (hx, hy, hz) in enumerate(HOME_FOOT_B):
        foot = m.leg(i).foot
        assert math.isclose(foot.x, hx, abs_tol=0.6)
        assert math.isclose(foot.y, hy, abs_tol=0.6)
        assert math.isclose(foot.z, hz, abs_tol=0.5)


def test_hip_points_match_mount_plus_lift() -> None:
    config = cfg.default_robot_config()
    m = HexapodPoseModel(config)
    for i, leg in enumerate(config.legs):
        hip = m.leg(i).hip
        assert math.isclose(hip.x, leg.mount_x_dmm / 10.0, abs_tol=1e-6)
        assert math.isclose(hip.y, leg.mount_y_dmm / 10.0, abs_tol=1e-6)
        assert math.isclose(hip.z, leg.mount_z_dmm / 10.0 + 21.0, abs_tol=1e-6)


def test_chain_has_four_points_per_leg() -> None:
    m = _model()
    for leg in m.legs():
        assert len(leg.points) == 4


def test_coxa_yaw_swings_foot_sideways() -> None:
    m = _model()
    base = m.leg(0).foot
    hip_before = m.leg(0).hip
    # Positive coxa angle rotates the foot about the hip-yaw axis.
    m.set_joint_angle(0, 0, math.radians(20.0))
    swung = m.leg(0).foot
    moved = math.hypot(swung.x - base.x, swung.y - base.y)
    assert moved > 1.0
    # The hip anchor is unchanged by joint motion.
    assert math.isclose(m.leg(0).hip.x, hip_before.x, abs_tol=1e-9)
    assert math.isclose(m.leg(0).hip.y, hip_before.y, abs_tol=1e-9)


def test_femur_lowers_foot() -> None:
    m = _model()
    z0 = m.leg(2).foot.z
    # A negative femur angle drops the foot (knee-down sweep).
    m.set_joint_angle(2, 1, math.radians(-20.0))
    z1 = m.leg(2).foot.z
    assert z1 != z0


def test_update_from_joint_state_moves_only_listed_legs() -> None:
    m = _model()
    foot1_before = m.leg(1).foot
    record = tlm.JointStateTelemetry(
        joints=[tlm.JointAngle(leg=0, joint=1, angle_centideg=1500)]
    )
    m.update_from_joint_state(record)
    # Leg 0 changed; leg 1 untouched.
    assert m.leg(1).foot.x == foot1_before.x
    assert m.leg(1).foot.y == foot1_before.y


def test_joint_state_angle_conversion_matches_set_angle() -> None:
    m1 = _model()
    m2 = _model()
    m1.update_from_joint_state(
        tlm.JointStateTelemetry(
            joints=[tlm.JointAngle(leg=3, joint=2, angle_centideg=1234)]
        )
    )
    m2.set_joint_angle(3, 2, math.radians(12.34))
    assert math.isclose(m1.leg(3).foot.x, m2.leg(3).foot.x, abs_tol=1e-9)
    assert math.isclose(m1.leg(3).foot.z, m2.leg(3).foot.z, abs_tol=1e-9)


def test_servo_status_fallback_matches_joint_state_path() -> None:
    config = cfg.default_robot_config()
    smap = cfg.ServoMap(config)
    # Build a servo_status that maps to a known +10deg on leg 4 / femur.
    servo = smap.servo_for(4, 1)
    assert servo is not None
    cmd = cfg.angle_to_tick(servo, math.radians(10.0))
    status = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(
                id=servo.id,
                position=cmd.tick,
                velocity=0,
                load=0,
                voltage_mv=12000,
                temperature_c=30,
                hardware_error=0,
            )
        ]
    )

    m_fallback = HexapodPoseModel(config)
    m_fallback.update_from_servo_status(status)

    # Reference: convert the same tick back to an angle and feed joint_state.
    joints = cfg.servo_status_to_joint_angles(config, status)
    m_ref = HexapodPoseModel(config)
    m_ref.update_from_joint_state(tlm.JointStateTelemetry(joints=joints))

    assert math.isclose(
        m_fallback.leg(4).foot.x, m_ref.leg(4).foot.x, abs_tol=1e-9
    )
    assert math.isclose(
        m_fallback.leg(4).foot.z, m_ref.leg(4).foot.z, abs_tol=1e-9
    )


def test_out_of_range_indices_are_ignored() -> None:
    m = _model()
    m.set_joint_angle(99, 0, 1.0)  # no crash, no change
    m.update_from_joint_state(
        tlm.JointStateTelemetry(
            joints=[tlm.JointAngle(leg=9, joint=9, angle_centideg=1000)]
        )
    )
    assert m.num_legs == 6


def test_home_constants_exposed() -> None:
    assert HOME_RADIUS_MM == 127.0
    assert HOME_FOOT_Z_MM == -44.55
