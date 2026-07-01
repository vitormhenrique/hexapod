"""Tests for the Qt-free URDF forward-kinematics engine (nxi.4)."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import pytest

from hexapod_protocol import telemetry as tlm

from data.urdf_fk import (
    UrdfForwardKinematics,
    joint_state_to_urdf_angles,
)
from data.urdf_model import find_hexnav_description, load_hexnav, load_urdf

# --- telemetry mapping (no URDF needed) ------------------------------------


def test_joint_state_to_urdf_angles() -> None:
    record = tlm.JointStateTelemetry(
        joints=[
            tlm.JointAngle(leg=0, joint=0, angle_centideg=9000),  # leg1 coxa 90deg
            tlm.JointAngle(leg=5, joint=2, angle_centideg=-4500),  # leg6 tibia -45
        ]
    )
    angles = joint_state_to_urdf_angles(record)
    assert math.isclose(angles["leg_1_coxa_joint"], math.pi / 2, rel_tol=1e-9)
    assert math.isclose(angles["leg_6_tibia_joint"], -math.pi / 4, rel_tol=1e-9)


# --- FK on a tiny deterministic URDF ---------------------------------------


def _tiny_two_link() -> Path:
    xml = """<?xml version="1.0"?>
<robot name="tiny">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="j1" type="continuous">
    <parent link="base"/>
    <child link="l1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="l1"/>
    <child link="l2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""
    tmp = Path(tempfile.mkdtemp()) / "tiny.urdf"
    tmp.write_text(xml)
    return tmp


def test_fk_zero_angles_places_links_at_origins() -> None:
    fk = UrdfForwardKinematics(load_urdf(_tiny_two_link()))
    pos = fk.link_positions()
    assert pos["base"] == pytest.approx((0.0, 0.0, 0.0))
    assert pos["l1"] == pytest.approx((1.0, 0.0, 0.0))
    assert pos["l2"] == pytest.approx((2.0, 0.0, 0.0))


def test_fk_rotation_propagates_downstream() -> None:
    fk = UrdfForwardKinematics(load_urdf(_tiny_two_link()))
    pos = fk.link_positions({"j1": math.pi / 2})
    # l1 origin is unaffected by its own joint rotation.
    assert pos["l1"] == pytest.approx((1.0, 0.0, 0.0))
    # l2 swings 90deg about z: (1,0,0) + Rz(90)*(1,0,0) = (1,1,0).
    assert pos["l2"] == pytest.approx((1.0, 1.0, 0.0), abs=1e-9)


def test_fk_segments_and_leaves() -> None:
    fk = UrdfForwardKinematics(load_urdf(_tiny_two_link()))
    assert len(fk.segments()) == 2  # one per joint
    assert fk.leaf_links() == ["l2"]
    assert fk.foot_positions() == [pytest.approx((2.0, 0.0, 0.0))]


# --- FK on the real HexNav description --------------------------------------


@pytest.fixture(scope="module")
def hexnav_fk() -> UrdfForwardKinematics:
    if find_hexnav_description() is None:
        pytest.skip("HexNav_description package not present in this checkout")
    return UrdfForwardKinematics(load_hexnav())


def test_hexnav_root_and_all_links_solved(hexnav_fk: UrdfForwardKinematics) -> None:
    assert hexnav_fk.root == "base_link"
    transforms = hexnav_fk.link_transforms()
    assert len(transforms) == 38  # every link gets a world transform
    assert hexnav_fk.link_positions()["base_link"] == pytest.approx((0.0, 0.0, 0.0))


def test_hexnav_segments_cover_every_joint(hexnav_fk: UrdfForwardKinematics) -> None:
    assert len(hexnav_fk.segments()) == 37  # one per kinematic joint


def test_hexnav_coxa_rotation_moves_only_that_leg(
    hexnav_fk: UrdfForwardKinematics,
) -> None:
    base = hexnav_fk.link_positions()
    moved = hexnav_fk.link_positions({"leg_1_coxa_joint": math.pi / 2})
    # The coxa link origin sits at the coxa joint, so it is unmoved; links
    # downstream of it (femur, tibia) swing with the rotation.
    assert moved["leg_1_femur"] != pytest.approx(base["leg_1_femur"])
    assert moved["leg_1_tibia"] != pytest.approx(base["leg_1_tibia"])
    # A different leg and the body are untouched.
    assert moved["leg_4_femur"] == pytest.approx(base["leg_4_femur"])
    assert moved["base_link"] == pytest.approx(base["base_link"])


def test_hexnav_joint_state_drives_pose(hexnav_fk: UrdfForwardKinematics) -> None:
    record = tlm.JointStateTelemetry(
        joints=[tlm.JointAngle(leg=0, joint=1, angle_centideg=3000)]
    )
    angles = joint_state_to_urdf_angles(record)
    moved = hexnav_fk.link_positions(angles)
    base = hexnav_fk.link_positions()
    assert moved["leg_1_tibia"] != pytest.approx(base["leg_1_tibia"])
