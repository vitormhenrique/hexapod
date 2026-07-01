"""Tests for the Qt-free URDF model loader (nxi.3).

Loads the real HexNav description shipped in robot_ros_simulation/ -- the same
model robot_ros_simulation/HexNav_description/launch/display.launch.py renders --
and asserts the body/leg/joint structure required by the acceptance criteria.
"""

from __future__ import annotations

import pytest

from data.urdf_model import (
    Origin,
    UrdfModel,
    find_hexnav_description,
    load_hexnav,
    load_urdf,
)


@pytest.fixture(scope="module")
def hexnav() -> UrdfModel:
    pkg = find_hexnav_description()
    if pkg is None:
        pytest.skip("HexNav_description package not present in this checkout")
    return load_hexnav()


def test_description_package_found() -> None:
    pkg = find_hexnav_description()
    assert pkg is not None
    assert (pkg / "urdf" / "HexNav.urdf").exists()


def test_urdf_loads_with_expected_link_and_joint_counts(hexnav: UrdfModel) -> None:
    # Matches the xacro-generated URDF display.launch.py builds: 38 links,
    # 37 kinematic joints (fixed mounts + 18 continuous), 18 actuated.
    assert hexnav.name == "HexNav"
    assert len(hexnav.links) == 38
    assert len(hexnav.joints) == 37


def test_body_root_link(hexnav: UrdfModel) -> None:
    # The body link is the one that is never a joint child.
    assert hexnav.root_link() == "base_link"


def test_six_legs(hexnav: UrdfModel) -> None:
    assert hexnav.leg_indices() == [1, 2, 3, 4, 5, 6]


def test_eighteen_actuated_joints(hexnav: UrdfModel) -> None:
    actuated = hexnav.actuated_joints()
    assert len(actuated) == 18
    assert all(j.type == "continuous" for j in actuated)
    # 3 actuated joints per leg: coxa, femur, tibia.
    for leg in range(1, 7):
        roles = sorted(j.leg_role for j in hexnav.leg_joints(leg))
        assert roles == ["coxa", "femur", "tibia"]


def test_actuated_joint_ordering(hexnav: UrdfModel) -> None:
    # Ordered by (leg, coxa->femur->tibia) so telemetry indexing is stable.
    assert hexnav.actuated_joint_names()[:3] == [
        "leg_1_coxa_joint",
        "leg_1_femur_joint",
        "leg_1_tibia_joint",
    ]


def test_visual_meshes_resolve_to_files(hexnav: UrdfModel) -> None:
    base = hexnav.links["base_link"]
    assert base.visual_meshes, "base_link should have a visual mesh"
    mesh = base.visual_meshes[0]
    assert mesh.filename.startswith("package://HexNav_description/")
    assert mesh.exists, f"mesh did not resolve: {mesh.filename} -> {mesh.resolved}"


def test_joint_tree_is_connected(hexnav: UrdfModel) -> None:
    # Every non-root link is reachable from base_link through parent->child.
    root = hexnav.root_link()
    seen = {root}
    frontier = [root]
    while frontier:
        link = frontier.pop()
        for j in hexnav.child_joints(link):
            if j.child not in seen:
                seen.add(j.child)
                frontier.append(j.child)
    assert seen == set(hexnav.links)


def test_ros2_control_joint_refs_are_ignored() -> None:
    # <joint> entries without type/parent/child (ros2_control) must not appear
    # as kinematic joints.
    model = load_urdf(
        _write_tmp_urdf(),
    )
    assert set(model.joints) == {"a_joint"}
    assert model.actuated_joint_names() == ["a_joint"]


def test_origin_parsing_defaults() -> None:
    assert Origin.from_element(None).xyz == (0.0, 0.0, 0.0)


# --- helpers ---------------------------------------------------------------


def _write_tmp_urdf():
    import tempfile
    from pathlib import Path

    xml = """<?xml version="1.0"?>
<robot name="tiny">
  <link name="base"/>
  <link name="tip"/>
  <joint name="a_joint" type="continuous">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
  </joint>
  <ros2_control name="x" type="system">
    <joint name="a_joint">
      <command_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""
    tmp = Path(tempfile.mkdtemp()) / "tiny.urdf"
    tmp.write_text(xml)
    return tmp
