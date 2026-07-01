"""URDF forward kinematics for the model viewer (nxi.4).

Walks the :class:`~data.urdf_model.UrdfModel` joint tree from the body/root link
and composes each link's world transform from the joint origins and the actuated
joint angles carried by ``joint_state`` telemetry. The result is a set of link
positions and skeleton segments the URDF Viewer projects to 2D (no OpenGL
dependency).

Math is numpy-only (numpy ships transitively with pyqtgraph) and Qt-free, so it
unit-tests headless. URDF conventions: a joint ``<origin xyz rpy>`` is the fixed
parent->child transform (rotation ``Rz(yaw)·Ry(pitch)·Rx(roll)``); an actuated
joint then rotates the child about its ``axis`` by the joint angle.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from hexapod_protocol import telemetry as tlm

from data.urdf_model import UrdfModel


def _rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _axis_angle_to_matrix(axis, angle: float) -> np.ndarray:
    ax = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(ax))
    if norm < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    x, y, z = ax / norm
    c, s = math.cos(angle), math.sin(angle)
    d = 1.0 - c
    return np.array(
        [
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
        ]
    )


def _homogeneous(rot: np.ndarray, trans) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = trans
    return mat


def joint_state_to_urdf_angles(record: tlm.JointStateTelemetry) -> dict:
    """Map a ``joint_state`` record to ``{urdf_joint_name: angle_rad}``.

    ``JointAngle.leg`` is 0-based and ``.joint`` is 0=coxa/1=femur/2=tibia, which
    matches the URDF joint names ``leg_<leg+1>_<role>_joint``. Angles are
    URDF-zero-relative centidegrees, converted to radians.
    """
    out: dict = {}
    for ja in record.joints:
        role = tlm.JOINT_ROLE_NAMES.get(ja.joint)
        if role is None:
            continue
        out[f"leg_{ja.leg + 1}_{role}_joint"] = math.radians(ja.angle_centideg / 100.0)
    return out


class UrdfForwardKinematics:
    """Solve link world transforms for a URDF given actuated joint angles."""

    def __init__(self, model: UrdfModel) -> None:
        self.model = model
        self.root = model.root_link()
        # Pre-compute the parent-link -> child-joints adjacency and a stable,
        # root-first traversal order so solving is a single linear pass.
        self._children: dict = {}
        for joint in model.joints.values():
            self._children.setdefault(joint.parent, []).append(joint)
        self._order: list = []
        if self.root is not None:
            seen = {self.root}
            stack = [self.root]
            while stack:
                link = stack.pop()
                for joint in self._children.get(link, []):
                    if joint.child in seen:
                        continue  # guard against malformed cyclic trees
                    seen.add(joint.child)
                    self._order.append(joint)
                    stack.append(joint.child)

    # --- solving -----------------------------------------------------------

    def link_transforms(self, angles: Optional[dict] = None) -> dict:
        """Return ``{link_name: 4x4 world transform}`` for the given angles."""
        angles = angles or {}
        transforms: dict = {}
        if self.root is not None:
            transforms[self.root] = np.eye(4)
        for joint in self._order:
            parent_t = transforms.get(joint.parent)
            if parent_t is None:
                continue
            origin = _homogeneous(_rpy_to_matrix(joint.origin.rpy), joint.origin.xyz)
            theta = float(angles.get(joint.name, 0.0)) if joint.is_actuated else 0.0
            actuation = _homogeneous(
                _axis_angle_to_matrix(joint.axis, theta), (0.0, 0.0, 0.0)
            )
            transforms[joint.child] = parent_t @ origin @ actuation
        return transforms

    def link_positions(self, angles: Optional[dict] = None) -> dict:
        """Return ``{link_name: (x, y, z)}`` link origins in the body frame."""
        return {
            name: tuple(t[:3, 3]) for name, t in self.link_transforms(angles).items()
        }

    def segments(self, angles: Optional[dict] = None) -> list:
        """Return skeleton segments ``[(parent_pos, child_pos), ...]``.

        One segment per kinematic joint (parent link origin -> child link
        origin), so the drawn skeleton has an edge for every joint in the tree.
        """
        positions = self.link_positions(angles)
        out: list = []
        for joint in self.model.joints.values():
            p0 = positions.get(joint.parent)
            p1 = positions.get(joint.child)
            if p0 is not None and p1 is not None:
                out.append((p0, p1))
        return out

    def leaf_links(self) -> list:
        """Links that are never a joint parent (leg tips / foot markers)."""
        parents = set(self._children)
        return [name for name in self.model.links if name not in parents]

    def foot_positions(self, angles: Optional[dict] = None) -> list:
        """World positions of the leaf/tip links (drawn as foot markers)."""
        positions = self.link_positions(angles)
        return [positions[name] for name in self.leaf_links() if name in positions]
