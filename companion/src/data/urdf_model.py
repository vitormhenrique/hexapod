"""UI-independent URDF model loader (nxi.3).

Loads the HexNav robot description -- the *same* model the ROS 2
``display.launch.py`` renders. That launch feeds ``HexNav.urdf.xacro`` through
xacro; the committed ``HexNav.urdf`` is its flattened output (verified link- and
joint-identical). We parse the flattened file with the standard library only, so
the companion and its tests need no ROS, xacro, ``yourdfpy``, or 3D dependency.

Downstream (nxi.4 URDF viewer) consumes this model to draw links and to animate
the 18 actuated joints (6 coxa + 6 femur + 6 tibia) from ``joint_state``
telemetry.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional
from xml.etree import ElementTree as ET

# Joint types that carry a controllable degree of freedom.
_ACTUATED_TYPES = frozenset({"revolute", "continuous", "prismatic"})

# leg_<n>_<role>_joint  ->  (leg index, role)
_LEG_JOINT_RE = re.compile(r"leg_(\d+)_(coxa|femur|tibia)_joint$")

# package://<pkg>/<rel-path>
_PACKAGE_RE = re.compile(r"^package://([^/]+)/(.+)$")


def _floats(text: Optional[str], count: int, default: float = 0.0) -> tuple:
    if not text:
        return (default,) * count
    parts = text.split()
    vals = [float(p) for p in parts[:count]]
    while len(vals) < count:
        vals.append(default)
    return tuple(vals)


@dataclass(frozen=True)
class Origin:
    """A joint/visual origin: translation (m) and RPY rotation (rad)."""

    xyz: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)

    @classmethod
    def from_element(cls, elem: Optional[ET.Element]) -> "Origin":
        if elem is None:
            return cls()
        return cls(
            xyz=_floats(elem.get("xyz"), 3),
            rpy=_floats(elem.get("rpy"), 3),
        )


@dataclass
class MeshRef:
    """A ``<mesh>`` reference and its resolved filesystem path (if found)."""

    filename: str  # original, e.g. package://HexNav_description/meshes/...
    resolved: Optional[Path] = None
    scale: tuple = (1.0, 1.0, 1.0)
    origin: Origin = field(default_factory=Origin)

    @property
    def exists(self) -> bool:
        return self.resolved is not None and self.resolved.exists()


@dataclass
class Link:
    name: str
    visual_meshes: list = field(default_factory=list)
    collision_meshes: list = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Origin = field(default_factory=Origin)
    axis: tuple = (1.0, 0.0, 0.0)

    @property
    def is_actuated(self) -> bool:
        return self.type in _ACTUATED_TYPES

    @property
    def leg_index(self) -> Optional[int]:
        """1-based leg index parsed from an actuated leg joint name, else None."""
        m = _LEG_JOINT_RE.match(self.name)
        return int(m.group(1)) if m else None

    @property
    def leg_role(self) -> Optional[str]:
        """coxa/femur/tibia for a leg joint, else None."""
        m = _LEG_JOINT_RE.match(self.name)
        return m.group(2) if m else None


class UrdfModel:
    """Parsed URDF: links, kinematic joints, and helpers for the viewer."""

    def __init__(self, name: str, links: list, joints: list) -> None:
        self.name = name
        self.links: dict = {ln.name: ln for ln in links}
        self.joints: dict = {j.name: j for j in joints}

    # --- joints ------------------------------------------------------------

    def actuated_joints(self) -> list:
        """Non-fixed joints, ordered by (leg index, role), then name."""
        role_order = {"coxa": 0, "femur": 1, "tibia": 2}

        def key(j: Joint):
            return (j.leg_index or 0, role_order.get(j.leg_role or "", 9), j.name)

        return sorted((j for j in self.joints.values() if j.is_actuated), key=key)

    def actuated_joint_names(self) -> list:
        return [j.name for j in self.actuated_joints()]

    def child_joints(self, link_name: str) -> list:
        """Joints whose parent is ``link_name`` (for tree traversal)."""
        return [j for j in self.joints.values() if j.parent == link_name]

    # --- links / structure -------------------------------------------------

    def root_link(self) -> Optional[str]:
        """The body/base link: the one that is never a joint child."""
        children = {j.child for j in self.joints.values()}
        roots = [name for name in self.links if name not in children]
        return roots[0] if roots else None

    def leg_indices(self) -> list:
        """Sorted, de-duplicated leg indices present in actuated joints."""
        return sorted({j.leg_index for j in self.actuated_joints() if j.leg_index})

    def leg_joints(self, leg_index: int) -> list:
        """Actuated joints belonging to ``leg_index``."""
        return [j for j in self.actuated_joints() if j.leg_index == leg_index]

    def meshes(self) -> Iterable:
        for link in self.links.values():
            yield from link.visual_meshes
            yield from link.collision_meshes


# --- parsing ---------------------------------------------------------------


def _resolve_package(filename: str, package_dirs: dict) -> Optional[Path]:
    m = _PACKAGE_RE.match(filename)
    if not m:
        # Plain/relative path: leave unresolved unless it exists as-is.
        p = Path(filename)
        return p if p.exists() else None
    pkg, rel = m.group(1), m.group(2)
    base = package_dirs.get(pkg)
    return (Path(base) / rel) if base else None


def _parse_meshes(parent: ET.Element, tag: str, package_dirs: dict) -> list:
    out = []
    for section in parent.findall(tag):
        geom = section.find("geometry")
        if geom is None:
            continue
        mesh = geom.find("mesh")
        if mesh is None:
            continue
        filename = mesh.get("filename", "")
        out.append(
            MeshRef(
                filename=filename,
                resolved=_resolve_package(filename, package_dirs),
                scale=_floats(mesh.get("scale"), 3, default=1.0),
                origin=Origin.from_element(section.find("origin")),
            )
        )
    return out


def load_urdf(path, package_dirs: Optional[dict] = None) -> UrdfModel:
    """Parse a (flattened) URDF file into a :class:`UrdfModel`.

    ``package_dirs`` maps a ROS package name to a directory on disk so
    ``package://<pkg>/...`` mesh paths resolve to real files.
    """
    path = Path(path)
    package_dirs = dict(package_dirs or {})
    root = ET.parse(path).getroot()

    links = []
    for le in root.findall("link"):
        links.append(
            Link(
                name=le.get("name", ""),
                visual_meshes=_parse_meshes(le, "visual", package_dirs),
                collision_meshes=_parse_meshes(le, "collision", package_dirs),
            )
        )

    joints = []
    for je in root.findall("joint"):
        jtype = je.get("type")
        parent = je.find("parent")
        child = je.find("child")
        if jtype is None or parent is None or child is None:
            # Skip non-kinematic <joint> refs (e.g. ros2_control entries).
            continue
        axis_el = je.find("axis")
        joints.append(
            Joint(
                name=je.get("name", ""),
                type=jtype,
                parent=parent.get("link", ""),
                child=child.get("link", ""),
                origin=Origin.from_element(je.find("origin")),
                axis=(
                    _floats(axis_el.get("xyz"), 3)
                    if axis_el is not None
                    else (1.0, 0.0, 0.0)
                ),
            )
        )

    return UrdfModel(name=root.get("name", "robot"), links=links, joints=joints)


# --- HexNav description discovery -----------------------------------------

_DESCRIPTION_PKG = "HexNav_description"


def find_hexnav_description() -> Optional[Path]:
    """Locate the ``HexNav_description`` package directory in the monorepo.

    Honors the ``HEXAPOD_ROBOT_DESCRIPTION`` env override, otherwise walks up
    from this file looking for ``robot_ros_simulation/HexNav_description``.
    Returns the package root (which contains ``urdf/`` and ``meshes/``).
    """
    override = os.environ.get("HEXAPOD_ROBOT_DESCRIPTION")
    if override:
        p = Path(override)
        return p if p.exists() else None

    here = Path(__file__).resolve()
    for base in (here, *here.parents):
        cand = base / "robot_ros_simulation" / _DESCRIPTION_PKG
        if (cand / "urdf").is_dir():
            return cand
    return None


def load_hexnav(urdf_name: str = "HexNav.urdf") -> UrdfModel:
    """Load the flattened HexNav URDF used by ``display.launch.py``.

    Raises ``FileNotFoundError`` if the description package cannot be located.
    """
    pkg = find_hexnav_description()
    if pkg is None:
        raise FileNotFoundError(
            "HexNav_description not found; set HEXAPOD_ROBOT_DESCRIPTION to its path"
        )
    urdf_path = pkg / "urdf" / urdf_name
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    return load_urdf(urdf_path, package_dirs={_DESCRIPTION_PKG: str(pkg)})
