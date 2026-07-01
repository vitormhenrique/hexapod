"""OpenGL 3D URDF view (nxi.4).

Renders the actual HexNav link meshes with :mod:`pyqtgraph.opengl` and poses
them from forward kinematics. ``set_angles`` maps a ``{joint_name: radians}``
dict to per-link world transforms and updates each mesh item in place, so live
USB telemetry (or a replayed session) animates the real robot model with mouse
orbit/zoom/pan for free.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph import Transform3D
from PySide6.QtGui import QColor, QVector3D

from data.urdf_fk import UrdfForwardKinematics
from data.urdf_model import UrdfModel

_BG = "#12131a"
_LINK_COLOR = (0.72, 0.74, 0.78, 1.0)
_BASE_COLOR = (0.45, 0.62, 0.86, 1.0)


def _to_transform3d(matrix: np.ndarray) -> Transform3D:
    """Convert a 4x4 row-major numpy matrix to a pyqtgraph ``Transform3D``."""
    m = np.asarray(matrix, dtype=float).reshape(4, 4)
    return Transform3D(*[float(x) for x in m.flatten()])


class UrdfGLView(gl.GLViewWidget):
    """A 3D mesh view of a URDF, posed by joint angles."""

    def __init__(
        self,
        model: UrdfModel,
        fk: UrdfForwardKinematics,
        link_meshes: dict,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._fk = fk

        self.setBackgroundColor(QColor(_BG))

        root = model.root_link()
        self._items: dict = {}
        for name, (verts, faces) in link_meshes.items():
            color = _BASE_COLOR if name == root else _LINK_COLOR
            mesh = gl.MeshData(vertexes=verts, faces=faces)
            # Compute smooth vertex normals now, under an error guard, so
            # pyqtgraph doesn't divide by zero on degenerate (zero-area) faces
            # while painting. Sanitize any NaNs the normalization still yields
            # and cache them back on the mesh so paint reuses clean normals.
            with np.errstate(invalid="ignore", divide="ignore"):
                normals = mesh.vertexNormals()
            np.nan_to_num(normals, copy=False)
            item = gl.GLMeshItem(
                meshdata=mesh,
                smooth=True,
                shader="shaded",
                color=color,
                glOptions="opaque",
            )
            self.addItem(item)
            self._items[name] = item

        # Frame the camera on the assembled robot (base_link sits at a body
        # corner, so the model is offset from the world origin).
        cx, cy, cz = self._rest_center()
        self.setCameraPosition(
            pos=QVector3D(cx, cy, cz), distance=0.75, elevation=22, azimuth=45
        )
        self.set_angles({})

    def _rest_center(self) -> tuple:
        """Approximate world-space centre of the rest pose (for framing)."""
        transforms = self._fk.link_transforms({})
        pts = [transforms[name][:3, 3] for name in self._items if name in transforms]
        if not pts:
            return (0.0, 0.0, 0.0)
        arr = np.asarray(pts, dtype=float)
        return tuple(((arr.min(0) + arr.max(0)) / 2.0).tolist())

    def mesh_count(self) -> int:
        """Number of rendered link meshes (used by tests)."""
        return len(self._items)

    def set_angles(self, angles: dict) -> None:
        """Pose every link mesh from ``{joint_name: radians}``."""
        transforms = self._fk.link_transforms(angles)
        for name, item in self._items.items():
            matrix = transforms.get(name)
            if matrix is not None:
                item.setTransform(_to_transform3d(matrix))
