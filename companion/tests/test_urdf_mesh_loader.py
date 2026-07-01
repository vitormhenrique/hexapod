"""Tests for the numpy URDF mesh loader (nxi.4).

Runs Qt-free against the real HexNav meshes; skips when the description package
is absent from this checkout.
"""

from __future__ import annotations

import numpy as np
import pytest

from data.urdf_model import Origin, find_hexnav_description, load_hexnav

pytestmark = pytest.mark.skipif(
    find_hexnav_description() is None,
    reason="HexNav_description package not present in this checkout",
)


def _mesh_dir():
    return find_hexnav_description() / "meshes" / "HexNav"


def test_load_stl_binary_collision_mesh() -> None:
    from data.mesh_loader import load_stl

    verts, faces = load_stl(_mesh_dir() / "leg_1_femur_collision.stl")
    assert verts.dtype == np.float32
    assert verts.ndim == 2 and verts.shape[1] == 3
    assert faces.shape[1] == 3
    assert len(faces) > 0
    # Every face index references a real vertex.
    assert int(faces.max()) < len(verts)


def test_load_dae_visual_mesh() -> None:
    from data.mesh_loader import load_dae

    verts, faces = load_dae(_mesh_dir() / "leg_1_femur.dae")
    # The COLLADA file declares a single <triangles count="16458"> group.
    assert faces.shape == (16458, 3)
    assert len(verts) > 8000
    assert int(faces.max()) < len(verts)


def test_load_mesh_dispatch_and_bad_format() -> None:
    from data.mesh_loader import load_mesh

    verts, _ = load_mesh(_mesh_dir() / "leg_1_femur.dae")
    assert len(verts) > 0
    with pytest.raises(ValueError):
        load_mesh("model.obj")


def test_apply_origin_scale_shrinks_and_translates() -> None:
    from data.mesh_loader import _apply_origin_scale

    verts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    scaled = _apply_origin_scale(verts, Origin(xyz=(0.0, 0.0, 0.0)), (0.01, 0.01, 0.01))
    assert scaled[0, 0] == pytest.approx(0.01, abs=1e-6)

    moved = _apply_origin_scale(verts, Origin(xyz=(2.0, 3.0, 4.0)), (1.0, 1.0, 1.0))
    assert moved[0].tolist() == pytest.approx([3.0, 3.0, 4.0])


def test_build_link_meshes_covers_all_links() -> None:
    from data.mesh_loader import build_link_meshes

    model = load_hexnav()
    meshes = build_link_meshes(model)
    # Every HexNav link carries visual geometry.
    assert len(meshes) == len(model.links) == 38
    assert model.root_link() in meshes
    for verts, faces in meshes.values():
        assert verts.dtype == np.float32
        assert faces.dtype == np.int32
        assert len(faces) > 0
        assert int(faces.max()) < len(verts)
