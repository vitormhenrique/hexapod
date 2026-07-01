"""Load URDF link meshes into numpy vertex/face arrays (nxi.4).

Qt-free and dependency-light: parses the *actual* HexNav meshes so the URDF
viewer renders real 3D geometry instead of primitives. Binary/ASCII STL and
COLLADA (``.dae``) are supported with the standard library + numpy only. Each
mesh is baked into its link frame by applying the URDF ``<visual>`` origin and
mesh ``scale``, so the forward-kinematics layer only has to place link frames.

Parsed files are cached at module scope so repeatedly constructing the viewer
(app start, tests) re-uses the ~30 MB of parsed geometry.
"""

from __future__ import annotations

import functools
import struct
from pathlib import Path
from typing import Optional

import numpy as np

from data.urdf_fk import _rpy_to_matrix
from data.urdf_model import MeshRef, Origin, UrdfModel

_COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"

Mesh = tuple  # (vertices: (N,3) float32, faces: (M,3) int32)

_EMPTY: Mesh = (
    np.zeros((0, 3), dtype=np.float32),
    np.zeros((0, 3), dtype=np.int32),
)


# --- STL -------------------------------------------------------------------


def _load_stl_binary(data: bytes, n_tri: int) -> Mesh:
    if n_tri == 0:
        return _EMPTY
    # 84-byte header (80 + uint32 count); 50 bytes/triangle: normal(12) + 3
    # vertices(36) + attribute(2). Read the 48 float bytes, drop the normal.
    raw = np.frombuffer(data, dtype=np.uint8, count=n_tri * 50, offset=84)
    tri = raw.reshape(n_tri, 50)[:, :48].copy()
    floats = tri.view("<f4").reshape(n_tri, 12)
    verts = floats[:, 3:12].reshape(n_tri * 3, 3).astype(np.float32)
    faces = np.arange(n_tri * 3, dtype=np.int32).reshape(n_tri, 3)
    return verts, faces


def _load_stl_ascii(text: str) -> Mesh:
    verts = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not verts:
        return _EMPTY
    v = np.asarray(verts, dtype=np.float32)
    faces = np.arange(len(v), dtype=np.int32).reshape(-1, 3)
    return v, faces


def load_stl(path) -> Mesh:
    """Parse a binary or ASCII STL file into (vertices, faces)."""
    data = Path(path).read_bytes()
    n_bin = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + n_bin * 50 and n_bin > 0:
        return _load_stl_binary(data, n_bin)
    text = data.decode("ascii", "replace")
    if text.lstrip().startswith("solid") and "facet" in text:
        return _load_stl_ascii(text)
    # Fall back to a binary read (some binary files start with "solid").
    return _load_stl_binary(data, n_bin)


# --- COLLADA (.dae) --------------------------------------------------------


def _fan_triangulate(idx: np.ndarray, vcount: np.ndarray) -> np.ndarray:
    faces = []
    k = 0
    for c in vcount:
        c = int(c)
        for j in range(1, c - 1):
            faces.append((idx[k], idx[k + j], idx[k + j + 1]))
        k += c
    if not faces:
        return np.zeros((0, 3), dtype=np.int64)
    return np.asarray(faces, dtype=np.int64)


def load_dae(path) -> Mesh:
    """Parse the geometry of a COLLADA file into (vertices, faces).

    Materials, normals, and texture coordinates are ignored -- only positions
    and triangle connectivity are needed to render a shaded solid.
    """
    from xml.etree import ElementTree as ET

    ns = {"c": _COLLADA_NS}
    root = ET.parse(Path(path)).getroot()

    all_v: list = []
    all_f: list = []
    base = 0
    for geom in root.findall(".//c:library_geometries/c:geometry", ns):
        mesh = geom.find("c:mesh", ns)
        if mesh is None:
            continue

        # source id -> (K, stride) float array (positions use the first 3 cols).
        sources: dict = {}
        for src in mesh.findall("c:source", ns):
            fa = src.find("c:float_array", ns)
            if fa is None or not fa.text:
                continue
            acc = src.find(".//c:accessor", ns)
            stride = int(acc.get("stride", "3")) if acc is not None else 3
            vals = np.fromstring(fa.text, dtype=np.float32, sep=" ")
            sources["#" + src.get("id", "")] = vals.reshape(-1, stride)[:, :3]

        # <vertices> maps an id to its POSITION source.
        vmap: dict = {}
        for vtag in mesh.findall("c:vertices", ns):
            pos = None
            for inp in vtag.findall("c:input", ns):
                if inp.get("semantic") == "POSITION":
                    pos = inp.get("source")
            vmap["#" + vtag.get("id", "")] = sources.get(pos)

        def resolve(src_id: Optional[str]):
            if src_id in vmap:
                return vmap[src_id]
            return sources.get(src_id)

        prims = list(mesh.findall("c:triangles", ns)) + list(
            mesh.findall("c:polylist", ns)
        )
        for prim in prims:
            inputs = prim.findall("c:input", ns)
            if not inputs:
                continue
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            voff = None
            vsrc = None
            for inp in inputs:
                if inp.get("semantic") == "VERTEX":
                    voff = int(inp.get("offset", "0"))
                    vsrc = inp.get("source")
            if voff is None:
                continue
            verts = resolve(vsrc)
            if verts is None or len(verts) == 0:
                continue
            p_el = prim.find("c:p", ns)
            if p_el is None or not p_el.text:
                continue
            p = np.fromstring(p_el.text, dtype=np.int64, sep=" ")
            idx = p.reshape(-1, stride)[:, voff]
            if prim.tag.endswith("polylist"):
                vc_el = prim.find("c:vcount", ns)
                vcount = np.fromstring(vc_el.text, dtype=np.int64, sep=" ")
                faces = _fan_triangulate(idx, vcount)
            else:
                faces = idx.reshape(-1, 3)
            all_v.append(verts.astype(np.float32))
            all_f.append(faces.astype(np.int64) + base)
            base += len(verts)

    if not all_v:
        return _EMPTY
    return (
        np.vstack(all_v).astype(np.float32),
        np.vstack(all_f).astype(np.int32),
    )


# --- dispatch + link assembly ---------------------------------------------


def load_mesh(path) -> Mesh:
    suffix = Path(path).suffix.lower()
    if suffix == ".stl":
        return load_stl(path)
    if suffix == ".dae":
        return load_dae(path)
    raise ValueError(f"unsupported mesh format: {path}")


@functools.lru_cache(maxsize=256)
def _load_mesh_cached(path_str: str) -> Mesh:
    return load_mesh(path_str)


def _apply_origin_scale(verts: np.ndarray, origin: Origin, scale) -> np.ndarray:
    """Bake a URDF ``<visual>`` origin + mesh scale into link-frame vertices."""
    v = verts.astype(np.float64)
    v = v * np.asarray(scale, dtype=np.float64)
    rot = _rpy_to_matrix(origin.rpy)
    v = v @ rot.T
    v = v + np.asarray(origin.xyz, dtype=np.float64)
    return v.astype(np.float32)


def _merge(refs) -> Optional[Mesh]:
    vlist: list = []
    flist: list = []
    base = 0
    for ref in refs:
        if not isinstance(ref, MeshRef) or not ref.exists:
            continue
        verts, faces = _load_mesh_cached(str(ref.resolved))
        if len(verts) == 0:
            continue
        verts = _apply_origin_scale(verts, ref.origin, ref.scale)
        vlist.append(verts)
        flist.append(faces.astype(np.int32) + base)
        base += len(verts)
    if not vlist:
        return None
    return (
        np.vstack(vlist).astype(np.float32),
        np.vstack(flist).astype(np.int32),
    )


def build_link_meshes(model: UrdfModel, prefer: str = "visual") -> dict:
    """Return ``{link_name: (vertices, faces)}`` in each link's own frame.

    ``prefer`` selects visual or collision geometry; visual falls back to
    collision when a link has no visual mesh.
    """
    out: dict = {}
    for link in model.links.values():
        primary = link.visual_meshes if prefer == "visual" else link.collision_meshes
        fallback = link.collision_meshes if prefer == "visual" else link.visual_meshes
        merged = _merge(primary) or _merge(fallback)
        if merged is not None:
            out[link.name] = merged
    return out
