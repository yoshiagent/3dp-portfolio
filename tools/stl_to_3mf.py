"""Write a 3MF next to each binary STL, using the STL's triangles as they are (no re-meshing).

Run:  python tools/stl_to_3mf.py ../mg90s_steering/out/steer_crank.stl [more.stl ...]
-> steer_crank.3mf beside the STL: one object named after the file, same coordinates as the STL.

Why: FreeCAD's Mesh.export (File > Export 3MF) meshes with a different mesher than the
shape.tessellate() the build scripts use for STL, and on some parts it leaves hairline T-junctions
that Bambu Studio reports as open edges. The STL from tessellate() is closed, so convert that.
"""
from __future__ import annotations

import sys
import zipfile
from pathlib import Path

import numpy as np

TYPES = ('<?xml version="1.0" encoding="UTF-8"?>\n'
         '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
         '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
         '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
         '</Types>')
RELS = ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Binary STL -> (unique vertices, triangle indices)."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + n * 50],
                        dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    verts, inv = np.unique(rec["v"].reshape(-1, 3), axis=0, return_inverse=True)
    return verts.astype(float), inv.reshape(-1, 3)


def write_3mf(objects: list[tuple[str, np.ndarray, np.ndarray]], path: Path, title: str = "") -> None:
    """objects: (name, vertices, triangles); one build item per object."""
    objs, items = [], []
    for i, (name, v, t) in enumerate(objects, start=1):
        vx = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in v)
        tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
        objs.append(f'<object id="{i}" name="{name}" type="model"><mesh><vertices>{vx}</vertices>'
                    f'<triangles>{tx}</triangles></mesh></object>')
        items.append(f'<item objectid="{i}"/>')
    meta = f'<metadata name="Title">{title}</metadata>' if title else ""
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'{meta}<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", TYPES)
        z.writestr("_rels/.rels", RELS)
        z.writestr("3D/3dmodel.model", model)


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        stl = Path(arg).resolve()
        v, t = read_stl(stl)
        dst = stl.with_suffix(".3mf")
        write_3mf([(stl.stem, v, t)], dst, stl.stem)
        print(f"{dst}: {len(t)} triangles, {dst.stat().st_size / 1e6:.2f} MB")
