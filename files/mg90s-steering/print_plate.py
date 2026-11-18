"""All MG90S steering print parts on one plate: 3MF/steering_plate.3mf, one named object per part.

Inputs are the STL files the FreeCAD builds already write (vehicle orientation, closed meshes):
  ../out/steer_*.stl                             (build_steering.py)
  ../../tt_motor_adapter/out/mg90s_side_*.stl    (build_mg90s_side.py)
The meshes are used as they are (only rotated and moved), so the plate is exactly as closed as those STLs;
re-meshing the FreeCAD STEP files instead leaves T-junction open edges in the crank, servo base and cover.

Each part is turned so a large flat face lies on the bed:
  frame    plate top face down (the bulkhead stands up from it)
  knuckle  front face down: the C profile lies flat, kingpin and axle holes print horizontally
  tie rod  as modelled (x2)
  crank    top face down: the horn recess and the 1.2 relief end up on top, no support
  base     as modelled (open slot up)
  cover    outer face down
Rebuild the FCStd/STL first when a part changes, then:  python src/print_plate.py
"""
from __future__ import annotations

import zipfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STEER = HERE.parent / "out"
SERVO = HERE.parents[1] / "tt_motor_adapter" / "out"
OUT = HERE.parent / "3MF" / "steering_plate.3mf"

KEEP = np.eye(3)
FLIP = np.diag([1.0, -1.0, -1.0])                          # about x 180 deg: +z face down
FRONT = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # about x -90 deg: +y face down

PARTS = [  # (name, stl, rotation, copies)
    ("steer_frame", STEER / "steer_frame.stl", FLIP, 1),
    ("steer_knuckle_right", STEER / "steer_knuckle_right.stl", FRONT, 1),
    ("steer_knuckle_left", STEER / "steer_knuckle_left.stl", FRONT, 1),
    ("steer_tierod", STEER / "steer_tierod.stl", KEEP, 2),
    ("steer_crank", STEER / "steer_crank.stl", FLIP, 1),
    ("mg90s_side_base", SERVO / "mg90s_side_base.stl", KEEP, 1),
    ("mg90s_side_cover", SERVO / "mg90s_side_cover.stl", FRONT, 1),
]

PLATE_W = 200.0   # packing width along x
GAP = 8.0
BED = (128.0, 128.0)  # plate centre (Bambu Studio puts 3MF build coordinates on the bed as-is)


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Binary STL -> (unique vertices, triangle indices)."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + n * 50],
                        dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    verts, inv = np.unique(rec["v"].reshape(-1, 3), axis=0, return_inverse=True)
    return verts.astype(float), inv.reshape(-1, 3)


def posed() -> list[tuple[str, np.ndarray, np.ndarray]]:
    out = []
    for name, path, rot, copies in PARTS:
        v, t = read_stl(path)
        v = v @ rot.T
        v -= v.min(axis=0)  # corner at the origin, bottom on z = 0
        for k in range(copies):
            out.append((name if copies == 1 else f"{name}_{k + 1}", v, t))
    return out


def layout(parts):
    """Shelf packing, deepest footprint first, then centred on the bed."""
    parts = sorted(parts, key=lambda p: -np.ptp(p[1][:, 1]))
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    for name, v, t in parts:
        sx, sy = np.ptp(v[:, 0]), np.ptp(v[:, 1])
        if x and x + sx > PLATE_W:
            x, y, row_h = 0.0, y + row_h + GAP, 0.0
        placed.append((name, v + (x, y, 0), t))
        x += sx + GAP
        row_h = max(row_h, sy)
    allv = np.vstack([v for _, v, _ in placed])
    shift = np.array([BED[0], BED[1], 0]) - np.array([*(allv.min(0)[:2] + allv.max(0)[:2]) / 2, 0])
    return [(name, v + shift, t) for name, v, t in placed]


def write_3mf(parts, path: Path) -> None:
    objs, items = [], []
    for i, (name, v, t) in enumerate(parts, start=1):
        vx = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in v)
        tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
        objs.append(f'<object id="{i}" name="{name}" type="model"><mesh><vertices>{vx}</vertices>'
                    f'<triangles>{tx}</triangles></mesh></object>')
        items.append(f'<item objectid="{i}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="Title">MG90S steering: all parts</metadata>'
             f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    types = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
             '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
             '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
             '</Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


if __name__ == "__main__":
    parts = layout(posed())
    write_3mf(parts, OUT)
    allv = np.vstack([v for _, v, _ in parts])
    print(f"wrote {OUT.name}: {len(parts)} objects, footprint "
          f"{np.ptp(allv[:, 0]):.1f} x {np.ptp(allv[:, 1]):.1f} mm, max height {allv[:, 2].max():.1f} mm, "
          f"{OUT.stat().st_size / 1e6:.2f} MB")
    for name, v, _ in parts:
        lo, hi = v.min(0), v.max(0)
        print(f"  {name:22s} x {lo[0]:6.1f}..{hi[0]:6.1f}  y {lo[1]:6.1f}..{hi[1]:6.1f}  h {hi[2]:5.1f}")
