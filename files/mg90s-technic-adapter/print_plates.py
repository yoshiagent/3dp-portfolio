"""One print plate per holder version, each part turned to its print orientation (user's request 2026-11-27):
  3MF/mg90s_top_loading_plate.3mf    frame (as modelled, open top up), lid (top face down), horn adapter (top face down)
  3MF/mg90s_side_loading_plate.3mf   base (back face down), cover (outer face down), horn adapter
Inputs are the closed STL meshes the FreeCAD builds write in ../../tt_motor_adapter/out (build_mg90s_technic.py,
build_mg90s_side.py); they are only rotated and moved. Layout and 3MF writing reuse
../../mg90s_steering/src/print_plate.py.
Why these orientations:
  frame   floor down: the servo pocket opens upward, the cable slot is a notch from the bottom (a short bridge)
  lid     top down: the tab / case recesses underneath end up on top, no support
  horn    top down: the cross-horn recess and screw-head pocket underneath end up on top, no support
  base    back face down: the servo cavity, tab slot and cable groove run straight up and open at the top (as
          modelled their roofs hung over the cavity: Bambu Studio warned of floating regions)
  cover   outer face down: the plug faces up
Run from mg90s_technic/:  python src/print_plates.py
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SERVO = ROOT / "tt_motor_adapter" / "out"
OUT = HERE.parent / "3MF"

_spec = importlib.util.spec_from_file_location("steer_plate", ROOT / "mg90s_steering" / "src" / "print_plate.py")
sp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sp)

KEEP, FLIP, FRONT, BACK = sp.KEEP, sp.FLIP, sp.FRONT, sp.BACK
PLATES = {
    "mg90s_top_loading_plate": [("mg90s_technic_frame", KEEP), ("mg90s_technic_lid", FLIP),
                                ("mg90s_horn_adapter", FLIP)],
    "mg90s_side_loading_plate": [("mg90s_side_base", BACK), ("mg90s_side_cover", FRONT),
                                 ("mg90s_horn_adapter", FLIP)],
}


def posed(parts):
    out = []
    for stem, rot in parts:
        v, t = sp.read_stl(SERVO / f"{stem}.stl")
        v = v @ rot.T
        out.append((stem, v - v.min(axis=0), t))
    return out


if __name__ == "__main__":
    sp.PLATE_W = 120.0
    for name, parts in PLATES.items():
        placed = sp.layout(posed(parts))
        path = OUT / f"{name}.3mf"
        sp.write_3mf(placed, path)
        allv = np.vstack([v for _, v, _ in placed])
        print(f"wrote {path.name}: {len(placed)} objects, footprint {np.ptp(allv[:, 0]):.1f} x "
              f"{np.ptp(allv[:, 1]):.1f} mm, max height {allv[:, 2].max():.1f} mm")
        for n, v, _ in placed:
            lo, hi = v.min(0), v.max(0)
            print(f"  {n:22s} x {lo[0]:6.1f}..{hi[0]:6.1f}  y {lo[1]:6.1f}..{hi[1]:6.1f}  h {hi[2]:5.1f}")
