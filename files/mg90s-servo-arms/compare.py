"""Compare the FreeCAD rebuilds (out/*.stl) with the source meshes of ../../MG90SSovoArm.3mf, both directions."""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "web" / "tools"))
sys.path.insert(0, str(HERE.parents[1] / "car_model" / "src"))
from inspect_3mf import meshes  # noqa: E402
from print_all import read_stl  # noqa: E402
from replace_bambu_mesh import to_surface, volume  # noqa: E402

PAIRS = {"SingleArm": ("mg90s_arm_single", (-7.125, 0, 2)), "DoubleArm": ("mg90s_arm_double", (0, 0, 2)),
         "CrossArm": ("mg90s_arm_cross", (0, 0, 2))}
for src, (stem, shift) in PAIRS.items():
    sv, st = meshes[src]
    sv = sv + np.array(shift)  # source: hub at x 7.125 (single), z from -2
    nv, nt = read_stl(HERE.parent / "out" / f"{stem}.stl")
    d1 = to_surface(sv, nv, nt)
    d2 = to_surface(nv, sv, st)
    print(f"{stem}: volume source {volume(sv, st):.2f}  rebuild {volume(nv, nt):.2f} mm3")
    print(f"   source vertices -> rebuild surface: median {np.median(d1):.4f}  p99 {np.percentile(d1, 99):.4f}  "
          f"max {d1.max():.4f} at {sv[d1.argmax()].round(3)}")
    print(f"   rebuild vertices -> source surface: median {np.median(d2):.4f}  p99 {np.percentile(d2, 99):.4f}  "
          f"max {d2.max():.4f} at {nv[d2.argmax()].round(3)}")
