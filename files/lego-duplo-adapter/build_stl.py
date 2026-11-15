"""產生 LEGO 底 + DUPLO 凸點轉接磚 STL（與 .scad 相同參數）。需要: pip install manifold3d numpy

用法: python build_stl.py [輸出檔.stl] [DUPLO凸點外徑mm] [LEGO寬] [LEGO長]
例:   python build_stl.py lego4x8_duplo2x4.stl 9.55 4 8
"""
import struct
import sys

import numpy as np
from manifold3d import Manifold

PITCH, PLAY = 8.0, 0.1
UNITS_X = int(sys.argv[3]) if len(sys.argv) > 3 else 4  # 須為偶數
UNITS_Y = int(sys.argv[4]) if len(sys.argv) > 4 else UNITS_X
BRICK_H, WALL, TOP_T = 9.6, 1.2, 1.0
TUBE_OD, TUBE_ID = 6.51, 4.8
D_PITCH = 16.0
D_NX, D_NY = UNITS_X // 2, UNITS_Y // 2
D_STUD_OD = float(sys.argv[2]) if len(sys.argv) > 2 else 9.55  # 原 9.35 實印太鬆
D_STUD_ID, D_STUD_H = 6.7, 4.5
FN = 96
E = 0.01

SIZE_X = UNITS_X * PITCH - 2 * PLAY
SIZE_Y = UNITS_Y * PITCH - 2 * PLAY
NAME = f"lego{UNITS_X}x{UNITS_Y}_duplo{D_NX}x{D_NY}"


def cyl(d, h):
    return Manifold.cylinder(h, d / 2, d / 2, FN)


def box(x, y, z):
    return Manifold.cube([x, y, z])


shell = box(SIZE_X, SIZE_Y, BRICK_H).translate([-SIZE_X / 2, -SIZE_Y / 2, 0]) - box(
    SIZE_X - 2 * WALL, SIZE_Y - 2 * WALL, BRICK_H - TOP_T + E
).translate([-SIZE_X / 2 + WALL, -SIZE_Y / 2 + WALL, -E])

tube = cyl(TUBE_OD, BRICK_H - TOP_T + E) - cyl(TUBE_ID, BRICK_H).translate([0, 0, -E])
tubes = [
    tube.translate([(i - UNITS_X / 2) * PITCH, (j - UNITS_Y / 2) * PITCH, 0])
    for i in range(1, UNITS_X)
    for j in range(1, UNITS_Y)
]

stud = cyl(D_STUD_OD, D_STUD_H + E) - cyl(D_STUD_ID, D_STUD_H + E).translate([0, 0, E])
studs = [
    stud.translate([(i - (D_NX - 1) / 2) * D_PITCH, (j - (D_NY - 1) / 2) * D_PITCH, BRICK_H - E])
    for i in range(D_NX)
    for j in range(D_NY)
]

brick = shell
for part in tubes + studs:
    brick = brick + part

mesh = brick.to_mesh()
verts = np.asarray(mesh.vert_properties)[:, :3]
tris = np.asarray(mesh.tri_verts)

out = sys.argv[1] if len(sys.argv) > 1 else f"{NAME}.stl"
with open(out, "wb") as f:
    f.write(NAME.encode().ljust(80, b" "))
    f.write(struct.pack("<I", len(tris)))
    for a, b, c in verts[tris]:
        n = np.cross(b - a, c - a)
        n = n / (np.linalg.norm(n) or 1)
        f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))

lo, hi = brick.bounding_box()[:3], brick.bounding_box()[3:]
print(f"wrote {out}: {len(tris)} triangles")
print(f"bbox: {np.round(np.subtract(hi, lo), 3)} mm, volume {brick.volume():.1f} mm^3, genus {brick.genus()}")
