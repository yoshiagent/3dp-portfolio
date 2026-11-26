"""List grid points where pin holes along x, y and z all meet (3-way crossings) in the MG90S side holder parts.
A hole along d passes p if, 3.2 mm along d on either side, a ring of radius 2 round the axis is empty and a ring
of radius 3.5 is solid (so open cavities do not count). Run with cadgen python from tt_motor_adapter/."""
import sys
from cadgen import build123d as bd
from cadgen import read_step

rows = [-19.9, -11.9, -3.9, 4.1]
AX = {"x": ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "y": ((0, 1, 0), (1, 0, 0), (0, 0, 1)), "z": ((0, 0, 1), (1, 0, 0), (0, 1, 0))}
total = 0
for stem in sys.argv[1:] or ["mg90s_side_base", "mg90s_side_cover"]:
    s = read_step(f"out/{stem}.step")
    ins = lambda p: s.is_inside(bd.Vector(*p))  # noqa: E731

    def hole(p, name):
        d, e1, e2 = AX[name]
        for k in (3.2, -3.2):
            c = [p[i] + k * d[i] for i in range(3)]
            ring = lambda r: [[c[i] + r * (a * e1[i] + b * e2[i]) for i in range(3)]  # noqa: E731
                              for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1))]
            if all(not ins(q) for q in ring(2.0)) and all(ins(q) for q in ring(3.5)):
                return True
        return False

    hits = [(x, y, z) for x in range(-24, 17, 8) for y in range(-16, 17, 8) for z in rows
            if not ins((x, y, z)) and all(hole((x, y, z), n) for n in "xyz")]
    total += len(hits)
    print(stem, len(hits), "3-way points:", hits)
print("RESULT", "PASS" if total == 0 else "FAIL")
