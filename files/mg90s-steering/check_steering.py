"""Sweep the steering range and check every pair of parts for overlap; report the tightest gaps; probe the parts'
holes. Run from mg90s_steering/:  python tools/check_steering.py"""
import itertools
import math
import sys

sys.path.insert(0, "src")
sys.path.insert(0, "tools")
from cadgen import read_step  # noqa: E402
from cadgen import build123d as bd  # noqa: E402
import linkage  # noqa: E402
from steering_assembly import posed_parts  # noqa: E402

import os  # noqa: E402

DIRECT = os.environ.get("STEER_CRANK") == "direct"  # STEER_CRANK=direct: the no-horn crank version
print("crank version:", "direct (no horn)" if DIRECT else "on the double-arm horn")
link = linkage.Linkage(**linkage.CHOSEN)
th_max = linkage.score(link)[0]
ok = True
tight = {}
for theta in (-th_max, -th_max / 2, 0.0, th_max / 2, th_max):
    parts = posed_parts(theta, direct=DIRECT)
    dl, _ = link.wheel_angle(-1, math.radians(theta))
    dr, _ = link.wheel_angle(1, math.radians(theta))
    bad = []
    for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
        if {na, nb} <= {"servo", "horn", "holder base", "holder cover"}:
            continue   # checked with the holder itself
        c = a & b
        v = 0.0 if c is None else c.volume
        if DIRECT and {na, nb} == {"servo", "crank"} and v > 1e-3:
            # the stand-in shaft is a plain r 2.4 cylinder: it overlaps the crank's spline teeth (r >= 2.009).
            # Fine only if all of it stays inside the spline hole (r <= 2.55, z within its 3 mm depth)
            bb = c.bounding_box()
            z0 = 31.8 + 13.5 - 3.0
            inside = (max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y)) <= 2.56
                      and bb.min.Z >= z0 - 0.01 and bb.max.Z <= z0 + 3.01)
            print(f"   spline engagement servo/crank {v:.2f} mm3, inside the spline hole: {inside}")
            if inside:
                continue
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
        d = a.distance_to(b)
        key = tuple(sorted((na, nb)))
        if d < 3 and (key not in tight or d < tight[key][0]):
            tight[key] = (d, theta)
    ok &= not bad
    print(f"servo {theta:6.1f}  wheels L {dl:6.1f} R {dr:6.1f}  overlaps: {bad or 'none'}")
print("tightest gaps under 3 mm over the sweep:")
for key, (d, th) in sorted(tight.items(), key=lambda kv: kv[1][0]):
    print(f"   {key[0]:14s} / {key[1]:14s} {d:5.2f} mm at servo {th:6.1f}")


def probe(name, path, empty, solid):
    s = read_step(path).solids()[0]
    inside = lambda p: s.is_inside(bd.Vector(*p))  # noqa: E731
    bad = [("empty?", p) for p in empty if inside(p)] + [("solid?", p) for p in solid if not inside(p)]
    print(f"{name}: valid {s.is_valid}, {len(empty)} empty + {len(solid)} solid -> {'PASS' if not bad else 'FAIL'}")
    for b in bad:
        print("   ", b)
    return not bad


ok &= probe("frame", "out/steer_frame.step",
            [(x, 24, z) for x in (-16, -8, 0, 8, 16) for z in (8.4, 15, 31.2)]
            + [(x, 20.7, z) for x in (-16, 0, 16) for z in (11.9, 19.9, 27.9)]
            + [(x, 43.3, 27.9) for x in (-16, 0, 16)] + [(x, y, 27.9) for x in (-24, 0, 24) for y in (32, 40)]
            # counterbores (2.8 off the axis): back face all rows, front face of the bulkhead / plate
            + [(x + 2.8, 20.7, z) for x in (-16, -8, 0, 8, 16) for z in (11.9, 19.9, 27.9)]
            + [(x + 2.8, 27.3, z) for x in (-16, -8, 0, 8, 16) for z in (11.9, 19.9)]
            + [(x + 2.8, 43.3, 27.9) for x in (-16, -8, 0, 8, 16)]
            + [(s * 40, 40, z) for s in (1, -1) for z in (24.6, 27.9, 31.2)] + [(s * 32, 40, 27.9) for s in (1, -1)],
            [(-12, 30, 27.9), (12, 36, 27.9), (36, 40, 25), (-36, 38, 30), (4, 26, 9), (12, 22, 15)])
ok &= probe("knuckle right", "out/steer_knuckle_right.step",
            [(0, 0, z) for z in (16.6, 20, 35, 39.2)] + [(x, 0, 27.9) for x in (6.2, 9.5, 12.8)]
            + [(-5.86, -17.02, z) for z in (32.6, 36, 39.2)] + [(3, 0, 27.9), (0, 0, 27.9)],
            [(9.5, 0, 31), (9.5, 0, 24), (3, 2.5, 20), (3, 2.5, 35), (-3, -8.7, 35), (9.5, 3, 27.9)])
ok &= probe("knuckle left", "out/steer_knuckle_left.step",
            [(0, 0, 20), (-9.5, 0, 27.9), (5.86, -17.02, 36)], [(-9.5, 3, 27.9), (3, -8.7, 35)])
ok &= probe("tie rod", "out/steer_tierod.step",
            [(0, 0, 3.7), (link.rod[1], 0, 3.7), (2.8, 0, 0.4), (link.rod[1] - 2.8, 0, 7.0)],
            [(link.rod[1] / 2, 0, 3.7), (2.8, 0, 3.7)])
# crank: underside z 0, relief 3.8, top 11.2; horn recess 1.9 deep; screws at x +-11.25 (cb D5.2); D4 centre hole
ok &= probe("crank", "out/steer_crank.step",
            [(8, 27, 6), (-8, 27, 10), (8, 27, 2), (0, 0, 10), (1.8, 0, 9), (-11.25, 0, 4), (11.25, 0, 10),
             (13.6, 0, 10), (-8.9, 0, 10), (15, 0, 1), (0, 3.5, 1), (17.5, 1, 1), (2, 1.5, 3)],
            [(0, 18, 6), (11, 27, 6), (4, -4, 6), (15.5, 0, 3), (19, 0, 1), (0, -5, 1), (8.75, 0, 1.5 + 2),
             (6.25, 0, 8), (-6.25, 0, 4), (2.3, 0, 9)])
# direct crank: underside z 0 on the shaft top, hub D10 up to the 5.9 relief, top 13.3; spline 3 deep
ok &= probe("crank direct", "out/steer_crank_direct.step",
            [(2.4 * math.sin(math.radians(9)), 2.4 * math.cos(math.radians(9)), 1.5), (0, 0, 4), (1.8, 0, 10),
             (0, 0, 12), (8, 27, 8), (-8, 27, 12), (8, 27, 3), (8, 0, 3)],
            [(0, 2.3, 1.5), (0, -2.3, 1.5), (4, 0, 2), (0, -6, 8), (0, 15, 9), (11, 27, 9), (3.5, 0, 4.5), (2.3, 0, 10)])
print("RESULT", "PASS" if ok else "FAIL")
