"""Checks for the universal-joint teaching module. Run from ujoint_diff/:  python tmp/check_uj.py

1. at rest (motor centred): every pair of parts, no overlap; gaps under 0.5 mm listed
2. turning: for each motor position and both yoke phases, the motor turned 0..180 deg in 15 deg steps (the joint
   is the same every half turn); the turning parts against each other and against the fixed ones
3. kinematics: pinion angle against motor angle, in phase (equal) and 90 deg out (ripple, compared with the formula
   for two joints in series)
4. pin holes: motor frame's lower side holes over the bed's holes, bearing tower's holes over the base's, at every
   motor position
5. hand crank (motor and coupler lifted off): the crank turned a full turn clear of the base and tower
"""
import itertools
import math
import sys

sys.path.insert(0, "src")
from cadgen import build123d as bd  # noqa: E402

import uj_assembly as A  # noqa: E402
import uj_parts as u  # noqa: E402

ok = True


def ov(a, b):
    if not a.bounding_box().overlaps(b.bounding_box(), 0.01):
        return 0.0
    r = a & b
    return 0.0 if r is None else r.volume


FIXED = ("base", "bearing tower", "motor frame", "tt motor", "sleeve left", "sleeve right")
# turning parts whose contact is by design: shaft in its bore etc. are cleared, but the TT shaft sits in the coupler
SKIP = {frozenset(("tt motor", "coupler"))}

print("== 1. at rest, motor centred")
P = A.parts(0.0, 0.0)
bad, near = [], []
for (na, a), (nb, b) in itertools.combinations(P.items(), 2):
    if frozenset((na, nb)) in SKIP:
        continue
    v = ov(a, b)
    if v > 1e-3:
        bad.append((na, nb, round(v, 3)))
    elif a.bounding_box().overlaps(b.bounding_box(), 0.5):
        g = a.distance_to(b)
        if g < 0.5:
            near.append((round(g, 2), na, nb))
ok &= not bad
print("overlaps:", bad or "none")
print("gaps under 0.5 mm:", len(near))
for g, na, nb in sorted(near)[:60]:
    print(f"   {g:5.2f}  {na} / {nb}")

print("== 2. turning")
MOVERS = ("coupler", "axle A1", "dial motor", "yoke motor", "spider J1", "yoke mid J1", "axle mid", "dial mid",
          "yoke mid J2", "spider J2", "yoke input", "dial input", "axle input")
for x in u.POSITIONS:
    for phase in (0.0, 90.0):
        bad = []
        fixed = None
        for th in range(0, 180, 15):
            p = A.parts(x, float(th), phase)
            if fixed is None:
                fixed = {k: p[k] for k in FIXED}
            mv = {k: p[k] for k in MOVERS}
            mv.update({k: v for k, v in p.items() if k.startswith("journal")})
            for (na, a), (nb, b) in itertools.combinations(mv.items(), 2):
                v = ov(a, b)
                if v > 1e-3:
                    bad.append((th, na, nb, round(v, 3)))
            for (na, a), (nb, b) in itertools.product(mv.items(), fixed.items()):
                if frozenset((na, nb)) in SKIP:
                    continue
                v = ov(a, b)
                if v > 1e-3:
                    bad.append((th, na, nb, round(v, 3)))
        ok &= not bad
        print(f"motor x {x:5.1f} (joint {u.joint_angle(x):4.1f} deg), phase {phase:4.0f}: {bad[:6] or 'no overlaps'}")

print("== 3. kinematics")
for x in u.POSITIONS:
    b = math.radians(u.joint_angle(x))
    err0 = max(abs(((A.out_angle(x, th, 0.0) - th + 180) % 360) - 180) for th in range(0, 360, 5))
    err90 = max(abs(((A.out_angle(x, th, 90.0) - th + 180) % 360) - 180) for th in range(0, 360, 5))
    one = math.degrees(math.atan((1 - math.cos(b)) / (2 * math.sqrt(math.cos(b)))))
    mid = max(abs(((A.solve(x, th)["mid"] - th + 180) % 360) - 180) for th in range(0, 360, 5))
    print(f"x {x:5.1f}: joint {math.degrees(b):4.1f} deg  intermediate lead/lag {mid:4.2f} (one joint, formula "
          f"{one:4.2f})  pinion vs motor: in phase {err0:5.3f}, 90 deg out {err90:5.2f}")
    ok &= err0 < 0.01 and abs(mid - one) < 0.05

print("== 4. pin holes")


def open_along(shape, p, dvec, depth, n=5):
    for i in range(n):
        t = 0.3 + (depth - 0.6) * i / (n - 1)
        if shape.is_inside(bd.Vector(*p) + bd.Vector(*dvec) * t):
            return False
    return True


base = P["base"]
for x in u.POSITIONS:
    frame = A.u.motor_frame(x)
    tower = u.bearing_tower_at(x)
    fx = [x - u.FRAME_SHAFT_X + 4.0 * k for k in range(-9, 10, 2)]
    pts = [(cx, u.FRAME_Y + dy, u.FRAME_LOW_Z) for cx in fx for dy in (-8.0, 0.0, 8.0)]
    miss = [pt for pt in pts if not (open_along(frame, pt, (0, 0, 1), 6.0) and open_along(base, pt, (0, 0, -1), 6.0))]
    tpts = [(x + dx, u.GRID_Y0, u.BASE_TOP) for dx in (-8.0, 0.0, 8.0)]
    tmiss = [pt for pt in tpts if not (open_along(tower, pt, (0, 0, 1), 6.0) and open_along(base, pt, (0, 0, -1), 6.0))]
    ok &= not miss and not tmiss
    print(f"x {x:5.1f}: motor frame {len(pts)} holes {'OK' if not miss else miss}; tower 3 holes {'OK' if not tmiss else tmiss}")

print("== 5. hand crank")
bad = []
for th in range(0, 360, 30):
    p = A.parts(0.0, float(th), 0.0, hand=True)
    for nb in ("base", "bearing tower"):
        v = ov(p["crank"], p[nb])
        if v > 1e-3:
            bad.append((th, nb, round(v, 3)))
ok &= not bad
print("crank:", bad or "clear over a full turn")
bb = bd.Compound(children=list(P.values())).bounding_box()
print(f"envelope x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}  z {bb.min.Z:.1f}..{bb.max.Z:.1f}"
      f"  parts {len(P)}")
print("ALL PASS" if ok else "FAILED")
