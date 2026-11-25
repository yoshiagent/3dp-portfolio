"""Checks for the differential + Oldham coupling + TT motor module. Run from oldham_drive/:  python tmp/check_drive.py

1. every pair of parts, at rest: no overlap; gaps under 1 mm listed
2. differential turning: carrier, ring, cross pin, spiders against the drive base, sleeves and motor frame; ring/pinion
   mesh; the Oldham coupling (4x the carrier) against itself and the fixed parts
3. pin holes: the motor frame's lower side lines up with the open holes on top of the piers
4. ground clearance and the module's envelope
(tmp/check_oldham.py turns the coupling alone every 15 deg and checks that both tongues stay in their hubs.)
"""
import itertools
import sys

sys.path.insert(0, "src")
from cadgen import build123d as bd  # noqa: E402

import drive_parts as c  # noqa: E402
from drive_assembly import diff_parts, drive_parts  # noqa: E402

ok = True


def overlap(a, b):
    if not a.bounding_box().overlaps(b.bounding_box(), 0.01):
        return 0.0
    r = a & b
    return 0.0 if r is None else r.volume


print("== 1. at rest")
parts = drive_parts(0.0)
bad, near = [], []
for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
    v = overlap(a, b)
    if v > 1e-3:
        bad.append((na, nb, round(v, 3)))
    elif a.bounding_box().overlaps(b.bounding_box(), 1.0):
        g = a.distance_to(b)
        if g < 1.0:
            near.append((round(g, 2), na, nb))
ok &= not bad
print("overlaps:", bad or "none")
print("gaps under 1 mm:")
for g, na, nb in sorted(near):
    print(f"   {g:5.2f}  {na} / {nb}")

print("== 2. differential + coupling turning")
fixed = {k: parts[k] for k in ("drive base", "inner sleeve left", "inner sleeve right", "motor frame", "tt motor")}
tooth = 360.0 / 44
for spin in (tooth / 2, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 3 * tooth, 3.5 * tooth):
    dp = diff_parts(spin)
    movers = {k: dp[k] for k in ("carrier", "ring gear", "cross pin", "spider top", "spider bottom")}
    bad = []
    for (na, a), (nb, b) in itertools.product(movers.items(), fixed.items()):
        v = overlap(a, b)
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
    v = overlap(dp["ring gear"], dp["pinion"])
    if v > 1e-3:
        bad.append(("ring gear", "pinion", round(v, 3)))
    co = {k: dp[k] for k in ("hub motor", "slider", "hub axle", "input axle")}
    for (na, a), (nb, b) in itertools.combinations(co.items(), 2):
        v = overlap(a, b)
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
    for (na, a), (nb, b) in itertools.product(co.items(), fixed.items()):
        if nb == "tt motor" and na == "hub motor":  # the hub sits on the motor's shaft by design
            continue
        v = overlap(a, b)
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
    ok &= not bad
    print(f"spin {spin:6.2f}: overlaps {bad or 'none'}")

print("== 3. pin holes")


def open_along(shape, p, d, depth, n=5):
    for i in range(n):
        t = 0.3 + (depth - 0.6) * i / (n - 1)
        if shape.is_inside(bd.Vector(*p) + bd.Vector(*d) * t):
            return False
    return True


mx = [(x, y) for cols in c.PIER_COLS for x in cols for y in (-56.0, -48.0, -40.0)]
miss = [p for p in mx if not (open_along(parts["motor frame"], (*p, c.PIER_TOP), (0, 0, 1), 6.0)
                              and open_along(parts["drive base"], (*p, c.PIER_TOP), (0, 0, -1), 6.0))]
ok &= not miss
print(f"motor frame <- drive base piers  {len(mx)} holes  {'OK' if not miss else 'BLOCKED ' + str(miss)}")

print("== 4. ground")
low = sorted((p.bounding_box().min.Z, k) for k, p in parts.items())
for zmin, k in low[:5]:
    print(f"   {k:22s} lowest z {zmin:6.2f}")
bb = bd.Compound(children=list(parts.values())).bounding_box()
print(f"envelope x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}  z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
print(f"parts: {len(parts)}")
print("ALL PASS" if ok else "FAILED")
