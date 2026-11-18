"""Car model checks. Run from car_model/:  python tmp/check_car.py

1. every pair of parts, straight ahead: no overlap; gaps under 1 mm listed
2. steering sweep (5 crank angles): the moving steering parts against everything else
3. differential: carrier + ring turned in steps, against the chassis and the parts beside it; ring/pinion mesh
4. pin holes: each mating hole of the servo holder, steering frame, battery box, motor frame, deck, posts and
   breadboard frame lines up with an open hole in the part it pins to (probe points along the shared axis)
5. ground clearance and wheel contact
"""
import itertools
import math
import sys

sys.argv = sys.argv[:1]  # linkage.py reads sys.argv
sys.path.insert(0, "src")
sys.path.insert(0, "../mg90s_steering/tools")
from cadgen import build123d as bd  # noqa: E402
import linkage  # noqa: E402
import car_parts as c  # noqa: E402
from car_assembly import diff_parts, posed_parts  # noqa: E402

ok = True


def overlap(a, b):
    if not a.bounding_box().overlaps(b.bounding_box(), 0.01):
        return 0.0
    r = a & b
    return 0.0 if r is None else r.volume


def bb_near(a, b, tol):
    return a.bounding_box().overlaps(b.bounding_box(), tol)


# parts that sit inside each other by design or were checked in their own projects
INSIDE = [{"servo", "holder base"}, {"servo", "holder cover"}, {"servo", "horn"}, {"horn", "holder base"},
          {"horn", "holder cover"}, {"holder base", "holder cover"}, {"horn", "crank"}]
STEER_MOVING = {"crank", "horn", "knuckle left", "knuckle right", "front wheel left", "front wheel right",
                "tie rod left", "tie rod right"}

print("== 1. static, straight ahead")
parts = posed_parts(0.0, 0.0)
bad, near = [], []
for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
    if {na, nb} in INSIDE:
        continue
    v = overlap(a, b)
    if v > 1e-3:
        bad.append((na, nb, round(v, 3)))
    elif bb_near(a, b, 1.0):
        g = a.distance_to(b)
        if g < 1.0:
            near.append((round(g, 2), na, nb))
ok &= not bad
print("overlaps:", bad or "none")
print("gaps under 1 mm:")
for g, na, nb in sorted(near):
    print(f"   {g:5.2f}  {na} / {nb}")

print("== 2. steering sweep")
link = linkage.Linkage(**linkage.CHOSEN)
th_max = linkage.score(link)[0]
tight = {}
for theta in (-th_max, -th_max / 2, th_max / 2, th_max):
    p = posed_parts(theta, 0.0)
    bad = []
    for na in STEER_MOVING:
        for nb, b in p.items():
            if nb in STEER_MOVING and nb <= na or {na, nb} in INSIDE:
                continue
            v = overlap(p[na], b)
            if v > 1e-3:
                bad.append((na, nb, round(v, 3)))
            elif nb not in STEER_MOVING and bb_near(p[na], b, 3.0):
                g = p[na].distance_to(b)
                key = (na, nb)
                if g < 3.0 and (key not in tight or g < tight[key][0]):
                    tight[key] = (g, theta)
    ok &= not bad
    print(f"crank {theta:6.1f}: overlaps {bad or 'none'}")
print("tightest moving/fixed gaps under 3 mm:")
for (na, nb), (g, th) in sorted(tight.items(), key=lambda kv: kv[1][0]):
    print(f"   {g:5.2f}  {na} / {nb}  at crank {th:.1f}")

print("== 3. differential turning")
fixed = {k: parts[k] for k in ("chassis", "inner sleeve left", "inner sleeve right", "input axle", "battery box",
                               "post rear")}  # side gears mesh with the spiders: checked in the bench model
tooth = 360.0 / 44
for spin in (tooth / 2, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 3 * tooth, 3.5 * tooth):
    dp = diff_parts(spin)
    movers = {k: dp[k] for k in ("carrier", "ring gear", "cross pin", "spider top", "spider bottom")}
    bad, gaps = [], []
    for (na, a), (nb, b) in itertools.product(movers.items(), fixed.items()):
        v = overlap(a, b)
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
    v = overlap(dp["ring gear"], dp["pinion"])
    if v > 1e-3:
        bad.append(("ring gear", "pinion", round(v, 3)))
    gap = dp["ring gear"].distance_to(fixed["chassis"])
    ok &= not bad
    print(f"spin {spin:6.2f}: overlaps {bad or 'none'}   ring-chassis gap {gap:.2f}")

print("== 4. pin holes line up")
ch = parts["chassis"]


def open_along(shape, p, d, depth, n=5):
    """True if points on the axis from p, `depth` along d (unit), are all outside `shape`."""
    for i in range(n):
        t = 0.3 + (depth - 0.6) * i / (n - 1)
        q = bd.Vector(*p) + bd.Vector(*d) * t
        if shape.is_inside(q):
            return False
    return True


def probe(name, shape_a, shape_b, pts, d, depth_a, depth_b):
    """Holes at pts (on the face between a and b), axis d pointing from b into a."""
    global ok
    miss = []
    for p in pts:
        nd = tuple(-x for x in d)
        if not (open_along(shape_a, p, d, depth_a) and open_along(shape_b, p, nd, depth_b)):
            miss.append(tuple(round(x, 1) for x in p))
    ok &= not miss
    print(f"{name:42s} {len(pts):3d} holes  {'OK' if not miss else 'BLOCKED ' + str(miss)}")


z = c.ROW_Z
probe("holder base right face <- chassis arm", parts["holder base"], ch,
      [(19.9, y, z) for y in c.HOLDER_SIDE_ROWS], (-1, 0, 0), 6.0, 6.0)
probe("holder cover left face <- chassis arm", parts["holder cover"], ch,
      [(-19.9, y, z) for y in (-24.0, 16.0)], (1, 0, 0), 6.0, 6.0)
probe("holder back end <- chassis front edge", parts["holder base"], ch,
      [(x, -28.0, z) for x in (-8.0, 0.0, 8.0, 16.0)], (0, 1, 0), 6.0, 6.0)
probe("holder cover back end <- chassis front edge", parts["holder cover"], ch,
      [(-16.0, -28.0, z)], (0, 1, 0), 6.0, 6.0)
probe("steering frame plate <- chassis risers", parts["frame"], ch,
      [(s * 24.0, y, 23.9) for s in (1, -1) for y in (24.0, 32.0)], (0, 0, 1), 6.0, 6.0)
bx = [(x, y) for x in (-48.0, -40.0, -32.0) for y in range(-88, -15, 8)]
probe("battery box bottom <- chassis", parts["battery box"], ch, [(x, y, c.Z1) for x, y in bx], (0, 0, 1), 6.0, 6.0)
probe("battery box top -> deck", parts["deck"], parts["battery box"],
      [(x, y, c.DECK_Z0 - 0.3) for x, y in bx if y <= -20], (0, 0, 1), 6.0, 6.0)
mx = [(x, y) for cols in c.PIER_COLS for x in cols for y in (-56.0, -48.0, -40.0)]
probe("motor frame lower side <- chassis piers", parts["motor frame"], ch,
      [(x, y, c.PIER_TOP) for x, y in mx], (0, 0, 1), 6.0, 6.0)
for k, yc in (("post front", c.POSTS_Y[0]), ("post rear", c.POSTS_Y[1])):
    pts = [(48.0, yc + dy) for dy in (-4.0, 4.0)]
    probe(f"{k} <- chassis", parts[k], ch, [(x, y, c.Z1) for x, y in pts], (0, 0, 1), 6.0, 6.0)
    probe(f"{k} -> deck", parts["deck"], parts[k], [(x, y, c.DECK_Z0 - 0.3) for x, y in pts], (0, 0, 1), 6.0, 6.0)
bb = [(x, y) for x in (-48.0, 48.0) for y in range(-88, -23, 8)] + [(x, y) for x in range(-40, 41, 8) for y in (-88.0, -24.0)]
probe("breadboard frame <- deck", parts["breadboard frame"], parts["deck"],
      [(x, y, c.DECK_Z1) for x, y in bb], (0, 0, 1), 5.0, 6.0)

print("== 5. ground")
low = sorted((p.bounding_box().min.Z, k) for k, p in parts.items())
for zmin, k in low[:8]:
    print(f"   {k:22s} lowest z {zmin:6.2f}")
print("overall size:", parts["chassis"].bounding_box().size, )
allbb = bd.Compound(children=list(parts.values())).bounding_box()
print(f"car envelope x {allbb.min.X:.1f}..{allbb.max.X:.1f}  y {allbb.min.Y:.1f}..{allbb.max.Y:.1f}  z {allbb.min.Z:.1f}..{allbb.max.Z:.1f}")
print("ALL PASS" if ok else "FAILED")
