"""Car model checks. Run from car_model/:  python tmp/check_car.py

1. every pair of parts, straight ahead: no overlap; gaps under 1 mm listed
2. steering sweep (5 crank angles): the moving steering parts against everything else
3. differential: carrier + ring turned in steps, against the chassis (ring and carrier openings) and the parts
   beside it; ring/pinion mesh; pinion, 3L axle and coupler (4x the carrier) against the fixed parts
4. pin holes: steering frame plate -> chassis risers, motor frame -> chassis, posts -> chassis and battery tray,
   battery tray <-> breadboard frame (back to back) each line up with an open hole in the part they pin to
   (probe points along the shared axis)
5. ground clearance and wheel contact
6. assembly order on the chassis: motor (frame, motor, coupler) down; 3L axle in from behind through the pinion
   tower; pinion on from behind; differential down (turned into mesh: the ring/pinion pair is left out);
   rear axles in from outside; steering module down; posts down; breadboard + battery tray pair down
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
                "front tyre left", "front tyre right", "tie rod left", "tie rod right"}

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
fixed = {k: parts[k] for k in ("chassis", "inner sleeve left", "inner sleeve right", "motor frame", "tt motor",
                               "post rear left", "post rear right")}  # side gears: bench model
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
    # input line (turns 4x the carrier): pinion, 3L axle, coupler against the fixed parts (the coupler sits on the
    # TT shaft, so not against the motor)
    co = {k: dp[k] for k in ("pinion", "input axle", "coupler")}
    for (na, a), (nb, b) in itertools.product(co.items(), fixed.items()):
        if na == "coupler" and nb == "tt motor":
            continue
        v = overlap(a, b)
        if v > 1e-3:
            bad.append((na, nb, round(v, 3)))
    gap = min(dp[k].distance_to(fixed["chassis"]) for k in ("ring gear", "carrier"))
    ok &= not bad
    print(f"spin {spin:6.2f}: overlaps {bad or 'none'}   ring/carrier-chassis gap {gap:.2f}")

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


# steering module -> chassis: down through the steering frame plate into the arms and nose
probe("steering frame plate <- chassis risers", parts["frame"], ch,
      [(x, y, c.RISER_TOP + 0.2) for x, y in c.STEER_PINS], (0, 0, 1), 6.0, 6.0)
probe("motor frame lower side <- chassis", parts["motor frame"], ch,
      [(x, y, c.Z1) for x, y in c.MOTOR_PINS], (0, 0, 1), 6.0, 6.0)
mx = [(x, y) for x in range(-32, 33, 8) for y in (-56.0, -48.0, -40.0)]  # every hole of the frame's lower side
probe("  (all 27 frame side holes over chassis holes)", parts["motor frame"], ch,
      [(x, y, c.Z1) for x, y in mx], (0, 0, 1), 6.0, 6.0)
tray = parts["battery tray"]
for s, side in ((1, "right"), (-1, "left")):
    for r, where in enumerate(("front", "rear")):
        k = f"post {where} {side}"
        pts = [(s * 48.0, c.POSTS_Y[r] + dy) for dy in (-4.0, 4.0)]
        probe(f"{k} <- chassis", parts[k], ch, [(x, y, c.Z1) for x, y in pts], (0, 0, 1), 6.0, 6.0)
        probe(f"{k} -> battery tray", tray, parts[k], [(x, y, c.POST_TOP + 0.3) for x, y in pts], (0, 0, 1), 6.0, 6.0)
# back to back: the battery tray's bottom (up) into the breadboard tray's bottom
probe("battery tray <-> breadboard frame (back to back)", parts["breadboard frame"], tray,
      [(x, y, c.TOP_Z0 - 0.1) for x, y in c.STACK_PINS], (0, 0, 1), 6.0, 6.0)

print("== 5. ground")
low = sorted((p.bounding_box().min.Z, k) for k, p in parts.items())
for zmin, k in low[:8]:
    print(f"   {k:22s} lowest z {zmin:6.2f}")
allbb = bd.Compound(children=list(parts.values())).bounding_box()
print(f"car envelope x {allbb.min.X:.1f}..{allbb.max.X:.1f}  y {allbb.min.Y:.1f}..{allbb.max.Y:.1f}  z {allbb.min.Z:.1f}..{allbb.max.Z:.1f}")

print("== 6. assembly order on the chassis")
from car_assembly import body_parts, drive_parts, steering_parts  # noqa: E402
dr = drive_parts()
body_all = body_parts()
chassis_only = [body_all["chassis"]]
motor = [dr[k] for k in ("motor frame", "tt motor", "coupler")]
diff_group = [dr[k] for k in ("carrier", "ring gear", "cross pin", "spider top", "spider bottom", "side gear left",
                              "side gear right", "inner sleeve left", "inner sleeve right")]
steer = list(steering_parts().values())
posts = [v for k, v in body_all.items() if k.startswith("post")]
stack = [v for k, v in body_all.items() if k.startswith("battery") or k == "breadboard frame"]


def lift_clear(name, moving, fixed_parts, d=(0, 0, 1)):
    """Parts in place, moved away along d: nothing already there in the way (reversed, they go on along -d)."""
    global ok
    bad = []
    for t in (0.5, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0):
        for a in moving:
            am = bd.Pos(d[0] * t, d[1] * t, d[2] * t) * a
            for b in fixed_parts:
                v = overlap(am, b)
                if v > 1e-3:
                    bad.append((t, a.label, b.label, round(v, 2)))
    ok &= not bad
    print(f"{name:56s} {'OK' if not bad else bad[:6]}")


lift_clear("motor (frame + motor + coupler) down onto the chassis", motor, chassis_only)
lift_clear("3L axle in from behind through the pinion tower", [dr["input axle"]], chassis_only + motor, (0, -1, 0))
lift_clear("pinion on to the axle from behind", [dr["pinion"]], chassis_only + motor, (0, -1, 0))
lift_clear("differential down between the towers (turned into mesh)", diff_group,
           chassis_only + motor + [dr["input axle"]])
for s, side in ((-1, "left"), (1, "right")):
    lift_clear(f"rear axle {side} in from outside (collar in place)", [dr[f"rear axle {side}"]],
               chassis_only + diff_group + [dr[f"collar {side}"]], (s, 0, 0))
drive_all = list(dr.values())
lift_clear("steering module down onto the nose and arms", steer, chassis_only + drive_all)
lift_clear("posts down onto the chassis", posts, chassis_only + drive_all + steer)
lift_clear("breadboard + battery trays (pinned) onto the posts", stack, chassis_only + drive_all + steer + posts)
print("ALL PASS" if ok else "FAILED")
