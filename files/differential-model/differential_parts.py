"""Open bevel-gear differential teaching model: part geometry (build123d).

Units mm. World frame: axles along X, pinion shaft along Y (crank at -Y), +Z up,
origin at the centre of the differential (spider/side-gear apex).

Straight bevel teeth use Tredgold's approximation: each tooth is the involute tooth of
the virtual spur gear on the back cone, lofted straight to the cone apex.
Pinion and spiders have few teeth, so pinion/ring and spider/side pairs use opposite
profile shifts (long-short addendum) to avoid undercut.

Every factory returns its part in the assembly frame; print_pose() turns a part so
its flat face is on the bed.
"""
from __future__ import annotations

import math
from pathlib import Path
from functools import reduce

from cadgen import build123d as bd

# --- gears ---
M = 1.5
PRESSURE_ANGLE = 22.5  # 22.5 deg lets the 11- and 10-tooth gears run without undercut
BACKLASH = 0.15  # tooth thinning at the outer pitch circle, per gear

RING_TEETH, PINION_TEETH = 33, 11  # 3:1
SIDE_TEETH, SPIDER_TEETH = 16, 10
RING_SHIFT, PINION_SHIFT = -0.2, 0.2
SIDE_SHIFT, SPIDER_SHIFT = -0.2, 0.2
RING_FACE = 7.0
SIDE_FACE = 4.5

# pitch cone angles (axes at 90 deg)
PINION_DELTA = math.degrees(math.atan2(PINION_TEETH, RING_TEETH))
RING_DELTA = 90.0 - PINION_DELTA
SPIDER_DELTA = math.degrees(math.atan2(SPIDER_TEETH, SIDE_TEETH))
SIDE_DELTA = 90.0 - SPIDER_DELTA

# --- carrier ---
CARRIER_IN = 11.0  # inner faces of both carrier plates at x = +/-11
PLATE_T = 3.0
PLATE_R = 22.0
POST_Z0, POST_Z1 = 14.5, 21.5  # posts hold the cross pin, above/below the spiders
POST_HALF_W = 4.0
PEG_D, PEG_LEN = 4.8, 3.0
SOCKET_D, SOCKET_DEPTH = 5.0, 3.2
CROSS_PIN_D, CROSS_PIN_FLAT = 4.0, 0.4
CROSS_PIN_HOLE_D = CROSS_PIN_D + 0.1  # snug in the posts
SPIDER_BORE_D = CROSS_PIN_D + 0.4  # spiders turn on the pin
CARRIER_BORE_D = 5.8  # carrier turns on the axles

# --- shafts ---
BEARING_D = 5.4  # round holes the LEGO axles turn in (axle 4.8)
# wheel axles: LEGO Technic cross axle, standard 4L length so a real LEGO axle fits too
LEGO_AXLE_D, LEGO_ARM_W = 4.8, 1.8
LEGO_AXLE_L = 4 * 8.0 - 0.2  # 4L
LEGO_CLEAR = 0.1  # per side, in the cross holes of the side gears and wheels
AXLE_IN = 2.5  # clear of the cross pin (|x| <= 2)
AXLE_OUT = AXLE_IN + LEGO_AXLE_L
INPUT_AXLE_L = 3 * 8.0 - 0.2  # input (pinion) shaft: LEGO 3L

# --- frame ---
BASE_Z0, BASE_Z1 = -34.0, -30.0
BASE_X, BASE_Y0, BASE_Y1 = 34.0, -42.0, 28.0  # front edge carries the pinion tower's outer gusset
AXLE_GUSSET_RUN, AXLE_GUSSET_H = 8.0, 20.0
INNER_GUSSET_RUN, INNER_GUSSET_H = 6.0, 8.0
OUTER_GUSSET_RUN, OUTER_GUSSET_H = 8.0, 20.0  # under the wheel (wheel bottom at z = -13)
PINION_GUSSET_RUN, PINION_GUSSET_H = 8.0, 10.0
TOWER_X0, TOWER_X1 = 21.0, 25.0  # axle towers at |x| in [21, 25]
TOWER_HALF_W, TOWER_TOP = 7.0, 7.0
PINION_TOWER_Y0, PINION_TOWER_Y1 = -33.0, -29.0
WHEEL_X0, WHEEL_X1, WHEEL_R = 27.5, 32.5, 13.0

GAP = 0.5

# --- axle sleeves: fill the gap between each tower and the carrier so it cannot slide ---
SLEEVE_OD = 9.0
SLEEVE_ID = BEARING_D  # loose on the axle; it only takes end thrust
SLEEVE_END_CLEAR = 0.15  # running clearance at each end


def deg(a: float) -> float:
    return math.radians(a)


def inv(a: float) -> float:
    return math.tan(a) - a


class BevelSpec:
    def __init__(self, teeth: int, delta: float, face: float, shift: float):
        self.teeth, self.delta, self.face, self.shift = teeth, delta, face, shift
        d = deg(delta)
        self.r = M * teeth / 2
        self.R = self.r / math.sin(d)  # cone distance
        self.h = self.R * math.cos(d)  # axial distance apex -> outer pitch circle
        self.rv = self.r / math.cos(d)  # virtual (back cone) pitch radius
        self.k = (self.R - face) / self.R  # inner/outer scale about the apex
        self.add = (1 + shift) * M
        self.ded = (1.25 - shift) * M
        self.t = bd.Vector(math.cos(d), 0, -math.sin(d))  # back-cone direction at the pitch point
        self.pitch_pt = bd.Vector(self.r, 0, self.h)
        self.vcenter = self.pitch_pt - self.t * self.rv  # virtual gear centre, on the axis

    def outer_root(self) -> bd.Vector:
        return self.pitch_pt - self.t * self.ded


RING = BevelSpec(RING_TEETH, RING_DELTA, RING_FACE, RING_SHIFT)
PINION = BevelSpec(PINION_TEETH, PINION_DELTA, RING_FACE, PINION_SHIFT)
SIDE = BevelSpec(SIDE_TEETH, SIDE_DELTA, SIDE_FACE, SIDE_SHIFT)
SPIDER = BevelSpec(SPIDER_TEETH, SPIDER_DELTA, SIDE_FACE, SPIDER_SHIFT)

# ring/pinion apex: where the pinion axis crosses the axle axis
RING_BACK_LOCAL_Z = 12.5  # ring back face = right carrier plate inner face (x = 11)
RING_APEX_X = CARRIER_IN + RING_BACK_LOCAL_Z


def virtual_tooth(spec: BevelSpec, n: int = 8) -> list[tuple[float, float]]:
    """One tooth of the virtual spur gear as (radius, angle) points, CCW, closed below the root."""
    a = deg(PRESSURE_ANGLE)
    rv = spec.rv
    rb = rv * math.cos(a)
    ra, rf = rv + spec.add, rv - spec.ded
    s = M * (math.pi / 2 + 2 * spec.shift * math.tan(a)) - BACKLASH
    hp = s / (2 * rv)

    def theta(r: float) -> float:
        return hp + inv(a) - inv(math.acos(rb / r))

    if 2 * ra * theta(ra) < 0.25 * M:
        raise ValueError(f"tooth tip thinner than 0.25 m: z={spec.teeth} shift={spec.shift}")
    r0 = max(rf, rb)
    radii = [r0 + (ra - r0) * i / n for i in range(n + 1)]
    th0 = theta(r0)
    sink = rf - 0.8  # overlap into the gear blank
    pts = [(sink, -th0), (rf, -th0)]
    pts += [(r, -theta(r)) for r in radii]
    pts += [(r, theta(r)) for r in reversed(radii)]
    pts += [(rf, th0), (sink, th0)]
    return pts


def bevel_gear(spec: BevelSpec, back_z: float, extend: bool = False) -> bd.Part:
    """Straight bevel gear, local frame: axis +Z, apex at the origin, teeth face the apex.

    The blank has a flat back at local z = back_z (the print bed). With extend, teeth and
    root cone run on along the cone down to that back face: shallow gears (cone angle
    under 45 deg) would otherwise print their outer tooth ends as downward ledges.
    No bore; callers cut it.
    """
    y = bd.Vector(0, 1, 0)
    outer = [spec.vcenter + spec.t * (r * math.cos(th)) + y * (r * math.sin(th)) for r, th in virtual_tooth(spec)]
    inner = [p * spec.k for p in outer]
    ro = spec.outer_root()
    ri = ro * spec.k
    if extend:
        tip_z = (spec.pitch_pt + spec.t * spec.add).Z
        outer = [p * (1.02 * back_z / tip_z) for p in outer]
        back = ro * (back_z / ro.Z)
        profile = [bd.Vector(0, 0, ri.Z), bd.Vector(ri.X, 0, ri.Z), bd.Vector(back.X, 0, back_z), bd.Vector(0, 0, back_z)]
    else:
        profile = [
            bd.Vector(0, 0, ri.Z),
            bd.Vector(ri.X, 0, ri.Z),
            bd.Vector(ro.X, 0, ro.Z),
            bd.Vector(ro.X, 0, back_z),
            bd.Vector(0, 0, back_z),
        ]
    tooth = bd.Solid.make_loft(
        [bd.Wire.make_polygon(outer, close=True), bd.Wire.make_polygon(inner, close=True)], ruled=True
    )
    if extend:
        tooth = tooth & (bd.Pos(0, 0, back_z - 100) * bd.Box(200, 200, 200))
    teeth = [bd.Rot(0, 0, i * 360 / spec.teeth) * tooth for i in range(spec.teeth)]
    blank = bd.revolve(bd.Face(bd.Wire.make_polygon(profile, close=True)), bd.Axis.Z)
    return reduce(lambda a, b: a + b, teeth, blank)


def cyl(r: float, h: float, z: float = 0.0) -> bd.Part:
    return bd.Pos(0, 0, z) * bd.Cylinder(r, h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def d_profile(d: float, flat: float) -> bd.Face:
    """D section in local XY, flat facing +X."""
    r = d / 2
    return bd.Face(bd.Wire.make_circle(r)) - bd.Pos(r - flat + r, 0) * bd.Rectangle(2 * r, 4 * r)


def cross_profile(d: float, arm_w: float) -> bd.Face:
    """LEGO Technic cross section in local XY: two arms, tips rounded to the diameter d."""
    arms = bd.Rectangle(d + 2, arm_w) + bd.Rectangle(arm_w, d + 2)
    return bd.Face(bd.Wire.make_circle(d / 2)) & arms


def cross_bore(z0: float, z1: float) -> bd.Part:
    c = 2 * LEGO_CLEAR
    return bd.Pos(0, 0, z0 - 0.1) * bd.extrude(cross_profile(LEGO_AXLE_D + c, LEGO_ARM_W + c), z1 - z0 + 0.2)


def round_bore(d: float, z0: float, z1: float) -> bd.Part:
    return cyl(d / 2, z1 - z0 + 0.2, z0 - 0.1)


def place(shape: bd.Part, origin, x_dir, z_dir) -> bd.Part:
    return bd.Location(bd.Plane(origin=origin, x_dir=x_dir, z_dir=z_dir)) * shape


# --- parts in the assembly frame ---

def side_gear(side: int) -> bd.Part:
    """side = +1 (right) or -1 (left). A tooth faces +Z, where it meets the top spider's space."""
    g = bevel_gear(SIDE, back_z=CARRIER_IN)  # back face bears on the carrier plate
    g -= cross_bore(0, CARRIER_IN)
    return place(g, (0, 0, 0), (0, 0, 1), (side, 0, 0))


def spider(side: int) -> bd.Part:
    """side = +1 (top) or -1 (bottom); half a pitch turned so a space faces the side gears."""
    g = bevel_gear(SPIDER, back_z=POST_Z0, extend=True)  # back face bears on the post
    g &= cyl(CARRIER_IN - 0.5, POST_Z0 + 1)  # extended teeth must clear the carrier plates
    g -= round_bore(SPIDER_BORE_D, 0, POST_Z0)
    g = bd.Rot(0, 0, 180 / SPIDER_TEETH) * g
    return place(g, (0, 0, 0), (1, 0, 0), (0, 0, side))


def ring_half() -> bd.Part:
    """Ring gear plus right carrier plate: flat back at x = 11, sockets for the posts."""
    g = bevel_gear(RING, back_z=RING_BACK_LOCAL_Z)
    g -= round_bore(CARRIER_BORE_D, 0, RING_BACK_LOCAL_Z)
    # local +Y = world +Z in this placement
    for s in (1, -1):
        g -= bd.Pos(0, s * (POST_Z0 + POST_Z1) / 2, RING_BACK_LOCAL_Z - SOCKET_DEPTH) * cyl(SOCKET_D / 2, SOCKET_DEPTH + 0.1)
    return place(g, (RING_APEX_X, 0, 0), (0, -1, 0), (-1, 0, 0))


def pinion() -> bd.Part:
    g = bevel_gear(PINION, back_z=-PINION_TOWER_Y1, extend=True)  # back face bears on the tower
    g = bd.Rot(0, 0, 180 / PINION_TEETH) * g  # teeth half a pitch round; the cross hole stays square to the frame
    g -= cross_bore(PINION.outer_root().Z * PINION.k - 1.0, -PINION_TOWER_Y1)
    return place(g, (RING_APEX_X, 0, 0), (-1, 0, 0), (0, -1, 0))


def carrier_left() -> bd.Part:
    """Left plate with the two posts that carry the cross pin and plug into the ring half."""
    plate = place(cyl(PLATE_R, PLATE_T), (-CARRIER_IN - PLATE_T, 0, 0), (0, 0, 1), (1, 0, 0))
    body = plate
    for s in (1, -1):
        zc = s * (POST_Z0 + POST_Z1) / 2
        post = bd.Pos(0, 0, zc) * bd.Box(2 * CARRIER_IN, 2 * POST_HALF_W, POST_Z1 - POST_Z0)
        peg = place(cyl(PEG_D / 2, PEG_LEN), (CARRIER_IN, 0, zc), (0, 0, 1), (1, 0, 0))
        body = body + post + peg
    body -= place(round_bore(CARRIER_BORE_D, 0, PLATE_T), (-CARRIER_IN - PLATE_T, 0, 0), (0, 0, 1), (1, 0, 0))
    body -= bd.Pos(0, 0, -POST_Z1 - 1) * cyl(CROSS_PIN_HOLE_D / 2, 2 * POST_Z1 + 2)
    return body


def cross_pin() -> bd.Part:
    length = 2 * POST_Z1
    return bd.Pos(0, 0, -POST_Z1) * bd.extrude(d_profile(CROSS_PIN_D, CROSS_PIN_FLAT), length)


def lego_axle(z0: float, length: float) -> bd.Part:
    """LEGO cross axle along local +Z from z0.

    A 0.2 mm flat on the +X arm tip, which prints face-down: a rounded tip only touches the bed on a line.
    """
    section = cross_profile(LEGO_AXLE_D, LEGO_ARM_W) - bd.Pos(LEGO_AXLE_D / 2 - 0.2 + 5, 0) * bd.Rectangle(10, 10)
    return bd.Pos(0, 0, z0) * bd.extrude(section, length)


def axle(side: int) -> bd.Part:
    return place(lego_axle(AXLE_IN, LEGO_AXLE_L), (0, 0, 0), (0, 0, 1), (side, 0, 0))


# The crank's geometry is edited in FreeCAD (../freecad/crank.FCStd, sizes in its Params sheet);
# read_crank.py exports it to ../freecad/out/crank.step, which crank() reads.
FREECAD_CRANK_STEP = Path(__file__).resolve().parents[1] / "freecad" / "out" / "crank.step"
CRANK_HUB_H = 6.0  # must match hub_h in crank.FCStd: the input shaft ends flush with the hub
CRANK_IN_Y = PINION_TOWER_Y0 - GAP  # crank inner face, 0.5 mm behind the pinion tower


def pinion_shaft() -> bd.Part:
    """LEGO 3L cross axle, flush with the crank's outer face; its inner end passes through the pinion."""
    y_out = -CRANK_IN_Y + CRANK_HUB_H
    return place(lego_axle(y_out - INPUT_AXLE_L, INPUT_AXLE_L), (RING_APEX_X, 0, 0), (-1, 0, 0), (0, -1, 0))


def crank() -> bd.Part:
    """The FreeCAD crank, placed in the assembly.

    Its own frame is the print frame: arm, hub and handle start on the bed (z = 0), which faces the tower.
    Assembly: local +Z -> world -Y, arm (local +X) up.
    """
    from cadgen import read_step

    c = read_step(str(FREECAD_CRANK_STEP))
    return place(c, (RING_APEX_X, CRANK_IN_Y, 0), (0, 0, 1), (0, -1, 0))


def sleeve_span(side: int) -> tuple[float, float]:
    """|x| range of the sleeve: from the carrier's outer face to the tower's inner face, less clearance.

    Left: the left carrier plate's outer face. Right: the ring gear's flat centre face.
    """
    if side < 0:
        carrier_face = CARRIER_IN + PLATE_T
    else:
        carrier_face = RING_APEX_X - RING.outer_root().Z * RING.k
    return carrier_face + SLEEVE_END_CLEAR, TOWER_X0 - SLEEVE_END_CLEAR


def sleeve(side: int) -> bd.Part:
    x0, x1 = sleeve_span(side)
    tube = cyl(SLEEVE_OD / 2, x1 - x0) - round_bore(SLEEVE_ID, 0, x1 - x0)
    return place(tube, (side * x0, 0, 0), (0, 0, 1), (side, 0, 0))


def wheel(side: int) -> bd.Part:
    """Wheel with a raised arrow so students can see each axle turn."""
    w = cyl(WHEEL_R, WHEEL_X1 - WHEEL_X0)
    w -= bd.Pos(0, 0, WHEEL_X1 - WHEEL_X0 - 1.0) * (cyl(WHEEL_R - 1.5, 1.2) - cyl(4.5, 1.2))
    # arrow from r = 3 to r = 11, inside the recess (r < 11.5)
    arrow = bd.Pos(6.0, 0, WHEEL_X1 - WHEEL_X0 - 1.0) * bd.extrude(
        bd.Polygon((-3, -1.2), (2, -1.2), (2, -3), (5, 0), (2, 3), (2, 1.2), (-3, 1.2), align=None), 1.0
    )
    w = w + arrow - cross_bore(0, WHEEL_X1 - WHEEL_X0)
    return place(w, (side * WHEEL_X0, 0, 0), (0, 0, 1), (side, 0, 0))


def frame() -> bd.Part:
    base = bd.Pos(0, (BASE_Y0 + BASE_Y1) / 2, (BASE_Z0 + BASE_Z1) / 2) * bd.Box(2 * BASE_X, BASE_Y1 - BASE_Y0, BASE_Z1 - BASE_Z0)
    tower_h = TOWER_TOP - BASE_Z1
    body = base
    for s in (1, -1):
        xc = s * (TOWER_X0 + TOWER_X1) / 2
        tower = bd.Pos(xc, 0, BASE_Z1 + tower_h / 2) * bd.Box(TOWER_X1 - TOWER_X0, 2 * TOWER_HALF_W, tower_h)
        body += tower
        body -= place(round_bore(BEARING_D, 0, TOWER_X1 - TOWER_X0), (s * TOWER_X0, 0, 0), (0, 0, 1), (s, 0, 0))
    ptower = bd.Pos(RING_APEX_X, (PINION_TOWER_Y0 + PINION_TOWER_Y1) / 2, BASE_Z1 + tower_h / 2) * bd.Box(
        2 * TOWER_HALF_W, PINION_TOWER_Y1 - PINION_TOWER_Y0, tower_h
    )
    body += ptower
    body += reduce(lambda a, b: a + b, tower_gussets())
    body -= place(round_bore(BEARING_D, 0, PINION_TOWER_Y1 - PINION_TOWER_Y0), (RING_APEX_X, PINION_TOWER_Y1, 0), (1, 0, 0), (0, -1, 0))
    return body


def gusset(corner, run_dir, run: float, height: float, thick_dir, thick: float) -> bd.Part:
    """Right-triangle rib standing on the base: from the tower face at `corner`, `run` out along
    run_dir and `height` up the tower; `thick` wide along thick_dir starting at corner."""
    c = bd.Vector(*corner)
    pts = [c, c + bd.Vector(*run_dir) * run, c + bd.Vector(0, 0, height)]
    return bd.extrude(bd.Face(bd.Wire.make_polygon(pts, close=True)), amount=thick, dir=thick_dir)


def tower_gussets() -> list[bd.Part]:
    """Ribs that brace the three towers to the base, kept clear of every moving part:
    front/back of the axle towers (pinion stays at y < -17), a low inner rib under the
    carrier (plates and ring stay above z = -22 there), an outer rib under each wheel, and
    low ribs either side of the pinion tower (pinion and crank sweep stay above z = -18)."""
    z = BASE_Z1
    t = TOWER_X1 - TOWER_X0
    ribs = []
    for s in (1, -1):
        x_lo = min(s * TOWER_X0, s * TOWER_X1)
        for sy in (1, -1):
            ribs.append(gusset((x_lo, sy * TOWER_HALF_W, z), (0, sy, 0), AXLE_GUSSET_RUN, AXLE_GUSSET_H, (1, 0, 0), t))
        ribs.append(gusset((s * TOWER_X0, -t / 2, z), (-s, 0, 0), INNER_GUSSET_RUN, INNER_GUSSET_H, (0, 1, 0), t))
        ribs.append(gusset((s * TOWER_X1, -t / 2, z), (s, 0, 0), OUTER_GUSSET_RUN, OUTER_GUSSET_H, (0, 1, 0), t))
    px = RING_APEX_X - t / 2
    ribs.append(gusset((px, PINION_TOWER_Y0, z), (0, -1, 0), PINION_GUSSET_RUN, PINION_GUSSET_H, (1, 0, 0), t))
    ribs.append(gusset((px, PINION_TOWER_Y1, z), (0, 1, 0), PINION_GUSSET_RUN, PINION_GUSSET_H, (1, 0, 0), t))
    return ribs


def print_pose(shape: bd.Part, x_dir, z_dir) -> bd.Part:
    """Re-orient a part: z_dir (world) becomes print up; then drop it onto z = 0."""
    loc = bd.Location(bd.Plane(origin=(0, 0, 0), x_dir=x_dir, z_dir=z_dir)).inverse()
    s = loc * shape
    bb = s.bounding_box()
    return bd.Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s
