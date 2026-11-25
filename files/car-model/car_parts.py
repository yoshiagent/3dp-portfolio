"""Car model: the motor-driven differential at the rear, the MG90S steering at the front, and on top the breadboard
tray pinned back to back to the two-cell 18650 tray. This file holds the new printed parts and the placements of the
reused ones.

Vehicle frame (same as ../../mg90s_steering): x right, y forward, z up from the ground, steering servo shaft on
the z axis. Front axle (kingpins) at y = 40; wheelbase 160 as the steering's Ackermann layout assumes, so the
rear axle is at y = -120.

Layout (2026-11-25 redesign, "simple and easy to assemble"):
* one printed chassis from the rear axle to the front axle (Technic plate z 13.3 - 21.1, 8 mm grid, x = 0, y = 0
  mod 8), with the axle towers, the pinion tower and, in front, two arms and a nose that rise to z 23.8 under
  the steering frame plate (z 24.2). No piers, no middle level.
* the TT motor in the 9L SideTop frame (../tt_motor_adapter, tt_motor_technic_SideTop_9L) lies on its side
  directly on the chassis, centred across the car (x -35.7 .. 35.7), pinned down with 2L pins. Its shaft is on
  the frame's hole column 16 mm from the frame centre, so it is at x = -16, 19.7 above the plate.
* the TT -> LEGO coupler on the shaft and a LEGO 3L cross axle go straight into the pinion, as in
  ../../electric_differential: the differential follows the motor shaft, 16 mm left of the centre line
  (DIFF_X) and with its axle 19.7 above the plate (the plate's height is set so that this is the wheels' 40.8).
  The rear axles are then LEGO 8L (left) and 12L (right); the wheels stay where they were.
* the steering module unchanged, pinned down into the arms and the nose.
* upper level unchanged: the breadboard tray (13 x 9 x 2) on top, pocket up, the two-cell 18650 tray turned over
  under it, bottoms pinned together, on four identical posts standing on the chassis.
* nothing on the rear axle line is printed: LEGO motorcycle tyres on their rims, cross axles, half bushes (the
  collars, inside the outer towers and outside the hubs) and bushes (the spacers). Their shapes here are
  stand-ins for the assembly and checks.

Units mm.
"""
from __future__ import annotations

import math
import sys
from functools import reduce
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "electric_differential" / "src"))

from cadgen import build123d as bd  # noqa: E402

import electric_parts as e  # noqa: E402
from electric_parts import d  # noqa: E402
from differential_parts import cross_bore, cyl, place, round_bore  # noqa: E402

# --- vehicle layout ---
REAR_Y = -120.0  # = YK - L of the steering linkage (40 - 160)
AXLE_Z = 40.8  # rear wheel radius: LEGO motorcycle tyre 81.6
PITCH = 8.0
PIN_D, CB_D, CB_DEPTH = e.PIN_D, e.CB_D, e.CB_DEPTH

# --- TT motor in the 9L frame, differential on its shaft ---
FRAME_STEM = "tt_motor_technic_SideTop_9L"
FRAME_HALF_L, FRAME_HALF_W = 35.7, 19.7  # 9L frame: 71.4 long, 39.4 wide (lying on its side: 39.4 high)
FRAME_SHAFT_X = 16.0  # shaft on the frame's hole column x = 16 (stop face 27.5)
DIFF_X = -FRAME_SHAFT_X  # differential centre and pinion axis: frame centred, its shaft 16 left of the centre
MOTOR_Y = REAR_Y - e.MOTOR_C_Y  # -48: frame centre (on the hole grid)
MOTOR_Y0, MOTOR_Y1 = MOTOR_Y - 11.9, MOTOR_Y + 11.9  # -59.9 .. -36.1
MOTOR_PINS = [(x, y) for x in (-32.0, 0.0, 32.0) for y in (MOTOR_Y - PITCH, MOTOR_Y + PITCH)]  # 6 x 2L pins

# --- rear axle line (|x|) ---
WHEEL_R, WHEEL_W = 40.8, 13.8  # LEGO motorcycle wheel: rim 2903 (61.6 x 13.8) in tyre 2902 (81.6 x 15)
RIM_R, TYRE_W = 30.8, 15.0
AXLE_N = {-1: 8, 1: 12}  # LEGO cross axle length per side: the differential is 16 (2 pitches) left
BUSH_L = 8.0  # LEGO bush between the outer tower and the wheel
OUTER_X0, OUTER_X1 = 46.0, 50.0  # outer axle towers
SPACER_X = (OUTER_X1 + 0.15, OUTER_X1 + 0.15 + BUSH_L)  # LEGO bush, tower -> wheel: no inward slide
WHEEL_X0 = SPACER_X[1] + 0.15  # 58.3: wheel inner face
WHEEL_X1 = WHEEL_X0 + WHEEL_W
COLLAR_L = 4.0
COLLAR_X = (OUTER_X0 - 0.15 - COLLAR_L, OUTER_X0 - 0.15)  # on the axle inside the outer tower: no outward slide
OUT_COLLAR_X = (WHEEL_X1 + 0.15, WHEEL_X1 + 0.15 + COLLAR_L)  # half bush outside the hub: the wheel stays on
TUBE_OD = d.SLEEVE_OD

# --- chassis: one Technic plate, the motor frame's lower side on its top ---
Z1 = AXLE_Z - FRAME_HALF_W  # 21.1: plate top = motor frame's lower side; shaft and axle 19.7 above it
Z0 = Z1 - 7.8  # 13.3
RISER_TOP = 23.8  # arms and nose under the steering frame plate (bottom 24.2)
HALF_W = 51.7
Y_REAR, Y_FRONT = -151.7, -12.3  # full-width part; at full lock the front wheels reach y -4.7
HOLDER_HALF, HOLDER_Y0 = 20.3, -28.3  # servo holder (|x| <= 19.7, y >= -27.7) hangs down to z 8 through here
ARM = (20.3, 27.7)  # |x| of the two arms beside the servo holder and bulkhead
NOSE = (28.3, 43.7)  # y: block under the steering frame plate, ahead of the bulkhead, up to the plate's front edge
STEER_PINS = [(x, y) for x in (-24.0, -16.0, -8.0, 0.0, 8.0, 16.0, 24.0) for y in (32.0, 40.0)] + [
    (-24.0, 24.0), (24.0, 24.0)]  # steering frame plate holes over the nose and arms (build_steering.py)
TOWER_TOP = AXLE_Z + d.TOWER_TOP
T = d.TOWER_X1 - d.TOWER_X0  # tower thickness 4
INNER = {s: tuple(sorted((DIFF_X + s * d.TOWER_X0, DIFF_X + s * d.TOWER_X1))) for s in (1, -1)}  # inner towers
OUTER = {s: tuple(sorted((s * OUTER_X0, s * OUTER_X1))) for s in (1, -1)}
PT_Y0, PT_Y1 = REAR_Y - e.PINION_TOWER_Y1, REAR_Y - e.PINION_TOWER_Y0  # pinion tower -85 .. -81
# openings in the plate: the 44-tooth ring (tips r 33.3, x DIFF_X - 15.4 .. - 5.6) and the carrier (r 22,
# x DIFF_X - 14 .. + 14) reach below the plate top, 19.7 under the axle
RING_SLOT = (DIFF_X - 17.0, DIFF_X - 4.0, REAR_Y - 29.0, REAR_Y + 29.0)
CARRIER_SLOT = (DIFF_X - 15.0, DIFF_X + 15.0, REAR_Y - 12.5, REAR_Y + 12.5)
EMBED = 1.0  # towers and ribs start this far inside the plate: a thin overlap left sliver faces in the mesh

# --- upper level: breadboard tray (13 x 9 x 2, 15.8) on top, pocket up; the two-cell 18650 tray of
# ../tt_motor_adapter (battery18650x2_technic, 13 x 9 x 3, 23.6, pocket 76 x 42.6 on a 7.8 floor) turned over under
# it, bottoms 0.2 apart and pinned together, centre y -48, clear of the ring gear (top 74.1 at y <= -86.7). The
# holder sticks 2.2 out of the turned-over tray, down over the motor frame (top 60.5), and the cells 2.5 more.
STACK_YC = -48.0
TRAY_FACE = 66.4  # underside of the pair (the battery tray's open face)
TRAY_H, TRAY_FLOOR = 23.6, 7.8
TRAY_C = (0.0, STACK_YC, TRAY_FACE + TRAY_H / 2)  # 78.2
TOP_Z0 = TRAY_FACE + TRAY_H + 0.2  # 90.2: breadboard frame bottom
BREADBOARD_C = (0.0, STACK_YC, TOP_Z0 + 7.9)  # frame 15.8 high, origin at its centre
HD2_L, HD2_W, HD_H = 76.0, 42.6, 18.0  # two-cell holder and its height
STACK_PINS = [(x, STACK_YC + y) for x in (-40.0, 40.0) for y in (-32.0, 32.0)] + [
    (-48.0, STACK_YC), (48.0, STACK_YC), (0.0, STACK_YC - 32.0), (0.0, STACK_YC + 32.0)]
# four identical posts at x = +-48 under the battery tray's ring holes (two each, y +-4 about the post)
POSTS_Y = (STACK_YC + 28.0, STACK_YC - 28.0)  # -20 (front), -76 (rear)
POST_X = (44.3, 51.7)  # |x|
POST_TOP = TRAY_FACE - 0.6  # 65.8


def to_vehicle(shape: bd.Part) -> bd.Part:
    """Differential coordinates (electric_parts, pinion axis x = 0) -> vehicle: turned 180 deg (motor ahead of the
    axle) and moved to DIFF_X."""
    return bd.Pos(DIFF_X, REAR_Y, AXLE_Z) * bd.Rot(0, 0, 180) * shape


def frame_location() -> bd.Location:
    """9L frame coordinates -> differential coordinates: shaft (frame x = 16) on the pinion axis, frame z (shaft
    out) toward the pinion, frame y down; the lower long side 19.7 under the axle."""
    return bd.Location(bd.Plane(origin=(-FRAME_SHAFT_X, e.MOTOR_C_Y, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0)))


def motor_frame() -> bd.Part:
    return to_vehicle(frame_location() * e._read(FRAME_STEM))


def tt_motor() -> bd.Part:
    """The electric differential's TT motor stand-in (not printed), 4 mm further toward the frame's front end:
    stop face 27.5 instead of 23.5, shaft at x = 16 instead of 12."""
    local = e.motor_frame_location().inverse() * e.tt_motor()
    return to_vehicle(frame_location() * bd.Pos(FRAME_SHAFT_X - e.MOTOR_FRAME_SHAFT_X, 0, 0) * local)


def box(x0, x1, y0, y1, z0, z1) -> bd.Part:
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def _perp(v):
    return (0, 0, 1) if abs(v[2]) < 0.9 else (1, 0, 0)


def hole(p0, direction, depth, cb_start=True, cb_end=False) -> bd.Part:
    """Pin hole tool from the face point p0 into the part along `direction`, counterbored at the face."""
    tool = cyl(PIN_D / 2, depth + 0.2, -0.1)
    if cb_start:
        tool += cyl(CB_D / 2, CB_DEPTH + 0.12, -0.1)
    if cb_end:
        tool += cyl(CB_D / 2, CB_DEPTH + 0.12, depth - CB_DEPTH - 0.02)
    return bd.Location(bd.Plane(origin=p0, x_dir=_perp(direction), z_dir=direction)) * tool


def vhole(x, y, z0, z1) -> bd.Part:
    return hole((x, y, z0), (0, 0, 1), z1 - z0, cb_start=True, cb_end=True)


def gusset(corner, run_dir, run, height, thick_dir, thick=T) -> bd.Part:
    """Rib standing on the plate top (corner z = Z1), its foot sunk EMBED into the plate; the part above the
    plate is the triangle run x height."""
    x, y, z = corner
    k = (height + EMBED) / height
    return d.gusset((x, y, z - EMBED), run_dir, run * k, height + EMBED, thick_dir, thick)


# --- chassis ---

def towers() -> tuple[bd.Part, list[tuple]]:
    """Inner axle towers (DIFF_X +- 21..25), outer axle towers (+-46..50), pinion tower, their ribs.
    Returns the solid and its footprints (to keep plate holes away). Rib heights keep under the axle line:
    collars and bushes reach down to z 36.3."""
    z = Z1
    parts, foot = [], []
    for s in (1, -1):
        for x0, x1 in (INNER[s], OUTER[s]):
            parts.append(box(x0, x1, REAR_Y - d.TOWER_HALF_W, REAR_Y + d.TOWER_HALF_W, z - EMBED, TOWER_TOP))
            for sy in (1, -1):  # ribs along y on both faces of every axle tower
                parts.append(gusset((x0, REAR_Y + sy * d.TOWER_HALF_W, z), (0, sy, 0), 8.0, 20.7, (1, 0, 0)))
            foot.append((x0, x1, REAR_Y - d.TOWER_HALF_W - 8, REAR_Y + d.TOWER_HALF_W + 8))
        i0, i1 = INNER[s]
        o0, o1 = OUTER[s]
        gap = (o0 - i1) if s > 0 else (i0 - o1)
        if gap < 10.0:  # the left inner tower stands 5 from the outer one: one low block joins them
            xa, xb = (i1, o0) if s > 0 else (o1, i0)
            parts.append(box(xa, xb, REAR_Y - T / 2, REAR_Y + T / 2, z - EMBED, z + 13.0))
            foot.append((xa, xb, REAR_Y - T / 2, REAR_Y + T / 2))
            continue
        # right: outward rib on the inner tower, inward rib on the outer tower, a low tie rib between them
        # (one continuous rib: two ribs ending on the same line left a seam the mesher could not close)
        xin = i1 if s > 0 else i0
        xout = o0 if s > 0 else o1
        parts.append(gusset((xin, REAR_Y - T / 2, z), (s, 0, 0), 8.0, 14.7, (0, 1, 0)))
        parts.append(gusset((xout, REAR_Y - T / 2, z), (-s, 0, 0), 8.0, 8.7, (0, 1, 0)))
        xa, xb = sorted((xin, xout))
        parts.append(box(xa, xb, REAR_Y - T / 2, REAR_Y + T / 2, z - EMBED, z + 5.0))
        foot.append((xa, xb, REAR_Y - T / 2, REAR_Y + T / 2))
    hw = e.PTOWER_HALF_W
    parts.append(box(DIFF_X - hw, DIFF_X + hw, PT_Y0, PT_Y1, z - EMBED, TOWER_TOP))
    parts.append(gusset((DIFF_X - T / 2, PT_Y1, z), (0, 1, 0), 8.0, 10.0, (1, 0, 0)))  # motor side, under the axle
    parts.append(gusset((DIFF_X - T / 2, PT_Y0, z), (0, -1, 0), 8.0, 6.0, (1, 0, 0)))  # pinion side, under it
    foot.append((DIFF_X - hw, DIFF_X + hw, PT_Y0 - 8, PT_Y1 + 8))
    return reduce(lambda a, b: a + b, parts), foot


def plate_outline() -> list[tuple]:
    """Plate parts (x0, x1, y0, y1), z Z0 .. Z1: the full-width rear and the two sides of the holder opening."""
    return [(-HALF_W, HALF_W, Y_REAR, HOLDER_Y0), (-HALF_W, -HOLDER_HALF, HOLDER_Y0, Y_FRONT),
            (HOLDER_HALF, HALF_W, HOLDER_Y0, Y_FRONT)]


def riser_outline() -> list[tuple]:
    """Arms and nose, z Z0 .. RISER_TOP: from the plate forward under the steering frame plate."""
    out = [(*sorted((s * ARM[0], s * ARM[1])), HOLDER_Y0, NOSE[0] + 1.0) for s in (1, -1)]
    out.append((-ARM[1], ARM[1], NOSE[0], NOSE[1]))
    return out


def chassis_grid(foot: list[tuple]) -> list[tuple[float, float, float]]:
    """Vertical holes (x, y, top z) on the 8 mm grid, whole hole and counterbore inside the outline, away from
    towers and the ring / carrier openings."""
    boxes = plate_outline() + riser_outline()
    m = CB_D / 2 + 0.6

    def inside(x, y, bxs):
        return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in bxs)

    pts = []
    for i in range(-6, 7):
        for j in range(-19, 6):
            x, y = i * PITCH, j * PITCH
            if not all(inside(x + dx, y + dy, boxes) for dx in (-3.7, 0, 3.7) for dy in (-3.7, 0, 3.7)):
                continue
            if any(x0 - m < x < x1 + m and y0 - m < y < y1 + m for x0, x1, y0, y1 in foot):
                continue
            pts.append((x, y, RISER_TOP if inside(x, y, riser_outline()) else Z1))
    return pts


def chassis() -> bd.Part:
    """The one printed chassis: plate, risers, towers; bearings, openings and the hole grid."""
    body = reduce(lambda a, b: a + b, [box(x0, x1, y0, y1, Z0, Z1) for x0, x1, y0, y1 in plate_outline()]
                  + [box(x0, x1, y0, y1, Z0, RISER_TOP) for x0, x1, y0, y1 in riser_outline()])
    tw, foot = towers()
    body = body.clean() + tw
    foot = foot + [RING_SLOT, CARRIER_SLOT]
    # bearings: rear axle line (all four axle towers), pinion line
    body -= bd.Pos(0, REAR_Y, AXLE_Z) * bd.Rot(0, 90, 0) * bd.Cylinder(d.BEARING_D / 2, 2 * HALF_W + 2)
    body -= bd.Pos(DIFF_X, (PT_Y0 + PT_Y1) / 2, AXLE_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(d.BEARING_D / 2, 20)
    for x0, x1, y0, y1 in (RING_SLOT, CARRIER_SLOT):
        body -= box(x0, x1, y0, y1, Z0 - 1, Z1 + 1)
    return body - bd.Compound([vhole(x, y, Z0, top) for x, y, top in chassis_grid(foot)])


def deck_post(yc: float, s: int = 1) -> bd.Part:
    """Post under the battery tray's edge (s = 1 right, -1 left), from the plate top up to 0.6 under the tray."""
    x0, x1 = sorted((s * POST_X[0], s * POST_X[1]))
    body = box(x0, x1, yc - 7.7, yc + 7.7, Z1, POST_TOP)
    tools = [vhole(s * 48.0, yc + dy, Z1, POST_TOP) for dy in (-4.0, 4.0)]
    return body - bd.Compound(tools)


# --- upper level: breadboard tray + two-cell 18650 tray back to back ---

def battery_tray_location() -> bd.Location:
    """Battery tray coordinates -> vehicle: turned over (about x), open face down at TRAY_FACE."""
    return bd.Pos(*TRAY_C) * bd.Rot(180, 0, 0)


def battery_tray() -> bd.Part:
    return battery_tray_location() * e._read("battery18650x2_technic")


def battery_holder() -> bd.Part:
    """Stand-in for a bought two-cell 18650 holder (not printed), in the turned-over tray, opening down."""
    z0 = -TRAY_H / 2 + TRAY_FLOOR
    shell = box(-HD2_L / 2, HD2_L / 2, -HD2_W / 2, HD2_W / 2, z0, z0 + HD_H)
    for y in (-HD2_W / 4, HD2_W / 4):
        shell -= box(-34.0, 34.0, y - 9.3, y + 9.3, z0 + 2.0, z0 + HD_H + 1)
    return battery_tray_location() * shell


def battery_cells() -> list[bd.Part]:
    """The two 18650 cells (18.5 x 65), side by side in the holder."""
    z0 = -TRAY_H / 2 + TRAY_FLOOR + 2.0 + 9.25
    return [battery_tray_location() * (bd.Pos(0, y, z0) * bd.Rot(0, 90, 0) * bd.Cylinder(9.25, 65.0))
            for y in (-HD2_W / 4, HD2_W / 4)]


# --- rear axle line ---

def knobby_tyre(r_out: float, r_in: float, w: float, knob_h: float, n: int, z0: float = 0.0) -> bd.Part:
    """Stand-in LEGO off-road tyre along +z (z0 .. z0 + w): a core with two staggered rows of tread blocks."""
    core = cyl(r_out - knob_h, w, z0) - cyl(r_in, w + 0.2, z0 - 0.1)
    gap = 1.2
    row_w = (w - gap) / 2
    block_w = 2 * math.pi * r_out / n * 0.55
    blocks = []
    for k in range(2):
        for i in range(n):
            b = bd.Box(knob_h + 0.6, block_w, row_w, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
            blocks.append(bd.Rot(0, 0, (i + 0.5 * k) * 360.0 / n) *
                          bd.Pos(r_out - knob_h - 0.3, 0, z0 + k * (row_w + gap)) * b)
    return core.fuse(*blocks).clean() & cyl(r_out, w, z0)


def rear_rim(s: int) -> bd.Part:
    """Stand-in for the LEGO motorcycle rim 2903 (61.6 x 13.8, not printed): rim ring, six spokes and the hub with
    the cross hole; its inner face against the spacer bush."""
    ring = cyl(RIM_R, WHEEL_W) - cyl(RIM_R - 3.8, WHEEL_W + 0.2, -0.1)
    spokes = [bd.Rot(0, 0, 60.0 * i) * bd.Pos(5.0, 0, WHEEL_W / 2) *
              bd.Box(RIM_R - 3.0 - 5.0, 4.0, 5.0, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.CENTER))
              for i in range(6)]
    rim = ring.fuse(cyl(6.0, WHEEL_W), *spokes).clean() - cross_bore(0, WHEEL_W)
    return place(rim, (s * WHEEL_X0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def rear_tyre(s: int) -> bd.Part:
    """Stand-in for the LEGO motorcycle tyre 2902 (81.6 x 15, not printed), centred on the rim."""
    t = knobby_tyre(WHEEL_R, RIM_R + 0.05, TYRE_W, 2.2, 26, -(TYRE_W - WHEEL_W) / 2)
    return place(t, (s * WHEEL_X0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def rear_axle(s: int) -> bd.Part:
    """LEGO cross axle (8L left, 12L right) from 2.5 off the cross pin to 10.2 past the wheel's outer face."""
    length = AXLE_N[s] * PITCH - 0.2
    return place(d.lego_axle(d.AXLE_IN, length), (DIFF_X, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def _tube(s: int, x0: float, x1: float, bore: bd.Part) -> bd.Part:
    return place(cyl(TUBE_OD / 2, x1 - x0) - bore, (s * x0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def spacer(s: int) -> bd.Part:
    x0, x1 = SPACER_X
    return _tube(s, x0, x1, round_bore(d.BEARING_D, 0, x1 - x0))


def collar(s: int, outer: bool = False) -> bd.Part:
    x0, x1 = OUT_COLLAR_X if outer else COLLAR_X
    return _tube(s, x0, x1, cross_bore(0, x1 - x0))


# --- reused parts, placed ---

def breadboard_frame() -> bd.Part:  # noqa: D103
    return bd.Pos(*BREADBOARD_C) * e._read("breadboard_technic")
