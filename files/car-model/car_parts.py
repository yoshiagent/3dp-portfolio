"""Car model: the motor-driven differential at the rear, the MG90S steering at the front, 3xAAA battery box and
breadboard. This file holds the new printed parts and the placements of the reused ones.

Vehicle frame (same as ../../mg90s_steering): x right, y forward, z up from the ground, steering servo shaft on
the z axis. Front axle (kingpins) at y = 40; wheelbase 160 as the steering's Ackermann layout assumes, so the
rear axle is at y = -120.

Why the rear is different from the bench model: the 44-tooth ring reaches 33.3 mm below the axle, so the rear
wheels grow to dia. 80 (axle 40 above the ground) and the plate gets a slot the ring turns in (ring 6.7 mm
above the ground). The front keeps the LEGO 56 x 26 wheels and the steering module unchanged.

Modules (each is assembled on its own, then joined with LEGO Technic 2L pins, all vertical, from above)
* drive module: drive base (Technic plate z 8 - 15.8, 8 mm grid, x = 0, y = 0 mod 8, the lowest level at the
  rear) with the differential's towers, outer axle towers and motor piers; carries the TT motor in its frame,
  coupler, differential, rear axles, collars, spacers and rear wheels.
* steering module: the MG90S steering exactly as in ../../mg90s_steering (servo holder hanging from the steering
  frame's bulkhead, frame plate z 24.2 - 31.6 out to the kingpins).
* chassis (middle level, plate z 16.0 - 23.8): stacked on the drive base's front edge and left side, and under
  the steering frame plate. Pins go down through the chassis into the drive base, and down through the steering
  frame plate into the chassis. Battery box and front post stand on the chassis.
* upper level: the breadboard frame itself (its bottom is a full 13 x 9 Technic grid), pinned straight onto the
  battery box top (stood on edge, lid outward) and two posts (front post on the chassis, rear post on the drive
  base: the motor leaves no chassis there). No separate deck plate.
* rear wheels dia. 80 x 24. The rear axle line's 10L axles, collars (= LEGO half bush, 4 mm, cross hole) and
  spacers (= LEGO bush, 8 mm) are LEGO standard parts; their shapes here are stand-ins, not printed.

Why three levels: the TT motor frame's lower side is 20.3 above the ground, so the drive base must stay at the
bottom (z 8 - 15.8) and the chassis that joins it to the steering goes one level up, clear of the motor.

Units mm.
"""
from __future__ import annotations

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
AXLE_Z = 40.0  # rear wheel radius
PITCH = 8.0
PIN_D, CB_D, CB_DEPTH = e.PIN_D, e.CB_D, e.CB_DEPTH

# --- rear axle line (|x|) ---
WHEEL_R, WHEEL_W = 40.0, 24.0
AXLE_10L = 10 * PITCH - 0.2
AXLE_END = d.AXLE_IN + AXLE_10L  # 82.3: flush with the wheel's outer face
WHEEL_X0, WHEEL_X1 = AXLE_END - WHEEL_W, AXLE_END
OUTER_X0, OUTER_X1 = 46.0, 50.0  # outer axle towers
SPACER_X = (OUTER_X1 + 0.15, WHEEL_X0 - 0.15)  # tower -> wheel: the wheel cannot move inward
COLLAR_L = 4.0
COLLAR_X = (OUTER_X0 - 0.15 - COLLAR_L, OUTER_X0 - 0.15)  # on the axle inside the outer tower: no outward slide
TUBE_OD = d.SLEEVE_OD

# --- drive base (lowest level, rear) ---
Z0, Z1 = 8.0, 15.8
HALF_W = 51.7
Y_REAR, Y_FRONT = -151.7, -12.3
HOLDER_HALF, HOLDER_Y0 = 20.3, -28.3  # holder +0.6 all round (holder |x| <= 19.7, y >= -27.7)
SLOT = (-17.0, -4.0, REAR_Y - 24.5, REAR_Y + 24.5)  # the ring (x -15.4 .. -5.6) turns through it
TOWER_TOP = AXLE_Z + d.TOWER_TOP
TIE_RIB_H = 8.0  # low rib joining each inner axle tower to the outer one
T = d.TOWER_X1 - d.TOWER_X0  # tower thickness 4
PT_Y0, PT_Y1 = REAR_Y - e.PINION_TOWER_Y1, REAR_Y - e.PINION_TOWER_Y0  # pinion tower -85 .. -81
MOTOR_Y = REAR_Y - e.MOTOR_C_Y  # -48
PIER_TOP = AXLE_Z + e.MOTOR_FRAME_Z  # 20.3: motor frame's lower side
PIER_COLS = tuple(sorted((-c1, -c0) for c0, c1 in e.PIER_COLS))  # (-24,-16), (0,8), (40,48)
MOTOR_X = (-27.7, 51.7)  # motor frame extent
MOTOR_Y0 = MOTOR_Y - 11.9  # -59.9 .. -36.1: motor frame (and its piers) along y
MOTOR_Y1 = MOTOR_Y + 11.9

# --- chassis (middle level): one Technic plate stacked on the drive base, under the steering frame plate ---
MID_Z0, MID_Z1 = Z1 + 0.2, Z1 + 8.0  # 16.0 .. 23.8; steering frame plate bottom 24.2 (0.4 above)
MID_FRONT = (-35.7, -12.3)  # y: full-width part over the drive base's front edge, 0.4 ahead of the motor frame;
                            # at full lock the front wheels reach y -4.7 (|x| >= 36.9) at this height
RAIL = (-51.7, -28.3, -99.7, -35.7)  # x0, x1, y0, y1: left rail over the drive base, beside the motor frame
                                     # (0.6 off x -27.7), ends clear of the left outer tower's ribs (y -105)
ARM = (20.3, 27.7)  # |x| of the two arms beside the servo holder and bulkhead (inner edge 0.6 off them)
NOSE = (28.3, 43.7)  # y: block under the steering frame plate, ahead of the bulkhead (y <= 27.7) up to the
                     # plate's front edge; |x| <= ARM[1], well inside the knuckles' lower ears (|x| >= 36.3)
STEER_PINS = [(x, y) for x in (-24.0, -16.0, -8.0, 0.0, 8.0, 16.0, 24.0) for y in (32.0, 40.0)] + [
    (-24.0, 24.0), (24.0, 24.0)]  # steering frame plate holes over the chassis (plate holes in build_steering.py)

# --- electronics ---
BATT_C = (-40.0, -52.0, MID_Z1 + 27.7)  # box stood on its long edge, lid facing -x, on the chassis rail
TOP_Z0 = MID_Z1 + 55.4 + 0.6  # 79.8: breadboard frame bottom, one pitch above the box top
POSTS_Y = (-28.0, -84.0)  # posts at x = 48, two holes each (y +/- 4), under the frame's bottom holes (y -88..-24)
POSTS_Z0 = (MID_Z1, Z1)  # front post on the chassis, rear post on the drive base
POST_X = (44.3, 51.7)
# pin rows under the battery box (x -48, -40, -32): the box and the chassis->drive base pins take turns
BATT_PIN_ROWS = (-88.0, -72.0, -56.0, -40.0, -24.0)
RAIL_PIN_ROWS = (-96.0, -80.0, -64.0, -48.0, -32.0, -16.0)
BREADBOARD_C = (0.0, -56.0, TOP_Z0 + 7.9)  # frame 15.8 high, origin at its centre


def to_vehicle(shape: bd.Part) -> bd.Part:
    """Differential coordinates (electric_parts) -> vehicle: turned 180 deg (motor ahead of the axle)."""
    return bd.Pos(0, REAR_Y, AXLE_Z) * bd.Rot(0, 0, 180) * shape


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


EMBED = 1.0  # towers, ribs, piers and risers start this far inside the plate: a thin overlap left 0.01 mm
             # sliver faces that meshed with open edges


def gusset(corner, run_dir, run, height, thick_dir, thick=T) -> bd.Part:
    """Rib standing on the plate top (corner z = Z1), its foot sunk EMBED into the plate; the part above the
    plate is the triangle run x height."""
    x, y, z = corner
    k = (height + EMBED) / height
    return d.gusset((x, y, z - EMBED), run_dir, run * k, height + EMBED, thick_dir, thick)


# --- chassis ---

def towers() -> tuple[bd.Part, list[tuple]]:
    """Inner axle towers (as in the bench model), outer axle towers, pinion tower, their ribs.
    Returns the solid and its footprints (to keep plate holes away)."""
    z = Z1
    parts, foot = [], []
    for s in (1, -1):
        for x_in, x_out, inner_rib in ((d.TOWER_X0, d.TOWER_X1, (6.0, 8.0)), (OUTER_X0, OUTER_X1, (8.0, 14.0))):  # outer rib 14 high: clear of the axle collar
            x_lo = min(s * x_in, s * x_out)
            parts.append(box(x_lo, x_lo + T, REAR_Y - d.TOWER_HALF_W, REAR_Y + d.TOWER_HALF_W, z - EMBED, TOWER_TOP))
            for sy in (1, -1):
                parts.append(gusset((x_lo, REAR_Y + sy * d.TOWER_HALF_W, z), (0, sy, 0), 8.0, 26.0, (1, 0, 0)))
            foot.append((x_lo, x_lo + T, REAR_Y - d.TOWER_HALF_W - 8, REAR_Y + d.TOWER_HALF_W + 8))
            # inward rib; none under the ring (vehicle -x side of the inner towers)
            if not (x_in == d.TOWER_X0 and s < 0):
                run, h = inner_rib
                parts.append(gusset((s * x_in, REAR_Y - T / 2, z), (-s, 0, 0), run, h, (0, 1, 0)))
                foot.append((min(s * x_in, s * (x_in - run)), max(s * x_in, s * (x_in - run)), REAR_Y - T / 2, REAR_Y + T / 2))
        # outward rib of the inner tower, and a low tie rib on to the outer tower (one continuous rib: two ribs
        # ending on the same line left a seam the mesher could not close)
        parts.append(gusset((s * d.TOWER_X1, REAR_Y - T / 2, z), (s, 0, 0), 8.0, 20.0, (0, 1, 0)))
        xa, xb = sorted((s * d.TOWER_X1, s * OUTER_X0))
        parts.append(box(xa, xb, REAR_Y - T / 2, REAR_Y + T / 2, z - EMBED, z + TIE_RIB_H))
        foot.append((xa, xb, REAR_Y - T / 2, REAR_Y + T / 2))
    hw = e.PTOWER_HALF_W
    parts.append(box(-hw, hw, PT_Y0, PT_Y1, z - EMBED, TOWER_TOP))
    parts.append(gusset((-T / 2, PT_Y1, z), (0, 1, 0), 8.0, 12.0, (1, 0, 0)))  # motor side
    parts.append(gusset((-T / 2, PT_Y0, z), (0, -1, 0), 8.0, 10.0, (1, 0, 0)))  # pinion side
    foot.append((-hw, hw, PT_Y0 - 8, PT_Y1 + 8))
    return reduce(lambda a, b: a + b, parts), foot


def piers() -> tuple[bd.Part, list[tuple]]:
    out, foot = [], []
    y0, y1 = MOTOR_Y - 11.8, MOTOR_Y + 11.8
    spans = []  # piers under the frame's hole columns; neighbours closer than 10 mm join into one block
    for c0, c1 in PIER_COLS:
        x0, x1 = max(c0 - 4.0, MOTOR_X[0]), min(c1 + 4.0, MOTOR_X[1])
        if spans and x0 - spans[-1][1] < 10.0:
            spans[-1] = (spans[-1][0], x1)
        else:
            spans.append((x0, x1))
    for x0, x1 in spans:
        out.append(box(x0, x1, y0, y1, Z1 - EMBED, PIER_TOP))
        foot.append((x0, x1, y0, y1))
    return reduce(lambda a, b: a + b, out), foot


def base_grid(foot: list[tuple]) -> list[tuple[float, float]]:
    """Vertical holes of the drive base on the 8 mm grid, away from towers and piers."""
    m = CB_D / 2 + 0.6
    pts = []
    for i in range(-6, 7):
        x = i * PITCH
        for j in range(-19, -1):
            y = j * PITCH
            if not (abs(x) <= HALF_W - 3.7 and Y_REAR + 3.7 <= y <= Y_FRONT - 3.7):
                continue
            if abs(x) < HOLDER_HALF + 3.7 and y > HOLDER_Y0 - 3.7:
                continue
            if any(x0 - m < x < x1 + m and y0 - m < y < y1 + m for x0, x1, y0, y1 in foot):
                continue
            pts.append((x, y))
    return pts


def drive_base() -> bd.Part:
    """Drive module base: the old one-piece chassis's rear, without the parts that held the steering."""
    body = box(-HALF_W, HALF_W, Y_REAR, Y_FRONT, Z0, Z1)
    body -= box(-HOLDER_HALF, HOLDER_HALF, HOLDER_Y0, Y_FRONT + 1, Z0 - 1, Z1 + 1)
    tw, foot = towers()
    pr, pfoot = piers()
    body = body + tw + pr
    foot += pfoot + [SLOT]
    # bearings: rear axle line (all four axle towers), pinion line
    body -= bd.Pos(0, REAR_Y, AXLE_Z) * bd.Rot(0, 90, 0) * bd.Cylinder(d.BEARING_D / 2, 2 * HALF_W + 2)
    body -= bd.Pos(0, (PT_Y0 + PT_Y1) / 2, AXLE_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(d.BEARING_D / 2, 20)
    body -= box(SLOT[0], SLOT[1], SLOT[2], SLOT[3], Z0 - 1, Z1 + 1)
    tools = [vhole(x, y, Z0, Z1) for x, y in base_grid(foot)]
    for c0, c1 in PIER_COLS:  # through pier and plate, under the motor frame's side holes
        for x in (c0, c1):
            for y in (MOTOR_Y - PITCH, MOTOR_Y, MOTOR_Y + PITCH):
                tools.append(vhole(x, y, Z0, PIER_TOP))
    return body - bd.Compound(tools)


def chassis_outline() -> list[tuple]:
    """Chassis plate as boxes (x0, x1, y0, y1): front part, left rail, two arms, nose."""
    out = [(-HALF_W, -HOLDER_HALF, MID_FRONT[0], MID_FRONT[1]), (HOLDER_HALF, HALF_W, MID_FRONT[0], MID_FRONT[1]),
           (-HOLDER_HALF, HOLDER_HALF, MID_FRONT[0], HOLDER_Y0), RAIL[:2] + RAIL[2:]]
    for s in (1, -1):
        out.append((*sorted((s * ARM[0], s * ARM[1])), MID_FRONT[1] - 1.0, NOSE[0] + 1.0))
    out.append((-ARM[1], ARM[1], NOSE[0], NOSE[1]))
    return out


def chassis_grid() -> list[tuple[float, float]]:
    """Every 8 mm grid point at least 3.7 inside the chassis outline (whole hole and counterbore in the plate)."""
    boxes = chassis_outline()

    def inside(x, y):
        return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in boxes)

    pts = []
    for i in range(-6, 7):
        for j in range(-13, 6):
            x, y = i * PITCH, j * PITCH
            if all(inside(x + dx, y + dy) for dx in (-3.7, 0, 3.7) for dy in (-3.7, 0, 3.7)):
                pts.append((x, y))
    return pts


def chassis() -> bd.Part:
    """Middle level: joins the drive module (below) to the steering module (above), carries the battery box."""
    body = reduce(lambda a, b: a + b, [box(x0, x1, y0, y1, MID_Z0, MID_Z1) for x0, x1, y0, y1 in chassis_outline()])
    body = body.clean()
    return body - bd.Compound([vhole(x, y, MID_Z0, MID_Z1) for x, y in chassis_grid()])


def deck_post(yc: float, z0: float) -> bd.Part:
    """Post under the breadboard frame's right edge, as high as the battery box."""
    body = box(*POST_X, yc - 7.7, yc + 7.7, z0, TOP_Z0 - 0.6)
    tools = [vhole(48.0, yc + dy, z0, TOP_Z0 - 0.6) for dy in (-4.0, 4.0)]
    return body - bd.Compound(tools)


# --- rear axle line ---

def rear_wheel(s: int) -> bd.Part:
    """Dish wheel dia. 80 x 24: web on the inner face (prints flat on it), two O-ring grooves, a raised radial
    marker inside the dish so the two sides can be compared turning."""
    rim = cyl(WHEEL_R, WHEEL_W) - cyl(WHEEL_R - 4.0, WHEEL_W + 0.2, -0.1)
    for zc in (6.0, WHEEL_W - 6.0):
        rim -= cyl(WHEEL_R + 0.1, 2.5, zc - 1.25) - cyl(WHEEL_R - 1.2, 2.7, zc - 1.35)
    web = cyl(WHEEL_R - 3.5, 4.0)
    for k in range(5):
        a = 36.0 + 72.0 * k
        web -= bd.Rot(0, 0, a) * bd.Pos(21.5, 0, 0) * cyl(6.5, 4.4, -0.2)
    hub = cyl(7.5, WHEEL_W)
    marker = bd.Pos(0, 0, 4.0) * bd.extrude(
        bd.Polygon((8.5, -1.5), (27.0, -1.5), (27.0, -3.5), (33.0, 0.0), (27.0, 3.5), (27.0, 1.5), (8.5, 1.5),
                   align=None), 1.2)
    w = rim + web + hub + marker - cross_bore(0, WHEEL_W)
    return place(w, (s * WHEEL_X0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def rear_axle(s: int) -> bd.Part:
    """LEGO 10L cross axle (or printed): from 2.5 off the cross pin to the wheel's outer face."""
    return place(d.lego_axle(d.AXLE_IN, AXLE_10L), (0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def _tube(s: int, x0: float, x1: float, bore: bd.Part) -> bd.Part:
    return place(cyl(TUBE_OD / 2, x1 - x0) - bore, (s * x0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


def spacer(s: int) -> bd.Part:
    x0, x1 = SPACER_X
    return _tube(s, x0, x1, round_bore(d.BEARING_D, 0, x1 - x0))


def collar(s: int) -> bd.Part:
    x0, x1 = COLLAR_X
    return _tube(s, x0, x1, cross_bore(0, x1 - x0))


# --- reused parts, placed ---

def battery_location() -> bd.Location:
    """Box coordinates -> vehicle: box length (x) along -y, box width (y) up, lid side (+z) facing -x."""
    return bd.Location(bd.Plane(origin=BATT_C, x_dir=(0, -1, 0), z_dir=(-1, 0, 0)))


def battery_box() -> bd.Part:
    return battery_location() * e._read("aaa3_battery_box_technic")


def battery_lid() -> bd.Part:
    """Same place on the box as in the bench model: over the cells, the last 3 pitches (switch, leads) open."""
    return battery_location() * bd.Pos(-12.0, 0, 11.9 + 3.9) * bd.Rot(0, 0, -90) * e._read("aaa3_battery_lid_technic")


def breadboard_frame() -> bd.Part:
    return bd.Pos(*BREADBOARD_C) * e._read("breadboard_technic")
