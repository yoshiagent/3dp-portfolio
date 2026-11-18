"""Car model: the motor-driven differential at the rear, the MG90S steering at the front, 3xAAA battery box and
breadboard. This file holds the new printed parts and the placements of the reused ones.

Vehicle frame (same as ../../mg90s_steering): x right, y forward, z up from the ground, steering servo shaft on
the z axis. Front axle (kingpins) at y = 40; wheelbase 160 as the steering's Ackermann layout assumes, so the
rear axle is at y = -120.

Why the rear is different from the bench model: the 44-tooth ring reaches 33.3 mm below the axle, so the rear
wheels grow to dia. 80 (axle 40 above the ground) and the plate gets a slot the ring turns in (ring 6.7 mm
above the ground). The front keeps the LEGO 56 x 26 wheels and the steering module unchanged.

New printed parts
* chassis: one Technic plate (8 mm grid, x = 0, y = 0 mod 8, 7.8 thick, bottom 8 above the ground, as the servo
  holder), with the differential's towers, outer axle towers, motor piers, side arms pinned to the servo holder
  and risers pinned under the steering frame.
* deck (upper level) for the breadboard, carried by the battery box (stood on edge, lid outward) and two posts.
* rear wheels dia. 80 x 24, 10L axles, spacers and collars.

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

# --- chassis plate ---
Z0, Z1 = 8.0, 15.8
ROW_Z = Z0 + 3.9  # 11.9: the servo holder's lowest hole row
HALF_W = 51.7
Y_REAR, Y_FRONT = -151.7, -12.3
HOLDER_HALF, HOLDER_Y0 = 20.3, -28.3  # holder +0.6 all round (holder |x| <= 19.7, y >= -27.7)
ARM_X1, ARM_Y1 = 35.7, 35.7
RISER = (20.3, 27.7, 20.3, 35.7)  # |x|, y under the steering frame plate (bottom z 24.2)
RISER_Z1 = 23.6
HOLDER_SIDE_ROWS = (-24.0, -16.0, -8.0, 0.0, 8.0, 16.0)  # holder side-face holes (y) at z = 11.9
HOLDER_END_COLS = (-16.0, -8.0, 0.0, 8.0, 16.0)  # holder back-end holes (x) at z = 11.9
SIDE_HOLE_DEPTH = 9.0
SLOT = (-17.0, -4.0, REAR_Y - 24.5, REAR_Y + 24.5)  # the ring (x -15.4 .. -5.6) turns through it
TOWER_TOP = AXLE_Z + d.TOWER_TOP
TIE_RIB_H = 8.0  # low rib joining each inner axle tower to the outer one
T = d.TOWER_X1 - d.TOWER_X0  # tower thickness 4
PT_Y0, PT_Y1 = REAR_Y - e.PINION_TOWER_Y1, REAR_Y - e.PINION_TOWER_Y0  # pinion tower -85 .. -81
MOTOR_Y = REAR_Y - e.MOTOR_C_Y  # -48
PIER_TOP = AXLE_Z + e.MOTOR_FRAME_Z  # 20.3: motor frame's lower side
PIER_COLS = tuple(sorted((-c1, -c0) for c0, c1 in e.PIER_COLS))  # (-24,-16), (0,8), (40,48)
MOTOR_X = (-27.7, 51.7)  # motor frame extent

# --- electronics ---
BATT_C = (-40.0, -52.0, Z1 + 27.7)  # box stood on its long edge, lid facing -x
DECK_Z0 = Z1 + 55.4 + 0.6  # 71.8: one pitch above the box top
DECK_Z1 = DECK_Z0 + 7.8
DECK_Y = (-95.7, -16.3)
POSTS_Y = (-20.0, -84.0)  # deck posts at x = 48, two holes each (y +/- 4)
POST_X = (44.3, 51.7)
BREADBOARD_C = (0.0, -56.0, DECK_Z1 + 7.9)


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


def plate_grid(foot: list[tuple]) -> list[tuple[float, float]]:
    """Vertical holes of the chassis plate on the 8 mm grid, away from towers, piers, risers and crossing holes."""
    m = CB_D / 2 + 0.6
    pts = []
    for i in range(-6, 7):
        x = i * PITCH
        for j in range(-19, 5):
            y = j * PITCH
            in_main = abs(x) <= HALF_W - 3.7 and Y_REAR + 3.7 <= y <= Y_FRONT - 3.7 and not (
                abs(x) < HOLDER_HALF + 3.7 and y > HOLDER_Y0 - 3.7)
            in_arm = HOLDER_HALF + 3.7 <= abs(x) <= ARM_X1 - 3.7 and HOLDER_Y0 <= y <= ARM_Y1 - 3.7
            if not (in_main or in_arm):
                continue
            if any(x0 - m < x < x1 + m and y0 - m < y < y1 + m for x0, x1, y0, y1 in foot):
                continue
            if y in HOLDER_SIDE_ROWS and abs(x) < HOLDER_HALF + SIDE_HOLE_DEPTH + 0.3:
                continue  # crosses a side hole into the holder
            if abs(x) <= 16 and y == -32:
                continue  # crosses a back-end hole into the holder
            pts.append((x, y))
    return pts


def chassis() -> bd.Part:
    body = box(-HALF_W, HALF_W, Y_REAR, Y_FRONT, Z0, Z1)
    body -= box(-HOLDER_HALF, HOLDER_HALF, HOLDER_Y0, Y_FRONT + 1, Z0 - 1, Z1 + 1)
    foot = []
    for s in (1, -1):
        body += box(*sorted((s * HOLDER_HALF, s * ARM_X1)), HOLDER_Y0, ARM_Y1, Z0, Z1)
        rx = sorted((s * RISER[0], s * RISER[1]))
        body += box(*rx, RISER[2], RISER[3], Z1 - EMBED, RISER_Z1)
        foot.append((*rx, RISER[2], RISER[3]))
    tw, tfoot = towers()
    pr, pfoot = piers()
    body = body + tw + pr
    foot += tfoot + pfoot + [SLOT]
    # bearings: rear axle line (all four axle towers), pinion line
    body -= bd.Pos(0, REAR_Y, AXLE_Z) * bd.Rot(0, 90, 0) * bd.Cylinder(d.BEARING_D / 2, 2 * HALF_W + 2)
    body -= bd.Pos(0, (PT_Y0 + PT_Y1) / 2, AXLE_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(d.BEARING_D / 2, 20)
    body -= box(SLOT[0], SLOT[1], SLOT[2], SLOT[3], Z0 - 1, Z1 + 1)
    tools = [vhole(x, y, Z0, Z1) for x, y in plate_grid(foot)]
    for s in (1, -1):
        for y in (24.0, 32.0):  # through the risers, into the steering frame plate
            tools.append(vhole(s * 24.0, y, Z0, RISER_Z1))
        for y in HOLDER_SIDE_ROWS:  # side arms -> servo holder side faces
            tools.append(hole((s * HOLDER_HALF, y, ROW_Z), (s, 0, 0), SIDE_HOLE_DEPTH))
    for x in HOLDER_END_COLS:  # plate front edge -> servo holder back end
        tools.append(hole((x, HOLDER_Y0, ROW_Z), (0, -1, 0), 8.5))
    for c0, c1 in PIER_COLS:  # through pier and plate, under the motor frame's side holes
        for x in (c0, c1):
            for y in (MOTOR_Y - PITCH, MOTOR_Y, MOTOR_Y + PITCH):
                tools.append(vhole(x, y, Z0, PIER_TOP))
    return body - bd.Compound(tools)


# --- upper level ---

def deck() -> bd.Part:
    body = box(-HALF_W, HALF_W, DECK_Y[0], DECK_Y[1], DECK_Z0, DECK_Z1)
    tools = [vhole(i * PITCH, j * PITCH, DECK_Z0, DECK_Z1) for i in range(-6, 7) for j in range(-11, -2)]
    return body - bd.Compound(tools)


def deck_post(yc: float) -> bd.Part:
    body = box(*POST_X, yc - 7.7, yc + 7.7, Z1, DECK_Z0 - 0.6)
    tools = [vhole(48.0, yc + dy, Z1, DECK_Z0 - 0.6) for dy in (-4.0, 4.0)]
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
