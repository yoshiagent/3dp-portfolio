"""Differential + Oldham coupling + TT motor, the drive module of the car model on its own.

FROZEN COPY (2026-11-25) of ../../car_model/src/car_parts.py, taken when the car's TT motor lay centred across the
car and drove the input axle through the Oldham coupling. The car is still being changed and may drop the coupling;
this project keeps that design as it was. Only the drive-module functions are used here (drive_base, oldham_*,
input_axle, spin_y, motor_place, to_vehicle, rear axle line); the rest is left as copied.

The original module docstring follows.
Car model: the motor-driven differential at the rear, the MG90S steering at the front, and on top the breadboard
tray pinned back to back to the two-cell 18650 tray. This file holds the new printed parts and the placements of the
reused ones.

Vehicle frame (same as ../../mg90s_steering): x right, y forward, z up from the ground, steering servo shaft on
the z axis. Front axle (kingpins) at y = 40; wheelbase 160 as the steering's Ackermann layout assumes, so the
rear axle is at y = -120.

Why the rear is different from the bench model: the 44-tooth ring reaches 33.3 mm below the axle, so the rear
wheels are the large LEGO motorcycle wheels, tyre 2902 81.6 x 15 on rim 2903 (axle 40.8 above the ground), and the
plate gets a slot the ring turns in (ring 7.5 mm above the ground). The front keeps the LEGO 56 x 26 wheels (tyre
55976 on rim 56145) and the steering module unchanged.

Modules (each is assembled on its own, then joined with LEGO Technic 2L pins, all vertical, from above)
* drive module: drive base (Technic plate z 8 - 15.8, 8 mm grid, x = 0, y = 0 mod 8, the lowest level at the
  rear) with the differential's towers, outer axle towers and motor piers; carries the TT motor lying in its frame,
  centred across the car, the Oldham coupling from its shaft (12 mm left of the centre line) to the input axle,
  the differential, rear axles, collars, spacers and rear wheels.
* steering module: the MG90S steering exactly as in ../../mg90s_steering (servo holder hanging from the steering
  frame's bulkhead, frame plate z 24.2 - 31.6 out to the kingpins).
* chassis (middle level, plate z 16.0 - 23.8): stacked on the drive base's front edge and left side, and under
  the steering frame plate. Pins go down through the chassis into the drive base, and down through the steering
  frame plate into the chassis. Three of the four posts stand on it.
* upper level: the breadboard tray (13 x 9 x 2) on top, pocket up, and the two-cell 18650 tray (13 x 9 x 3, the
  same footprint) turned over under it, the two bottoms pinned together; the pair sits on four posts.
* nothing on the rear axle line is printed: LEGO motorcycle tyres on their rims, 10L axles, half bushes (the
  collars, 4 mm, inside the outer towers and outside the hubs) and bushes (the spacers, 8 mm). Their shapes here are stand-ins for the assembly and checks.

Why three levels: the TT motor frame's lower side is 20.3 above the ground, so the drive base must stay at the
bottom (z 8 - 15.8) and the chassis that joins it to the steering goes one level up, clear of the motor.

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
AXLE_Z = 40.8  # rear wheel radius: LEGO tyre 81.6 x 38 R
PITCH = 8.0
PIN_D, CB_D, CB_DEPTH = e.PIN_D, e.CB_D, e.CB_DEPTH

# --- rear axle line (|x|) ---
WHEEL_R, WHEEL_W = 40.8, 13.8  # LEGO motorcycle wheel: rim 2903 (61.6 x 13.8) in tyre 2902 (81.6 x 15)
RIM_R, TYRE_W = 30.8, 15.0
AXLE_10L = 10 * PITCH - 0.2
AXLE_END = d.AXLE_IN + AXLE_10L  # 82.3: through the 13.8 hub, 10.2 out past its outer face
BUSH_L = 8.0  # LEGO bush between the outer tower and the wheel
OUTER_X0, OUTER_X1 = 46.0, 50.0  # outer axle towers
SPACER_X = (OUTER_X1 + 0.15, OUTER_X1 + 0.15 + BUSH_L)  # LEGO bush, tower -> wheel: no inward slide
WHEEL_X0 = SPACER_X[1] + 0.15  # 58.3: wheel inner face
WHEEL_X1 = WHEEL_X0 + WHEEL_W
COLLAR_L = 4.0
COLLAR_X = (OUTER_X0 - 0.15 - COLLAR_L, OUTER_X0 - 0.15)  # on the axle inside the outer tower: no outward slide
OUT_COLLAR_X = (WHEEL_X1 + 0.15, WHEEL_X1 + 0.15 + COLLAR_L)  # half bush outside the hub: the wheel stays on
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
MOTOR_Y0 = MOTOR_Y - 11.9  # -59.9 .. -36.1: motor frame along y
MOTOR_Y1 = MOTOR_Y + 11.9
# TT motor lying across the car as in the electric differential, but moved 12 to the left so its frame is centred
# (x -39.7 .. 39.7); its shaft is then at x -12, 12 left of the input axle, and an Oldham coupling joins the two.
MOTOR_SHIFT = -12.0
MOTOR_X = (-27.7 + MOTOR_SHIFT, 51.7 + MOTOR_SHIFT)  # motor frame extent
TT_SHAFT_X = MOTOR_SHIFT  # -12
PIER_TOP = AXLE_Z + e.MOTOR_FRAME_Z  # 20.3: motor frame's lower side
PIER_COLS = tuple(sorted((-c1 + MOTOR_SHIFT, -c0 + MOTOR_SHIFT) for c0, c1 in e.PIER_COLS))  # (-36,-28),(-12,-4),(28,36)
# Oldham coupling, along -y from the TT gearbox: hub on the TT shaft (a D bore, small boss through the frame's lip,
# then a disc), the slider (two tongues at right angles), hub on the input axle (cross bore) against the pinion tower.
# Hub slots: the motor hub's vertical, the axle hub's horizontal; straight ahead the slider's centre is on the TT axis.
OD_Y0 = -58.9  # front of the motor hub (as the old TT coupler), 1.0 off the gearbox face
OD_R = 20.0  # hub radius: the slider's 15 mm tongue stays fully in the slot over the whole 12 mm offset
OD_BOSS_R, OD_BOSS_L = 4.8, 1.4  # boss inside the frame's lip, to y -60.3 (frame face -59.9)
OD_BORE = 7.5  # TT shaft depth (shaft 8.5 out of the gearbox face, which is 1.0 behind the hub)
OD_FLOOR = 0.4  # between the shaft's end and the slot
OD_SLOT_D, OD_SLOT_W = 2.8, 5.3  # slot depth and width (tongue 5.0)
OD_GAP = 0.3  # hub face -> slider body
OD_DISC_R, OD_DISC_T = 8.0, 3.0  # slider body: 16 x 16 square, 3 thick
OD_TONGUE_H, OD_TONGUE_W = 2.5, 5.0  # tongue height (into a slot, 0.3 short of its bottom) and width
OD_TONGUE_L = 15.0  # tongue length: at 12 mm off centre its corners stay 0.3 inside the hub's 20 mm radius
OD_A1 = OD_BORE + OD_FLOOR + OD_SLOT_D  # 10.7: motor hub length (local, from OD_Y0 rearward)
OD_S0 = OD_A1 + OD_GAP  # 11.0: slider body front
OD_B0 = OD_S0 + OD_DISC_T + OD_GAP  # 14.3: axle hub front
OD_B1 = PT_Y1 + 0.3  # axle hub back face (y), 0.3 in front of the pinion tower (front face y -81)
INPUT_AXLE = (-92.0, 2 * PITCH - 0.2)  # LEGO 2L from the pinion's blind hole to the axle hub's slot bottom
# --- chassis (middle level): one Technic plate stacked on the drive base, under the steering frame plate ---
MID_Z0, MID_Z1 = Z1 + 0.2, Z1 + 8.0  # 16.0 .. 23.8; steering frame plate bottom 24.2 (0.4 above)
MID_FRONT = (-35.7, -12.3)  # y: full-width part over the drive base's front edge, 0.4 ahead of the motor frame;
                            # at full lock the front wheels reach y -4.7 (|x| >= 36.9) at this height
RAIL = (-51.7, MOTOR_X[0] - 0.6, -99.7, -35.7)  # x0, x1, y0, y1: left rail over the drive base, beside the
                                     # centred motor frame (0.6 off x -39.7: one column of holes, x -48), ends clear
                                     # of the left outer tower's ribs (y -105)
ARM = (20.3, 27.7)  # |x| of the two arms beside the servo holder and bulkhead (inner edge 0.6 off them)
NOSE = (28.3, 43.7)  # y: block under the steering frame plate, ahead of the bulkhead (y <= 27.7) up to the
                     # plate's front edge; |x| <= ARM[1], well inside the knuckles' lower ears (|x| >= 36.3)
STEER_PINS = [(x, y) for x in (-24.0, -16.0, -8.0, 0.0, 8.0, 16.0, 24.0) for y in (32.0, 40.0)] + [
    (-24.0, 24.0), (24.0, 24.0)]  # steering frame plate holes over the chassis (plate holes in build_steering.py)

# --- electronics ---
# chassis -> drive base pins on the left rail (x -48)
RAIL_PIN_ROWS = (-96.0, -80.0, -64.0, -48.0, -32.0, -16.0)

# --- upper level: breadboard tray (13 x 9 x 2, 15.8) on top, pocket up; the two-cell 18650 tray of
# ../tt_motor_adapter (battery18650x2_technic, 13 x 9 x 3, 23.6, pocket 76 x 42.6 on a 7.8 floor) turned over under
# it, bottoms 0.2 apart and pinned together. One pitch further forward than the old breadboard (centre y -48), so
# the pair clears the ring gear (top 74.1 at y <= -86.7). The holder sticks 2.2 out of the turned-over tray, down
# over the motor (frame top 60.5) and the coupling hubs (top 60.8), and the cells 2.5 more: tray face at z 66.4.
STACK_YC = -48.0
TRAY_FACE = 66.4  # underside of the pair (the battery tray's open face)
TRAY_H, TRAY_FLOOR = 23.6, 7.8
TRAY_C = (0.0, STACK_YC, TRAY_FACE + TRAY_H / 2)  # 75.8
TOP_Z0 = TRAY_FACE + TRAY_H + 0.2  # 87.8: breadboard frame bottom
BREADBOARD_C = (0.0, STACK_YC, TOP_Z0 + 7.9)  # frame 15.8 high, origin at its centre
HD2_L, HD2_W, HD_H = 76.0, 42.6, 18.0  # two-cell holder and its height
# pins between the two bottoms (both full 13 x 9 grids where the battery tray has holes)
STACK_PINS = [(x, STACK_YC + y) for x in (-40.0, 40.0) for y in (-32.0, 32.0)] + [
    (-48.0, STACK_YC), (48.0, STACK_YC), (0.0, STACK_YC - 32.0), (0.0, STACK_YC + 32.0)]
# four posts at x = +-48 under the battery tray's ring holes (two each, y +-4 about the post)
POSTS_Y = (STACK_YC + 28.0, STACK_YC - 28.0)  # -20 (front), -76 (rear)
POSTS_Z0 = {(1, 0): MID_Z1, (1, 1): Z1, (-1, 0): MID_Z1, (-1, 1): MID_Z1}  # (side, rear): right rear post on the
                              # drive base (the motor leaves no chassis there), the other three on the chassis
POST_X = (44.3, 51.7)  # |x|
POST_TOP = TRAY_FACE - 0.6  # 63.4
# post feet: the left rail's x = -48 holes at RAIL_PIN_ROWS carry chassis -> drive base pins, so the left posts
# pin down through their other hole only
POST_PINS_DOWN = {(1, 0): [(48.0, -24.0), (48.0, -16.0)], (1, 1): [(48.0, -80.0), (48.0, -72.0)],
                  (-1, 0): [(-48.0, -24.0)], (-1, 1): [(-48.0, -72.0)]}


def to_vehicle(shape: bd.Part) -> bd.Part:
    """Differential coordinates (electric_parts) -> vehicle: turned 180 deg (motor ahead of the axle)."""
    return bd.Pos(0, REAR_Y, AXLE_Z) * bd.Rot(0, 0, 180) * shape


def motor_place(shape: bd.Part) -> bd.Part:
    """Vehicle-placed motor frame / motor (as in the electric differential) -> centred across the car."""
    return bd.Pos(MOTOR_SHIFT, 0, 0) * shape


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
    parts.append(gusset((-T / 2, PT_Y1, z), (0, 1, 0), 8.0, 4.5, (1, 0, 0)))  # motor side: low, under the axle hub
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
    """Middle level: joins the drive module (below) to the steering module (above), carries three of the posts."""
    body = reduce(lambda a, b: a + b, [box(x0, x1, y0, y1, MID_Z0, MID_Z1) for x0, x1, y0, y1 in chassis_outline()])
    body = body.clean()
    return body - bd.Compound([vhole(x, y, MID_Z0, MID_Z1) for x, y in chassis_grid()])


def deck_post(yc: float, z0: float, s: int = 1) -> bd.Part:
    """Post under the battery tray's edge (s = 1 right, -1 left), from z0 up to 0.6 under the tray."""
    x0, x1 = sorted((s * POST_X[0], s * POST_X[1]))
    body = box(x0, x1, yc - 7.7, yc + 7.7, z0, POST_TOP)
    tools = [vhole(s * 48.0, yc + dy, z0, POST_TOP) for dy in (-4.0, 4.0)]
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


# --- Oldham coupling: TT shaft (x -12) -> input axle (x 0) ---

def _along_y(shape: bd.Part, x: float) -> bd.Part:
    """Coupling-local (axis +z, rearward from OD_Y0, local y up) -> vehicle at shaft position x."""
    return place(shape, (x, OD_Y0, AXLE_Z), (1, 0, 0), (0, -1, 0))


def oldham_hub_motor() -> bd.Part:
    """On the TT shaft: boss through the frame's lip, disc with a vertical slot in its back face, D bore."""
    hub = cyl(OD_BOSS_R, OD_BOSS_L) + cyl(OD_R, OD_A1 - OD_BOSS_L, OD_BOSS_L)
    hub -= bd.Pos(0, 0, OD_A1 - OD_SLOT_D) * bd.Box(OD_SLOT_W, 2 * OD_R + 2, OD_SLOT_D + 1,
                                                   align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    shaft = bd.Face(bd.Wire.make_circle(2.7 + 0.1)) & bd.Rectangle(3.7 + 0.15, 7)  # TT double-D, +0.1 / +0.075
    hub -= bd.Pos(0, 0, -0.1) * bd.extrude(shaft, OD_BORE + 0.1)
    return _along_y(hub, TT_SHAFT_X)


def oldham_slider_local() -> bd.Part:
    """Slider in coupling-local coordinates, centred on z = 0's axis: a square body (it prints standing on an edge,
    the body's edge flat on the bed, no supports), vertical tongue forward (into the
    motor hub's slot), horizontal tongue rearward (into the axle hub's slot)."""
    body = bd.Pos(0, 0, OD_S0) * bd.Box(2 * OD_DISC_R, 2 * OD_DISC_R, OD_DISC_T,
                                        align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    front = bd.Pos(0, 0, OD_S0 - OD_TONGUE_H) * bd.Box(OD_TONGUE_W, OD_TONGUE_L, OD_TONGUE_H + 0.01,
                                                      align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    back = bd.Pos(0, 0, OD_S0 + OD_DISC_T - 0.01) * bd.Box(OD_TONGUE_L, OD_TONGUE_W, OD_TONGUE_H + 0.01,
                                                          align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return body.fuse(front, back).clean()


def oldham_slider(phi: float = 0.0) -> bd.Part:
    """Slider with the coupling turned phi degrees (same sense for both shafts, about +y). Its centre lies on both
    slots: straight ahead on the TT axis; it orbits on a 12 mm circle through both axes."""
    a = math.radians(phi)
    e_off = -TT_SHAFT_X  # 12: the axle hub is 12 to the right of the motor hub
    # vehicle x / z of the slider centre (see the derivation in check_car.py): x = -12 + 12 sin^2, z = 12 sin cos
    dx, dz = e_off * math.sin(a) ** 2, e_off * math.sin(a) * math.cos(a)
    s = _along_y(oldham_slider_local(), TT_SHAFT_X)
    return bd.Pos(dx, 0, dz) * spin_y(s, TT_SHAFT_X, phi)


def oldham_hub_axle() -> bd.Part:
    """On the input axle: horizontal slot in its front face, cross bore, back 0.3 off the pinion tower."""
    length = (OD_Y0 - OD_B1) - OD_B0
    hub = cyl(OD_R, length, OD_B0)
    hub -= bd.Pos(0, 0, OD_B0 - 1) * bd.Box(2 * OD_R + 2, OD_SLOT_W, OD_SLOT_D + 1,
                                           align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    hub -= cross_bore(OD_B0, OD_B0 + length)
    return _along_y(hub, 0.0)


def spin_y(shape: bd.Part, x: float, phi: float) -> bd.Part:
    """Turn a part phi degrees about the line along y through (x, AXLE_Z), the sense of three.js makeRotationY."""
    return bd.Pos(x, 0, AXLE_Z) * bd.Rot(0, phi, 0) * bd.Pos(-x, 0, -AXLE_Z) * shape


def input_axle() -> bd.Part:
    """LEGO 2L cross axle from the pinion's blind hole to the axle hub's slot bottom."""
    y0, length = INPUT_AXLE
    return place(d.lego_axle(0, length), (0, y0, AXLE_Z), (1, 0, 0), (0, 1, 0))


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
    """LEGO 10L cross axle (or printed): from 2.5 off the cross pin to the wheel's outer face."""
    return place(d.lego_axle(d.AXLE_IN, AXLE_10L), (0, REAR_Y, AXLE_Z), (0, 0, 1), (s, 0, 0))


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
