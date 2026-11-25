"""TT motor (centred) + double universal joint + bevel differential: a teaching module about universal joints.

The differential is ../../differential_model unchanged (33:11 = 3:1 ring and pinion, carrier, spiders, side gears,
sleeves, 4L axles, wheels with arrows). Its pinion axis is not on the centre line: it lies 23.5 mm to the right
(x = RING_APEX_X), parallel to the axles' normal. The TT motor's shaft is on the centre line (x = 0 at the default
position), so the two shafts are parallel and offset, and a double universal joint in Z layout joins them:

  TT motor -> coupler (TT shaft to LEGO cross) -> axle A1 in the bearing tower -> joint J1 -> intermediate LEGO
  axle (slanted) -> joint J2 -> input axle through the pinion tower -> pinion

Both joint angles are equal (Z layout), so with the intermediate axle's two yokes in one plane the pinion turns
exactly with the motor; turn one of those yokes 90 deg on its cross axle and the speed ripple doubles instead.

Teaching aids: a pointer dial on A1, on the intermediate axle and on the input axle (fixed index fins on the
bearing tower and the pinion tower), the motor at four sideways positions on the base (x = 0, -8, -16, -24: joint
angle 22.8 .. 40.3 deg, a matching LEGO axle length for each), and the differential model's hand crank, which goes
on A1's free end when the motor and coupler are lifted off.

Frame (as ../../differential_model): x along the axles (right +), y forward (the motor is at -y), z up; axle
centre line z = 0, differential at the origin. Units mm.
"""
from __future__ import annotations

import math
import sys
from functools import lru_cache, reduce
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "electric_differential" / "src"))

from cadgen import build123d as bd  # noqa: E402

import electric_parts as e  # noqa: E402
from electric_parts import d  # noqa: E402
from differential_parts import cross_bore, cyl, place, round_bore  # noqa: E402

PITCH = 8.0
PIN_D, CB_D, CB_DEPTH = e.PIN_D, e.CB_D, e.CB_DEPTH
LEGO_CLEAR = d.LEGO_CLEAR
BEARING_D = d.BEARING_D

# --- universal joint (all four yokes the same part, two spiders) ---
# Yoke local frame: axis +z from the tail face (z = 0) toward the joint centre (z = Z_C); pin axis along local x.
HUB_R, HUB_L = 8.0, 12.0  # hub with a blind LEGO cross bore, BORE deep from the tail
BORE = 13.0  # into the plate: A1 stops 0.5 short of the bottom
PLATE_T = 3.0  # plate across the fork, between the hub and the ears
EAR_IN, EAR_T, EAR_W = 11.4, 7.8, 11.0  # ears at |x| in [EAR_IN, EAR_IN + EAR_T], EAR_W wide (local y)
EAR_OUT = EAR_IN + EAR_T  # 19.2
SP_ARM, SP_W = 11.0, 10.0  # spider arms reach +-SP_ARM, square SP_W section
SP_HOLE_D, SP_HOLE_DEPTH = BEARING_D, 8.0  # journal holes: a LEGO 2L axle turns in them
SP_SWEEP = math.hypot(SP_ARM, SP_W / 2)  # 12.08: spider arm corner about the other pin axis
Z_C = HUB_L + PLATE_T + SP_SWEEP + 0.4  # 27.5: joint centre from the tail
# journal: LEGO 2L axle (15.8) from the ear's outer face through the ear (cross hole), 0.4 gap, 7.6 into the spider
JOURNAL_L = 2 * PITCH - 0.2
SP_GAP = EAR_IN - SP_ARM  # 0.4

# --- dials: pointer discs on the shafts ---
DIAL_R, DIAL_T = 17.0, 2.4
DIAL_GAP = 0.3

# --- differential (../../differential_model) ---
APEX_X = d.RING_APEX_X  # 23.5: pinion axis x
PT_Y0 = d.PINION_TOWER_Y0  # -33: pinion tower back face
BASE_TOP = d.BASE_Z1  # -30
BASE_T = 7.8
BASE_BOT = BASE_TOP - BASE_T

# --- layout along y (motor side -y) ---
DIAL_IN_Y = (PT_Y0 - DIAL_GAP - DIAL_T, PT_Y0 - DIAL_GAP)  # input dial between the pinion tower and J2's yoke
YOKE_IN_TAIL = DIAL_IN_Y[0] - DIAL_GAP  # input yoke's tail
J2 = (APEX_X, YOKE_IN_TAIL - Z_C, 0.0)
D_Y = 56.0  # J1 is this far behind J2 (along -y)
POSITIONS = (0.0, -8.0, -16.0, -24.0)  # motor shaft x; the default (centred) first
J1_Y = J2[1] - D_Y
YOKE_M_TAIL = J1_Y - Z_C  # motor-side yoke's tail
DIAL_M_Y = (YOKE_M_TAIL - DIAL_GAP - DIAL_T, YOKE_M_TAIL - DIAL_GAP)
B_T = 7.8  # bearing tower thickness (y)
B_Y1 = DIAL_M_Y[0] - DIAL_GAP  # bearing tower front face
B_Y0 = B_Y1 - B_T
B_HALF_W, B_TOP = 12.0, 8.0
GRID_Y0 = (B_Y0 + B_Y1) / 2  # base hole rows: y = GRID_Y0 + 8k (the bearing tower's pins on one of them)
COUPLER_GAP = 0.7
COUPLER_L, COUPLER_TT_DEPTH, COUPLER_X_DEPTH = 16.5, 7.5, 8.0
COUPLER_Y1 = B_Y0 - COUPLER_GAP  # coupler's LEGO end
COUPLER_Y0 = COUPLER_Y1 - COUPLER_L  # coupler's TT end
GEARBOX_Y = COUPLER_Y0 - 1.0  # TT gearbox face: the shaft (8.5 out) runs 7.5 into the coupler
MOTOR_TOP_LOCAL = e.MOTOR_TOP_LOCAL  # 9.9: gearbox face in frame coordinates
FRAME_Y = GEARBOX_Y - MOTOR_TOP_LOCAL  # motor frame origin (its rows at FRAME_Y + 0, +-8)
FRAME_SHAFT_X = e.MOTOR_FRAME_SHAFT_X  # 12
FRAME_HALF_H = 11.9
FRAME_LOW_Z = -19.7  # frame's lower long side (frame y = +19.7)
BED = (-80.0 + 4.3, 27.7, FRAME_Y - FRAME_HALF_H, FRAME_Y + FRAME_HALF_H)  # raised bed under every motor position
BASE_X = (-79.7, 37.9)
BASE_Y = (FRAME_Y - FRAME_HALF_H - 3.9, 28.0)  # front edge = the differential frame's


def lego_len(n: int) -> float:
    return n * PITCH - 0.2


def joint_angle(x: float) -> float:
    """Joint angle (deg) for the motor shaft at x: both joints bend by the same angle (Z layout)."""
    return math.degrees(math.atan2(APEX_X - x, D_Y))


def j1(x: float):
    return (x, J1_Y, 0.0)


def mid_length(x: float) -> float:
    """J1 -> J2 distance."""
    return math.hypot(APEX_X - x, D_Y)


def mid_axle(x: float) -> tuple[int, float]:
    """LEGO length (studs) for the intermediate axle and how deep it sits in each yoke's bore (equal both ends)."""
    span = mid_length(x) - 2 * Z_C  # between the two tails
    best = None
    for n in range(2, 13):
        e_each = (lego_len(n) - span) / 2
        if 6.0 <= e_each <= BORE and (best is None or abs(e_each - 9.0) < abs(best[1] - 9.0)):
            best = (n, e_each)
    if best is None:
        raise ValueError(f"no LEGO axle fits x = {x}: span {span:.1f}")
    return best


A1_N = 4  # A1: LEGO 4L from 0.2 short of the coupler's cross bore bottom through the tower and dial into the
A1_Y0 = COUPLER_Y1 - COUPLER_X_DEPTH + 0.2  # motor-side yoke (0.5 short of its bore bottom)
A1_Y1 = A1_Y0 + lego_len(A1_N)
INPUT_N = 4  # input axle: LEGO 4L from inside the pinion through the tower and dial to the input yoke's bore bottom
INPUT_Y0 = YOKE_IN_TAIL - BORE
INPUT_Y1 = INPUT_Y0 + lego_len(INPUT_N)


# --- helpers ---

def box(x0, x1, y0, y1, z0, z1) -> bd.Part:
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def frame_at(origin, x_dir, z_dir) -> bd.Location:
    return bd.Location(bd.Plane(origin=origin, x_dir=x_dir, z_dir=z_dir))


def lego_axle(length: float) -> bd.Part:
    return d.lego_axle(0.0, length)


def pin_hole(x, y, z_top, depth, cb_top=True, cb_bottom=False) -> bd.Part:
    """Vertical Technic pin hole from z_top down `depth`, counterbored at the top (and bottom)."""
    tool = bd.Pos(x, y, z_top - depth - 0.1) * cyl(PIN_D / 2, depth + 0.2)
    if cb_top:
        tool += bd.Pos(x, y, z_top - CB_DEPTH) * cyl(CB_D / 2, CB_DEPTH + 0.1)
    if cb_bottom:
        tool += bd.Pos(x, y, z_top - depth - 0.1) * cyl(CB_D / 2, CB_DEPTH + 0.1)
    return tool


# --- printed parts, local frames ---

@lru_cache(maxsize=None)
def yoke() -> bd.Part:
    """Fork: hub with a blind LEGO cross bore at the tail, a plate, two ears with LEGO cross holes for the journals
    (2L axles). Local: tail at z = 0, joint centre at z = Z_C, journals along x."""
    body = cyl(HUB_R, HUB_L + 0.01)
    body += box(-EAR_OUT, EAR_OUT, -EAR_W / 2, EAR_W / 2, HUB_L, HUB_L + PLATE_T)
    for s in (1, -1):
        x0, x1 = sorted((s * EAR_IN, s * EAR_OUT))
        ear = box(x0, x1, -EAR_W / 2, EAR_W / 2, HUB_L, Z_C)
        ear += place(cyl(EAR_W / 2, EAR_T), (x0, 0, Z_C), (0, 0, 1), (1, 0, 0))  # rounded end about the journal
        body += ear
        body -= place(cross_bore(-0.1, EAR_T + 0.2), (x0, 0, Z_C), (0, 0, 1), (1, 0, 0))
    body -= cross_bore(-0.1, BORE)
    return body


@lru_cache(maxsize=None)
def spider() -> bd.Part:
    """Cross: two square arms, a blind round hole in each arm end for a journal. Local: arms along x and y."""
    body = box(-SP_ARM, SP_ARM, -SP_W / 2, SP_W / 2, -SP_W / 2, SP_W / 2)
    body += box(-SP_W / 2, SP_W / 2, -SP_ARM, SP_ARM, -SP_W / 2, SP_W / 2)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        body -= place(round_bore(SP_HOLE_D, -0.1, SP_HOLE_DEPTH), (dx * SP_ARM, dy * SP_ARM, 0), (0, 0, 1),
                      (-dx, -dy, 0))
    return body


@lru_cache(maxsize=None)
def dial() -> bd.Part:
    """Pointer disc: notches every 15 deg round the rim (deep ones every 90), a round window marks 0 deg (+x).
    Local: axis z, faces z = 0 .. DIAL_T."""
    body = cyl(DIAL_R, DIAL_T)
    for k in range(24):
        a = 15.0 * k
        deep = k % 6 == 0
        notch = bd.Pos(DIAL_R, 0, DIAL_T / 2) * bd.Box(2 * (2.6 if deep else 1.4), 1.0 if deep else 0.8, DIAL_T + 1)
        body -= bd.Rot(0, 0, a) * notch
    body -= bd.Pos(DIAL_R - 5.0, 0, -0.1) * cyl(1.9, DIAL_T + 0.2)  # 0 deg window
    body -= cross_bore(-0.1, DIAL_T + 0.2)
    return body


@lru_cache(maxsize=None)
def bearing_tower() -> bd.Part:
    """Tower for axle A1 at the motor position x = 0 (moves with the motor): bearing hole at z = 0, three pin holes
    from below, an index fin on top over the dial. World placement at x = 0; shift with the motor."""
    body = box(-B_HALF_W, B_HALF_W, B_Y0, B_Y1, BASE_TOP, B_TOP)
    body += box(-1.2, 1.2, B_Y0, B_Y1 + DIAL_GAP + DIAL_T * 0.8, B_TOP, DIAL_R + 4.0)  # index fin
    body -= box(-2, 2, B_Y1, B_Y1 + 5, B_TOP - 0.01, DIAL_R + 1.2)  # under the fin's tip: clear of the dial
    body -= place(round_bore(BEARING_D, -0.1, B_T + 0.2), (0, B_Y0, 0), (1, 0, 0), (0, 1, 0))
    for x in (-PITCH, 0.0, PITCH):
        body -= pin_hole(x, GRID_Y0, BASE_TOP + PITCH, PITCH, cb_top=False, cb_bottom=True)
    return body


def tower_index_fin() -> bd.Part:
    """Index fin on the pinion tower's top, over the input dial (part of the base)."""
    fin = box(APEX_X - 1.2, APEX_X + 1.2, PT_Y0 - DIAL_GAP - DIAL_T * 0.8, d.PINION_TOWER_Y1, d.TOWER_TOP, DIAL_R + 4.0)
    fin -= box(APEX_X - 2, APEX_X + 2, PT_Y0 - 5, PT_Y0, d.TOWER_TOP - 0.01, DIAL_R + 1.2)
    return fin


def grid_xy():
    xs = [PITCH * i for i in range(-12, 6) if BASE_X[0] + 3.9 <= PITCH * i <= BASE_X[1] - 3.9]
    ys = []
    k = 0
    while GRID_Y0 + PITCH * k <= BASE_Y[1] - 3.9:
        if GRID_Y0 + PITCH * k >= BASE_Y[0] + 3.9:
            ys.append(GRID_Y0 + PITCH * k)
        k += 1
    k = -1
    while GRID_Y0 + PITCH * k >= BASE_Y[0] + 3.9:
        ys.append(GRID_Y0 + PITCH * k)
        k -= 1
    return xs, sorted(ys)


# plan footprints (x0, x1, y0, y1) of the differential model's towers and ribs: no base holes under them
_T0, _T1, _TW = d.TOWER_X0, d.TOWER_X1, d.TOWER_HALF_W
UPRIGHTS = [(s0, s1, -_TW - 8.0, _TW + 8.0) for s0, s1 in ((_T0, _T1), (-_T1, -_T0))] + [
    (_T0 - 6.0, _T1 + 8.0, -2.0, 2.0), (-_T1 - 8.0, -_T0 + 6.0, -2.0, 2.0),
    (APEX_X - _TW, APEX_X + _TW, PT_Y0, d.PINION_TOWER_Y1), (APEX_X - 2.0, APEX_X + 2.0, PT_Y0 - 8.0, d.PINION_TOWER_Y1 + 8.0)]


@lru_cache(maxsize=None)
def base() -> bd.Part:
    """Technic plate (top z = -30) with the differential model's towers and ribs, a raised bed for the motor frame
    (pins up into its lower side holes at every position) and an index fin on the pinion tower."""
    plate = box(*BASE_X, *BASE_Y, BASE_BOT, BASE_TOP)
    for sx in (0, 1):  # rounded plan corners
        for sy in (0, 1):
            cx = BASE_X[sx] + (3.7 if sx == 0 else -3.7)
            cy = BASE_Y[sy] + (3.7 if sy == 0 else -3.7)
            corner = box(cx - 3.7 if sx == 0 else cx, cx if sx == 0 else cx + 3.7,
                         cy - 3.7 if sy == 0 else cy, cy if sy == 0 else cy + 3.7, BASE_BOT - 1, BASE_TOP + 1)
            plate = plate - corner + bd.Pos(cx, cy, BASE_BOT) * cyl(3.7, BASE_T)
    bed = box(BED[0], BED[1], BED[2], BED[3], BASE_TOP - 0.01, FRAME_LOW_Z)
    body = plate + d.frame() + bed + tower_index_fin()
    xs, ys = grid_xy()
    m = CB_D / 2 + 0.3
    tools = []
    for x in xs:
        for y in ys:
            if any(x0 - m < x < x1 + m and y0 - m < y < y1 + m for x0, x1, y0, y1 in UPRIGHTS):
                continue
            on_bed = BED[0] + 3.6 <= x <= BED[1] - 3.6 and BED[2] + 3.6 <= y <= BED[3] - 3.6
            top = FRAME_LOW_Z if on_bed else BASE_TOP
            tools.append(pin_hole(x, y, top, top - BASE_BOT, cb_top=True, cb_bottom=True))
    return body - reduce(lambda a, b: a + b, tools)


# --- placements (world) ---

def motor_frame_loc(x: float) -> bd.Location:
    return frame_at((x - FRAME_SHAFT_X, FRAME_Y, 0.0), (1, 0, 0), (0, 1, 0))


@lru_cache(maxsize=None)
def _step(name: str) -> bd.Part:
    return e._read(name)


def motor_frame(x: float = 0.0) -> bd.Part:
    return motor_frame_loc(x) * _step("tt_motor_technic_SideTop")


def tt_motor(x: float = 0.0) -> bd.Part:
    """Simplified TT gear motor as in ../../electric_differential (frame coordinates first)."""
    t, top = 18.8, MOTOR_TOP_LOCAL
    gb = bd.Pos((23.5 - 13.5) / 2, 0, top - t / 2) * bd.Box(37.0, 22.0, t)
    for s in (1, -1):
        gb -= bd.Pos(21.7, s * 9.3, top - t / 2) * bd.Box(3.7, 3.7, t + 1)
        gb += bd.Pos(19.85, s * 7.41, top - t) * bd.Cylinder(3.6, t, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    can = bd.Pos(-13.5 - 10, 0, top - t / 2) * (bd.Rot(0, 90, 0) * bd.Cylinder(10.0, 20.0) & bd.Box(20.0, 30.0, 15.4))
    shaft = bd.Pos(FRAME_SHAFT_X, 0, top) * bd.extrude(bd.Face(bd.Wire.make_circle(2.7)) & bd.Rectangle(3.7, 6), 8.5)
    tab = bd.Pos(23.5 + 2.0, 0, top - 18.8 + 7.45 + 1.9) * bd.Box(4.0, 6.0, 3.6)
    return motor_frame_loc(x) * (gb + can + shaft + tab)


def coupler(x: float = 0.0, theta: float = 0.0) -> bd.Part:
    return frame_at((x, COUPLER_Y0, 0.0), (math.cos(math.radians(theta)), 0, -math.sin(math.radians(theta))),
                    (0, 1, 0)) * _step("tt_technic_coupler")


def crank(x: float = 0.0, theta: float = 0.0) -> bd.Part:
    """The differential model's FreeCAD crank on A1's free end (hand mode: motor and coupler lifted off)."""
    c = _crank_step()  # local +z = hub axis (bed side faces the tower), arm along +x
    up = (math.sin(math.radians(theta)), 0, math.cos(math.radians(theta)))
    return frame_at((x, B_Y0 - d.GAP, 0.0), up, (0, -1, 0)) * c


@lru_cache(maxsize=None)
def _crank_step() -> bd.Part:
    from cadgen import read_step

    return read_step(str(d.FREECAD_CRANK_STEP))


def bearing_tower_at(x: float = 0.0) -> bd.Part:
    return bd.Pos(x, 0, 0) * bearing_tower()
