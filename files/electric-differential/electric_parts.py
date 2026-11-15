"""Motor-driven differential teaching model: the input shaft on the model's centre line.

Built on ../../differential_model/src/differential_parts.py (same carrier, cross pin, spiders, side gears,
axles, sleeves and wheels). What changes:

* Pinion axis on the centre line (x = 0). Its axis must cross the axle axis at the ring gear's cone apex, so
  the ring apex moves to the differential centre. A pinion that close would hit the turning carrier, so the
  ring grows to 44 teeth (4:1): the pinion then sits 26.5 mm out, clear of the carrier (radius 21.9).
* The ring gear wraps round the right carrier plate; inside radius 23 everything left of the plate face is
  cut away so the right side gear and the posts keep their room.
* The crank is replaced by a TT motor in its Technic frame (tt_motor_technic_SideTop), joined by the
  TT -> LEGO coupler (tt_technic_coupler) and a LEGO 3L cross axle.
* One printed base: a 7.8 mm Technic plate (8 mm hole grid) carrying the three towers, three piers for the
  motor frame, and pin holes under the 3xAAA battery box and the breadboard frame.

Units mm. World frame as in differential_parts: axles along X, input shaft along Y (motor at -Y), +Z up,
origin at the differential centre. Hole grid: x = 4 (mod 8), y = 0 (mod 8).
"""
from __future__ import annotations

import math
import sys
from functools import reduce
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "differential_model" / "src"))

from cadgen import build123d as bd  # noqa: E402

import differential_parts as d  # noqa: E402
from differential_parts import BevelSpec, bevel_gear, cross_bore, cyl, place, round_bore  # noqa: E402

TT_OUT = HERE.parents[1] / "tt_motor_adapter" / "out"

# --- ring and pinion: apex at the origin ---
RING_TEETH, PINION_TEETH = 44, 11  # 4:1
PINION_DELTA = math.degrees(math.atan2(PINION_TEETH, RING_TEETH))
RING = BevelSpec(RING_TEETH, 90.0 - PINION_DELTA, d.RING_FACE, d.RING_SHIFT)
PINION = BevelSpec(PINION_TEETH, PINION_DELTA, d.RING_FACE, d.PINION_SHIFT)

RING_BACK_X = 15.4  # ring back face (print bed); plate x = 11 .. 15.4 holds the post sockets
RING_IN_R = 23.0  # left of the plate face (x < 11) the ring keeps only its rim outside this radius
PINION_TOWER_Y1 = -35.0  # pinion back face bears on this tower face
PINION_TOWER_Y0 = PINION_TOWER_Y1 - 4.0
PINION_END_WALL = 1.5  # the pinion's cross hole is blind: the input axle bottoms here

# --- base ---
PITCH = 8.0
BASE_T = 7.8
BASE_Z1 = -35.5  # base top: 2.2 mm under the ring (outer tip radius 33.3)
BASE_Z0 = BASE_Z1 - BASE_T
BASE_X0, BASE_X1 = -55.7, 99.7
BASE_Y0, BASE_Y1 = -99.7, 107.7
PIN_D, CB_D, CB_DEPTH = 4.94, 6.2, 0.8  # as in the Technic frames
TOWER_X0, TOWER_X1 = d.TOWER_X0, d.TOWER_X1
TOWER_HALF_W, TOWER_TOP = 7.0, 7.0
PTOWER_HALF_W = 6.0

# --- motor, coupler, input axle ---
TT_SHAFT_OUT = 8.5  # TT output shaft length beyond the gearbox face (assumed; measure the motor)
MOTOR_FRAME_SHAFT_X = 12.0  # shaft position in the frame's own coordinates
MOTOR_TOP_LOCAL = 11.9 - 1.9 - 0.1  # gearbox top face in frame coordinates (lips flush with the frame top)
MOTOR_C_Y = -72.0  # frame centre y (on the hole grid); frame top face at y = -60.1
COUPLER_L, COUPLER_TT_DEPTH, COUPLER_WEB = 16.5, 7.5, 1.0
INPUT_AXLE_L = 3 * 8.0 - 0.2  # LEGO 3L

# --- electronics on the base ---
BATTERY_C = (68.0, -60.0)  # 3xAAA box turned 90 deg: its 10-pitch length along Y
BREADBOARD_C = (4.0, 72.0)  # one pitch further back: the ring (radius 33.3) clears it by 9 mm


def pinion_bore_end() -> float:
    """|y| where the blind cross hole of the pinion ends (inner end wall)."""
    return PINION.outer_root().Z * PINION.k + PINION_END_WALL


# --- gears ---

def ring_half() -> bd.Part:
    """44-tooth ring gear joined to the right carrier plate (inner face x = 11, back face x = 15.4)."""
    g = bevel_gear(RING, back_z=RING_BACK_X)
    g -= cyl(RING_IN_R, d.CARRIER_IN + 1, -1)  # room for the right side gear and the posts
    g -= round_bore(d.CARRIER_BORE_D, 0, RING_BACK_X)
    for s in (1, -1):  # local +Y = world -Z in this placement
        g -= bd.Pos(0, s * (d.POST_Z0 + d.POST_Z1) / 2, d.CARRIER_IN) * cyl(d.SOCKET_D / 2, d.SOCKET_DEPTH)
    return place(g, (0, 0, 0), (0, -1, 0), (1, 0, 0))


def pinion() -> bd.Part:
    back = -PINION_TOWER_Y1
    g = bevel_gear(PINION, back_z=back, extend=True)  # back face bears on the tower
    g = bd.Rot(0, 0, 180 / PINION_TEETH) * g  # a space faces the ring at the mesh (+x)
    g -= cross_bore(pinion_bore_end(), back)
    return place(g, (0, 0, 0), (1, 0, 0), (0, -1, 0))


def pinion_shaft() -> bd.Part:
    """LEGO 3L cross axle: bottoms in the pinion, its outer end runs into the coupler."""
    y0 = pinion_bore_end()
    return place(d.lego_axle(y0, INPUT_AXLE_L), (0, 0, 0), (1, 0, 0), (0, -1, 0))


def sleeve(side: int) -> bd.Part:
    x0 = (RING_BACK_X if side > 0 else d.CARRIER_IN + d.PLATE_T) + d.SLEEVE_END_CLEAR
    x1 = TOWER_X0 - d.SLEEVE_END_CLEAR
    tube = cyl(d.SLEEVE_OD / 2, x1 - x0) - round_bore(d.SLEEVE_ID, 0, x1 - x0)
    return place(tube, (side * x0, 0, 0), (0, 0, 1), (side, 0, 0))


# --- bought / earlier parts, placed ---

def _read(name: str) -> bd.Part:
    from cadgen import read_step

    return read_step(str(TT_OUT / f"{name}.step"))


def motor_frame_location() -> bd.Location:
    """Frame coordinates -> world: frame x -> world x (shaft at x = 0), frame z (shaft up) -> world +y,
    frame y -> world -z. Its lower long side, with 10 x 3 side holes, sits on the base piers."""
    return bd.Location(bd.Plane(origin=(-MOTOR_FRAME_SHAFT_X, MOTOR_C_Y, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0)))


def motor_frame() -> bd.Part:
    return motor_frame_location() * _read("tt_motor_technic_SideTop")


def coupler_y0() -> float:
    """Coupler's TT end: the shaft (assumed TT_SHAFT_OUT long) bottoms on the web."""
    return MOTOR_C_Y + MOTOR_TOP_LOCAL + TT_SHAFT_OUT - COUPLER_TT_DEPTH


def coupler() -> bd.Part:
    return bd.Location(bd.Plane(origin=(0, coupler_y0(), 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))) * _read(
        "tt_technic_coupler")


def tt_motor() -> bd.Part:
    """Simplified TT gear motor (not printed): gearbox block, motor can, output shaft, in frame coordinates
    first. Gearbox 22.6 wide x 18.8 thick from the stop face (x = 23.5) back 37 mm; can 20 mm long."""
    t, top = 18.8, MOTOR_TOP_LOCAL
    gb = bd.Pos((23.5 - 13.5) / 2, 0, top - t / 2) * bd.Box(37.0, 22.0, t)
    for s in (1, -1):  # rounded front corners, inside the frame's concave arcs (r 3.65 about x 19.85, y 7.41)
        gb -= bd.Pos(21.7, s * 9.3, top - t / 2) * bd.Box(3.7, 3.7, t + 1)
        gb += bd.Pos(19.85, s * 7.41, top - t) * bd.Cylinder(3.6, t, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    can = bd.Pos(-13.5 - 10, 0, top - t / 2) * (bd.Rot(0, 90, 0) * bd.Cylinder(10.0, 20.0)
                                                & bd.Box(20.0, 30.0, 15.4))
    shaft = bd.Pos(MOTOR_FRAME_SHAFT_X, 0, top) * bd.extrude(
        bd.Face(bd.Wire.make_circle(2.7)) & bd.Rectangle(3.7, 6), TT_SHAFT_OUT)
    tab = bd.Pos(23.5 + 2.0, 0, top - 18.8 + 7.45 + 1.9) * bd.Box(4.0, 6.0, 3.6)
    return motor_frame_location() * (gb + can + shaft + tab)


def battery_box() -> bd.Part:
    x, y = BATTERY_C
    return bd.Pos(x, y, BASE_Z1 + 11.9) * bd.Rot(0, 0, 90) * _read("aaa3_battery_box_technic")


def battery_lid() -> bd.Part:
    """Lid flush with the box's front end (-y), the last 3 pitches toward the breadboard left open
    for the switch and the leads."""
    x, y = BATTERY_C
    return bd.Pos(x, y - 39.7 + 27.7, BASE_Z1 + 23.8 + 3.9) * _read("aaa3_battery_lid_technic")


def breadboard_frame() -> bd.Part:
    x, y = BREADBOARD_C
    return bd.Pos(x, y, BASE_Z1 + 7.9) * _read("breadboard_technic")


# --- base ---

def _box(x0, x1, y0, y1, z0, z1) -> bd.Part:
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


PIER_COLS = ((-48.0, -40.0), (-8.0, 0.0), (16.0, 24.0))  # under the motor frame's hole columns
PIER_ROWS = (MOTOR_C_Y - PITCH, MOTOR_C_Y, MOTOR_C_Y + PITCH)
MOTOR_FRAME_Z = -19.7  # frame's lower side face


def piers() -> list[tuple[bd.Part, tuple]]:
    out = []
    y0, y1 = MOTOR_C_Y - 11.8, MOTOR_C_Y + 11.8
    for cols in PIER_COLS:
        x0, x1 = max(cols[0] - 4.0, -51.7), min(cols[1] + 4.0, 27.7)
        out.append((_box(x0, x1, y0, y1, BASE_Z1 - 0.01, MOTOR_FRAME_Z), (x0, x1, y0, y1)))
    return out


def gusset(corner, run_dir, run, height, thick_dir, thick) -> bd.Part:
    return d.gusset(corner, run_dir, run, height, thick_dir, thick)


def towers() -> tuple[bd.Part, list[tuple]]:
    """Axle towers, pinion tower and their ribs; returns the solid and the footprints to keep hole-free."""
    z, t = BASE_Z1 - 0.01, TOWER_X1 - TOWER_X0
    h = TOWER_TOP - z
    parts, foot = [], []
    for s in (1, -1):
        xc = s * (TOWER_X0 + TOWER_X1) / 2
        parts.append(bd.Pos(xc, 0, z + h / 2) * bd.Box(t, 2 * TOWER_HALF_W, h))
        x_lo = min(s * TOWER_X0, s * TOWER_X1)
        for sy in (1, -1):  # front/back ribs
            parts.append(gusset((x_lo, sy * TOWER_HALF_W, z), (0, sy, 0), 8.0, 26.0, (1, 0, 0), t))
        parts.append(gusset((s * TOWER_X0, -t / 2, z), (-s, 0, 0), 6.0, 8.0, (0, 1, 0), t))  # under carrier
        parts.append(gusset((s * TOWER_X1, -t / 2, z), (s, 0, 0), 8.0, 20.0, (0, 1, 0), t))  # under wheel
        foot.append((x_lo, x_lo + t, -TOWER_HALF_W - 8, TOWER_HALF_W + 8))
        foot.append((min(s * 15.0, s * 33.0), max(s * 15.0, s * 33.0), -t / 2, t / 2))
    parts.append(bd.Pos(0, (PINION_TOWER_Y0 + PINION_TOWER_Y1) / 2, z + h / 2)
                 * bd.Box(2 * PTOWER_HALF_W, PINION_TOWER_Y1 - PINION_TOWER_Y0, h))
    parts.append(gusset((-t / 2, PINION_TOWER_Y0, z), (0, -1, 0), 8.0, 12.0, (1, 0, 0), t))
    parts.append(gusset((-t / 2, PINION_TOWER_Y1, z), (0, 1, 0), 8.0, 10.0, (1, 0, 0), t))
    foot.append((-PTOWER_HALF_W, PTOWER_HALF_W, PINION_TOWER_Y0 - 8, PINION_TOWER_Y1 + 8))
    return reduce(lambda a, b: a + b, parts), foot


def _hole_tool(x, y, z0, z1, cb_bottom=True) -> bd.Part:
    tool = bd.Pos(x, y, z0 - 0.1) * bd.Cylinder(PIN_D / 2, z1 - z0 + 0.2, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    tool += bd.Pos(x, y, z1 - CB_DEPTH - 0.02) * bd.Cylinder(CB_D / 2, CB_DEPTH + 0.12, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    if cb_bottom:
        tool += bd.Pos(x, y, z0 - 0.1) * bd.Cylinder(CB_D / 2, CB_DEPTH + 0.12, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    return tool


def grid_holes(foot: list[tuple]) -> list[tuple[float, float]]:
    """Plate grid points, leaving out those whose counterbore would reach a tower, rib or pier."""
    m = CB_D / 2 + 0.6
    pts = []
    x = BASE_X0 + 3.7
    while x < BASE_X1:
        y = BASE_Y0 + 3.7
        while y < BASE_Y1:
            if not any(x0 - m < x < x1 + m and y0 - m < y < y1 + m for x0, x1, y0, y1 in foot):
                pts.append((round(x, 3), round(y, 3)))
            y += PITCH
        x += PITCH
    return pts


def frame() -> bd.Part:
    """The printed base: Technic plate, towers with bearings, motor piers."""
    plate = _box(BASE_X0, BASE_X1, BASE_Y0, BASE_Y1, BASE_Z0, BASE_Z1)
    tw, foot = towers()
    pier_parts = piers()
    body = plate + tw + reduce(lambda a, b: a + b, [p for p, _ in pier_parts])
    foot += [f for _, f in pier_parts]
    # bearings: axle towers along X, pinion tower along Y
    for s in (1, -1):
        body -= place(round_bore(d.BEARING_D, 0, TOWER_X1 - TOWER_X0), (s * TOWER_X0, 0, 0), (0, 0, 1), (s, 0, 0))
    body -= place(round_bore(d.BEARING_D, 0, PINION_TOWER_Y1 - PINION_TOWER_Y0), (0, PINION_TOWER_Y1, 0),
                  (1, 0, 0), (0, -1, 0))
    tools = [_hole_tool(x, y, BASE_Z0, BASE_Z1) for x, y in grid_holes(foot)]
    for x0, x1 in PIER_COLS:  # blind pin holes in the pier tops, one pin deep
        for x in (x0, x1):
            for y in PIER_ROWS:
                tools.append(_hole_tool(x, y, MOTOR_FRAME_Z - 9.0, MOTOR_FRAME_Z, cb_bottom=False))
    body -= bd.Compound(tools)
    return body
