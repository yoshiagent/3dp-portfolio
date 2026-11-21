"""Reference assembly of the MG90S front steering at straight ahead: side-loading servo holder, stand-in MG90S
and the printed double-arm horn, crank, tie rods, knuckles, the steering frame, and stand-ins for the LEGO front
wheels (balloon tyre 55976, 56 x 26, on rim 56145, 30.4 x 20), each drawn as rim + tyre.

`posed_parts(theta)` places everything for a crank angle theta (deg, + = counter-clockwise seen from above)
using the linkage solver in ../tools/linkage.py; checks use it to sweep the steering range.
Vehicle coordinates: x right, y forward, z up from the ground; servo shaft on the z axis.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

from cadgen import build123d as bd
from cadgen import read_step, step

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "tools"))
sys.path.insert(0, str(ROOT / "mg90s_technic" / "src"))
import linkage  # noqa: E402
import mg90s_side_assembly as servo_parts  # noqa: E402

OUT = HERE.parent / "out"
HOLDER = ROOT / "tt_motor_adapter" / "out"
Z_TAB = 31.8              # servo tab underside (holder bottom 8 above the ground)
# printed double-arm horn (../../mg90s_arm): boss down on the servo spline, its flat plate on top
HORN_STEP = ROOT / "mg90s_arm" / "out" / "mg90s_arm_double.step"
HORN_TOP = 13.5 - 3.0 + 2.0 + 2.0   # plate top above the tab underside: spline top - spline hole depth + boss + plate
Z_CRANK = Z_TAB + HORN_TOP - 1.9    # crank underside: its recess (1.9, horn plate 2 - 0.1) sits on the plate
Z_CRANK_DIRECT = Z_TAB + 13.5 - 3.0  # direct crank underside: the shaft top bottoms in its 3 deep spline hole
Z_ROD = 40.2
KP = 40.0
WHEEL_D, WHEEL_W = 56.0, 26.0
WHEEL_IN = 5.8 + 7.4 + 0.6        # wheel inner face from the kingpin
WHEEL_Z = 27.9


def labeled(shape, label, color):
    shape.label = label
    shape.color = bd.Color(color)
    return shape


def holder_to_vehicle(shape, turn=0.0):
    """Holder coordinates (shaft on z, short end +x, tab underside z = 0) -> vehicle; `turn` spins about the shaft."""
    return bd.Pos(0, 0, Z_TAB) * bd.Rot(0, 0, 90 + turn) * shape


RIM_D, RIM_W = 30.4, 20.0       # LEGO rim 56145 (30.4 x 20) in tyre 55976 (56 x 26 balloon), centred


def _cyl(r, h, z=0.0):
    return bd.Pos(0, 0, z) * bd.Cylinder(r, h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def _cross_bore(z0, z1, clear=0.15):
    """LEGO cross hole (axle 4.8, arms 1.8) along z, `clear` per side."""
    d, w = 4.8 + 2 * clear, 1.8 + 2 * clear
    prof = bd.Face(bd.Wire.make_circle(d / 2)) & (bd.Rectangle(d + 2, w) + bd.Rectangle(w, d + 2))
    return bd.Pos(0, 0, z0 - 0.1) * bd.extrude(prof, z1 - z0 + 0.2)


def _on_axle(s, shape):
    """Local wheel frame (axis +z, tyre inner face z = 0) -> knuckle frame of side s at straight ahead."""
    return bd.Location(bd.Plane(origin=(s * WHEEL_IN, 0, WHEEL_Z), x_dir=(0, 0, 1), z_dir=(s, 0, 0))) * shape


def wheel(s) -> bd.Part:
    """Stand-in for the LEGO rim 56145: dished on both faces, hub with the cross hole for the axle with stop."""
    z0, r = (WHEEL_W - RIM_W) / 2, RIM_D / 2
    rim = _cyl(r, RIM_W, z0) - _cyl(12.6, 7.0, z0 - 0.1) - _cyl(12.6, 7.0, z0 + RIM_W - 6.9)
    return _on_axle(s, rim.fuse(_cyl(4.5, RIM_W, z0)).clean() - _cross_bore(z0, z0 + RIM_W))


def tyre(s, knob_h=2.2, n=20) -> bd.Part:
    """Stand-in for the LEGO balloon tyre 55976 (56 x 26): a core with two staggered rows of tread blocks."""
    r_out, w = WHEEL_D / 2, WHEEL_W
    core = _cyl(r_out - knob_h, w) - _cyl(RIM_D / 2 + 0.05, w + 0.2, -0.1)
    gap = 1.2
    row_w = (w - gap) / 2
    block_w = 2 * math.pi * r_out / n * 0.55
    blocks = [bd.Rot(0, 0, (i + 0.5 * k) * 360.0 / n) * bd.Pos(r_out - knob_h - 0.3, 0, k * (row_w + gap)) *
              bd.Box(knob_h + 0.6, block_w, row_w, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
              for k in range(2) for i in range(n)]
    return _on_axle(s, core.fuse(*blocks).clean() & _cyl(r_out, w))


def posed_parts(theta: float = 0.0, direct: bool = False):
    """direct=True: the steer_crank_direct version (no horn) instead of the horn and crank."""
    link = linkage.Linkage(**linkage.CHOSEN)
    th = math.radians(theta)
    parts = {
        "holder base": labeled(holder_to_vehicle(read_step(str(HOLDER / "mg90s_side_base.step"))), "holder base", "lightgray"),
        "holder cover": labeled(holder_to_vehicle(read_step(str(HOLDER / "mg90s_side_cover.step"))), "holder cover", "steelblue"),
        "servo": labeled(holder_to_vehicle(servo_parts.servo()), "MG90S (not printed)", "#4b2a7a"),
        # built plate-down (z 0..2, boss 2..4): flip it so the boss points down onto the servo
        "horn": labeled(bd.Pos(0, 0, Z_TAB + HORN_TOP) * bd.Rot(0, 0, theta) * bd.Rot(180, 0, 0) *
                        read_step(str(HORN_STEP)), "servo horn (double arm)", "white"),
        "crank": labeled(bd.Pos(0, 0, Z_CRANK) * bd.Rot(0, 0, theta) * read_step(str(OUT / "steer_crank.step")),
                         "crank", "orange"),
        "frame": labeled(read_step(str(OUT / "steer_frame.step")), "steering frame", "dimgray"),
    }
    if direct:  # the crank without a horn, its spline hole straight on the servo shaft
        del parts["horn"]
        parts["crank"] = labeled(bd.Pos(0, 0, Z_CRANK_DIRECT) * bd.Rot(0, 0, theta) *
                                 read_step(str(OUT / "steer_crank_direct.step")), "crank (direct)", "orange")
    for s, side in ((1, "right"), (-1, "left")):
        delta, _ = link.wheel_angle(s, th)
        kp = bd.Pos(s * KP, linkage.YK, 0) * bd.Rot(0, 0, delta)
        parts[f"knuckle {side}"] = labeled(kp * read_step(str(OUT / f"steer_knuckle_{side}.step")),
                                           f"knuckle {side}", "gold")
        parts[f"wheel {side}"] = labeled(kp * wheel(s), f"wheel {side} (LEGO, not printed)", "#a0a5a9")
        parts[f"tyre {side}"] = labeled(kp * tyre(s), f"tyre {side} (LEGO, not printed)", "#1c1c1c")
        # tie rod from the crank pin to the arm end
        jx, jy = linkage.rot((s * link.c, link.j), th)
        a0 = link.arm0[s]
        ax, ay = linkage.rot((a0[0] - s * KP, a0[1] - linkage.YK), math.radians(delta))
        ax, ay = ax + s * KP, ay + linkage.YK
        phi = math.degrees(math.atan2(ay - jy, ax - jx))
        parts[f"tie rod {side}"] = labeled(bd.Pos(jx, jy, Z_ROD) * bd.Rot(0, 0, phi) *
                                           read_step(str(OUT / "steer_tierod.step")), f"tie rod {side}", "tomato")
    return parts


@step(out="../STEP/steering_assembly.step")
def steering_assembly():
    return bd.Compound(children=list(posed_parts(0.0).values()), label="mg90s_steering")


if __name__ == "__main__":
    steering_assembly()
