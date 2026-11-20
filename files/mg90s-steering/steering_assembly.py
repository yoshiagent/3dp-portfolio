"""Reference assembly of the MG90S front steering at straight ahead: side-loading servo holder, stand-in MG90S
and the printed double-arm horn, crank, tie rods, knuckles, 56 x 26 stand-in wheels and the steering frame.

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


def wheel(s) -> bd.Part:
    w = bd.Rot(0, 90, 0) * bd.Cylinder(WHEEL_D / 2, WHEEL_W)
    return bd.Pos(s * (WHEEL_IN + WHEEL_W / 2), 0, WHEEL_Z) * w


def posed_parts(theta: float = 0.0):
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
    for s, side in ((1, "right"), (-1, "left")):
        delta, _ = link.wheel_angle(s, th)
        kp = bd.Pos(s * KP, linkage.YK, 0) * bd.Rot(0, 0, delta)
        parts[f"knuckle {side}"] = labeled(kp * read_step(str(OUT / f"steer_knuckle_{side}.step")),
                                           f"knuckle {side}", "gold")
        parts[f"wheel {side}"] = labeled(kp * wheel(s), f"wheel {side} (LEGO, not printed)", "#222222")
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
