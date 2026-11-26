"""Reference assembly of the side-loading MG90S holder: base + side cover (FreeCAD exports in
../tt_motor_adapter/out), a stand-in MG90S shaped after the user's photo, its cross horn and the horn adapter.

Same coordinates as the top-loading set (mg90s_assembly.py): output shaft on z through (0, 0), z = 0 at the
underside of the servo tabs. The servo slides in along -y; the cover goes on from +y.
"""
from __future__ import annotations

from pathlib import Path

from cadgen import build123d as bd
from cadgen import read_step, step

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "tt_motor_adapter" / "out"

BODY_L, BODY_W, SHAFT_END = 22.8, 12.2, 6.0
TAB_Z, TAB_T, TAB_SPAN, TAB_W = 18.5, 2.5, 32.5, 11.0
CASE_TOP = 4.7                     # top of the case above the tab underside (photo)
HOUSING_D, HOUSING_INSET, HOUSING_TOP = 14.6, 0.7, 9.9
SPLINE_TOP = 13.5
HORN_TOP = 35.5 - TAB_Z            # horn arms' top (datasheet height with a horn)
ARM_T, ARM_W, HORN_R = 1.8, 5.4, 11.0
BODY_CX = SHAFT_END - BODY_L / 2
HOUSING_CX = SHAFT_END - HOUSING_INSET - HOUSING_D / 2


def up():
    """Cylinder alignment: centred in x and y, base at its z."""
    return (bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)


def labeled(shape, label, color):
    shape.label = label
    shape.color = bd.Color(color)
    return shape


def servo() -> bd.Part:
    body = bd.Pos(BODY_CX, 0, (-TAB_Z + CASE_TOP) / 2) * bd.Box(BODY_L, BODY_W, TAB_Z + CASE_TOP)
    tabs = bd.Pos(BODY_CX, 0, TAB_T / 2) * bd.Box(TAB_SPAN, TAB_W, TAB_T)
    housing = bd.Pos(HOUSING_CX, 0, CASE_TOP) * bd.Cylinder(HOUSING_D / 2, HOUSING_TOP - CASE_TOP, align=up())
    spline = bd.Pos(0, 0, HOUSING_TOP) * bd.Cylinder(2.4, SPLINE_TOP - HOUSING_TOP, align=up())
    return body + tabs + housing + spline


def horn() -> bd.Part:
    hub = bd.Pos(0, 0, SPLINE_TOP) * bd.Cylinder(3.5, HORN_TOP - SPLINE_TOP, align=up())
    arms = None
    for ang in (45, -45):
        a = bd.Rot(0, 0, ang) * bd.Pos(0, 0, HORN_TOP - ARM_T / 2) * bd.Box(2 * HORN_R, ARM_W, ARM_T)
        arms = a if arms is None else arms + a
    return hub + arms


@step(out="../STEP/mg90s_side_assembly.step")
def mg90s_side_assembly():
    parts = [
        labeled(read_step(str(OUT / "mg90s_side_base.step")), "side_base", "lightgray"),
        labeled(read_step(str(OUT / "mg90s_side_cover.step")), "side_cover", "steelblue"),
        labeled(servo(), "MG90S (not printed)", "#4b2a7a"),
        labeled(horn(), "cross horn (not printed)", "white"),
        labeled(bd.Pos(0, 0, HORN_TOP - ARM_T) * read_step(str(OUT / "mg90s_horn_adapter.step")),
                "horn_adapter", "orange"),
    ]
    return bd.Compound(children=parts, label="mg90s_side")


if __name__ == "__main__":
    mg90s_side_assembly()
