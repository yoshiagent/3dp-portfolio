"""Reference assembly: MG90S Technic frame + lid + horn adapter (FreeCAD exports in ../tt_motor_adapter/out) with a
stand-in MG90S and its cross horn. The stand-in is the one in mg90s_side_assembly.py (shaped after the user's
photo: case top 4.7 above the tabs, round gear housing dia. 14.6 offset from the shaft), so both holder versions
are checked against the same servo.

Frame coordinates: output shaft on z through (0, 0); z = 0 is the frame top = underside of the servo tabs.
"""
from __future__ import annotations

from pathlib import Path

from cadgen import build123d as bd
from cadgen import read_step, step

from mg90s_side_assembly import ARM_T, HORN_TOP, horn, servo

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "tt_motor_adapter" / "out"


def labeled(shape, label, color):
    shape.label = label
    shape.color = bd.Color(color)
    return shape


@step(out="../STEP/mg90s_assembly.step")
def mg90s_assembly():
    adapter_z = HORN_TOP - ARM_T       # adapter bottom level with the arms' underside
    parts = [
        labeled(read_step(str(OUT / "mg90s_technic_frame.step")), "frame", "lightgray"),
        labeled(read_step(str(OUT / "mg90s_technic_lid.step")), "lid", "steelblue"),   # sits on the frame top
        labeled(servo(), "MG90S (not printed)", "#4b2a7a"),
        labeled(horn(), "cross horn (not printed)", "white"),
        labeled(bd.Pos(0, 0, adapter_z) * read_step(str(OUT / "mg90s_horn_adapter.step")), "horn_adapter", "orange"),
    ]
    return bd.Compound(children=parts, label="mg90s_technic")


if __name__ == "__main__":
    mg90s_assembly()
