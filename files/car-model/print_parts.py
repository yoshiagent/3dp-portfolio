"""Print files for the car model's new parts.

Plate 1 (car_drive_base): the drive module's base alone, flat bottom down (103.4 x 139.4 mm, towers 39 mm high).
Plate 2 (car_chassis): the middle-level chassis alone, a flat 7.8 mm Technic plate (103.4 x 143.4 mm).
The two 18650 trays (battery_trays) go into 3MF/car_parts.3mf and plate 4 of 3MF/car_all_plates.3mf through print_all.py, whose meshes are repaired watertight.
LEGO standard parts are not printed: 81.6 x 15 motorcycle rear tyres on rims, 10L cross axles x2, half bushes x4
(the axle collars, inside the outer towers and outside the hubs), bushes x2 (the wheel spacers); car_parts.py keeps their shapes as stand-ins.
After rebuilding, close the mesher's T-junctions:
    python ../../web/tools/fix_tjunctions.py ../3MF/car_drive_base.3mf ../3MF/car_chassis.3mf ../STL/car_drive_base.stl ../STL/car_chassis.stl
Everything else is an existing print: the steering parts (../../mg90s_steering), the differential parts and
motor frame (../../electric_differential, ../../tt_motor_adapter), breadboard frame.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cadgen import build123d as bd  # noqa: E402
from cadgen import stl, threemf  # noqa: E402

import car_parts as c  # noqa: E402
from car_parts import d  # noqa: E402

X, Z = (1, 0, 0), (0, 0, 1)


def battery_trays() -> list[tuple[str, bd.Part]]:
    """The two 18650 trays: the 12-unit single tray of ../../tt_motor_adapter (the same part twice), as
    modelled, floor on the bed and pocket up."""
    tray = c.e._read("battery18650_12_technic")
    return [("battery_tray_right", tray), ("battery_tray_left", tray)]


def layout(parts: list[tuple[str, bd.Part]], width: float = 220.0, gap: float = 6.0) -> bd.Compound:
    """Shelf packing, largest first, inside `width` along X."""
    parts = sorted(parts, key=lambda p: -p[1].bounding_box().size.Y)
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    for label, part in parts:
        bb = part.bounding_box()
        if x and x + bb.size.X > width:
            x, y, row_h = 0.0, y + row_h + gap, 0.0
        p = bd.Pos(x - bb.min.X, y - bb.min.Y, 0) * part
        p.label = label
        placed.append(p)
        x += bb.size.X + gap
        row_h = max(row_h, bb.size.Y)
    return bd.Compound(children=placed, label="car_parts")


@stl(out="../STL/car_drive_base.stl")
@threemf(out="../3MF/car_drive_base.3mf")
def car_drive_base():
    part = d.print_pose(c.drive_base(), x_dir=X, z_dir=Z)
    part.label = "car_drive_base"
    return part


@stl(out="../STL/car_chassis.stl")
@threemf(out="../3MF/car_chassis.3mf")
def car_chassis():
    part = d.print_pose(c.chassis(), x_dir=X, z_dir=Z)
    part.label = "car_chassis"
    return part


if __name__ == "__main__":
    car_drive_base()
    car_chassis()
