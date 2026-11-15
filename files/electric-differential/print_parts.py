"""Print files for the motor-driven differential.

Plate 1 (electric_base): the base alone, flat side down (155.4 x 207.4 mm).
Plate 2 (electric_gears): every other printed part of the differential, each turned onto its flat face and
laid out in a grid. Coupler, motor frame, battery box + lid and breadboard frame are the existing prints in
../../tt_motor_adapter.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cadgen import build123d as bd  # noqa: E402
from cadgen import stl, threemf  # noqa: E402

import electric_parts as e  # noqa: E402
from electric_parts import d  # noqa: E402

X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def gear_plate_parts() -> list[tuple[str, bd.Part]]:
    """(label, part in print pose); the poses follow differential_model/src/part_*.py."""
    pose = d.print_pose
    return [
        ("ring_half", pose(e.ring_half(), x_dir=Y, z_dir=(-1, 0, 0))),  # back face (x = 15.4) on the bed
        ("pinion", pose(e.pinion(), x_dir=X, z_dir=Y)),  # back face (y = -35) on the bed
        ("carrier_left", pose(d.carrier_left(), x_dir=Y, z_dir=X)),
        ("side_gear_1", pose(d.side_gear(1), x_dir=Y, z_dir=(-1, 0, 0))),
        ("side_gear_2", pose(d.side_gear(1), x_dir=Y, z_dir=(-1, 0, 0))),
        ("spider_1", pose(d.spider(1), x_dir=X, z_dir=(0, 0, -1))),
        ("spider_2", pose(d.spider(1), x_dir=X, z_dir=(0, 0, -1))),
        ("wheel_1", pose(d.wheel(1), x_dir=Y, z_dir=X)),
        ("wheel_2", pose(d.wheel(1), x_dir=Y, z_dir=X)),
        ("sleeve_right", pose(e.sleeve(1), x_dir=Y, z_dir=X)),
        ("sleeve_left", pose(e.sleeve(-1), x_dir=Y, z_dir=X)),
        ("cross_pin", pose(d.cross_pin(), x_dir=Z, z_dir=(-1, 0, 0))),  # D flat on the bed
        ("axle_1", pose(d.axle(1), x_dir=X, z_dir=(0, 0, -1))),
        ("axle_2", pose(d.axle(1), x_dir=X, z_dir=(0, 0, -1))),
        ("input_axle_3L", pose(e.pinion_shaft(), x_dir=Y, z_dir=(-1, 0, 0))),
    ]


def layout(parts: list[tuple[str, bd.Part]], width: float = 200.0, gap: float = 6.0) -> bd.Compound:
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
    return bd.Compound(children=placed, label="electric_gears")


@stl(out="../STL/electric_base.stl")
@threemf(out="../3MF/electric_base.3mf")
def electric_base():
    base = d.print_pose(e.frame(), x_dir=X, z_dir=Z)
    base.label = "electric_base"
    return base


@threemf(out="../3MF/electric_gears.3mf")
def electric_gears():
    return layout(gear_plate_parts())


if __name__ == "__main__":
    electric_base()
    electric_gears()
