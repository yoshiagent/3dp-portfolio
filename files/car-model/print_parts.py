"""Print files for the car model's new parts.

Plate 1 (car_drive_base): the drive module's base alone, flat bottom down (103.4 x 139.4 mm, towers 39 mm high).
Plate 2 (car_chassis): the middle-level chassis alone, a flat 7.8 mm Technic plate (103.4 x 143.4 mm).
Plate 3 (car_parts): deck, two deck posts, two rear wheels (web face down), spacers, collars and two 10L axles
(a real LEGO 10L axle works too).
After rebuilding, close the mesher's T-junctions:
    python ../../web/tools/fix_tjunctions.py ../3MF/car_drive_base.3mf ../3MF/car_chassis.3mf ../3MF/car_parts.3mf ../STL/car_drive_base.stl ../STL/car_chassis.stl
Everything else is an existing print: the steering parts (../../mg90s_steering), the differential parts and
motor frame (../../electric_differential, ../../tt_motor_adapter), battery box + lid, breadboard frame.
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


def small_parts() -> list[tuple[str, bd.Part]]:
    pose = d.print_pose
    out = [("deck", pose(c.deck(), x_dir=X, z_dir=Z))]
    for k, yc, z0 in (("post_front", c.POSTS_Y[0], c.POSTS_Z0[0]), ("post_rear", c.POSTS_Y[1], c.POSTS_Z0[1])):
        out.append((k, pose(c.deck_post(yc, z0), x_dir=(0, 1, 0), z_dir=X)))  # on its side, layers along the length
    for s, side in ((1, "right"), (-1, "left")):
        out.append((f"rear_wheel_{side}", pose(c.rear_wheel(s), x_dir=(0, 1, 0), z_dir=(s, 0, 0))))
        out.append((f"spacer_{side}", pose(c.spacer(s), x_dir=(0, 1, 0), z_dir=(s, 0, 0))))
        out.append((f"collar_{side}", pose(c.collar(s), x_dir=(0, 1, 0), z_dir=(s, 0, 0))))
        out.append((f"axle_10L_{side}", pose(c.rear_axle(s), x_dir=X, z_dir=(0, 0, -1))))  # arm-tip flat down
    return out


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


@threemf(out="../3MF/car_parts.3mf")
def car_parts():
    return layout(small_parts())


if __name__ == "__main__":
    car_drive_base()
    car_chassis()
    car_parts()
