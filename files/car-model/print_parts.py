"""Print files for the car model's new parts.

car_chassis: the one chassis, flat bottom down (103.4 x 195.4 mm, towers 34.5 mm high).
The four posts (small_parts) go into plate 2 of 3MF/car_all_plates.3mf and 3MF/car_parts.3mf through
print_all.py, whose meshes are repaired watertight.
LEGO standard parts are not printed: 81.6 x 15 motorcycle rear tyres on rims, cross axles (3L, 8L, 12L), half
bushes x4 (the axle collars, inside the outer towers and outside the hubs), bushes x2 (the wheel spacers);
car_parts.py keeps their shapes as stand-ins.
After rebuilding, close the mesher's T-junctions:
    python ../../web/tools/fix_tjunctions.py ../3MF/car_chassis.3mf ../STL/car_chassis.stl
Everything else is an existing print: the steering parts (../../mg90s_steering), the differential parts
(../../electric_differential), the 9L motor frame, the coupler, the breadboard frame and the 18650 tray
(../../tt_motor_adapter).
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
    """The four posts (all the same), each on its side so the layers run along its length."""
    pose = d.print_pose
    return [(k, pose(c.deck_post(c.POSTS_Y[r], s), x_dir=(0, 1, 0), z_dir=X))
            for k, s, r in (("post_front_right", 1, 0), ("post_rear_right", 1, 1), ("post_front_left", -1, 0),
                            ("post_rear_left", -1, 1))]


@stl(out="../STL/car_chassis.stl")
@threemf(out="../3MF/car_chassis.3mf")
def car_chassis():
    part = d.print_pose(c.chassis(), x_dir=X, z_dir=Z)
    part.label = "car_chassis"
    return part


if __name__ == "__main__":
    car_chassis()
