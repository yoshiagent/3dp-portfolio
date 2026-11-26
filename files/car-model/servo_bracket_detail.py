"""Close-up of the MG90S side brackets: the front of the chassis (cut at y = -40), the servo holder with the servo,
the steering frame and the two brackets, straight ahead, for the preview pictures.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from cadgen import build123d as bd  # noqa: E402
from cadgen import step  # noqa: E402

KEEP = ("holder base", "holder cover", "servo", "frame", "servo bracket right", "servo bracket left")


@step(out="../STEP/servo_bracket_detail.step")
def servo_bracket_detail():
    import car_parts as c
    from car_assembly import bracket_parts, labeled, steering_parts

    front = c.box(-60, 60, -40, 50, 0, 60)
    ch = labeled(c.chassis() & front, "chassis (front)", "#5a6470")
    parts = {**steering_parts(), **bracket_parts()}
    return bd.Compound(children=[ch] + [parts[k] for k in KEEP], label="servo_bracket_detail")


if __name__ == "__main__":
    servo_bracket_detail()
