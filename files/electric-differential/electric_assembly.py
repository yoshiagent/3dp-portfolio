"""Motor-driven differential teaching model: full assembly (geometry in electric_parts.py)."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cadgen import build123d as bd  # noqa: E402
from cadgen import step  # noqa: E402

import electric_parts as e  # noqa: E402
from electric_parts import d  # noqa: E402


def labeled(shape: bd.Part, label: str, color: str) -> bd.Part:
    shape.label = label
    shape.color = bd.Color(color)
    return shape


@step(out="../STEP/electric_assembly.step")
def electric_assembly():
    parts = [
        labeled(e.frame(), "frame", "dimgray"),
        labeled(d.carrier_left(), "carrier_left", "limegreen"),
        labeled(e.ring_half(), "ring_half", "orange"),
        labeled(d.side_gear(1), "side_gear:right", "gold"),
        labeled(d.side_gear(-1), "side_gear:left", "gold"),
        labeled(d.spider(1), "spider:top", "deepskyblue"),
        labeled(d.spider(-1), "spider:bottom", "deepskyblue"),
        labeled(d.cross_pin(), "cross_pin", "white"),
        labeled(e.pinion(), "pinion", "red"),
        labeled(e.pinion_shaft(), "pinion_shaft", "silver"),
        labeled(e.coupler(), "coupler", "white"),
        labeled(e.motor_frame(), "motor_frame", "lightgray"),
        labeled(e.tt_motor(), "tt_motor (not printed)", "yellow"),
        labeled(d.axle(1), "axle:right", "silver"),
        labeled(d.axle(-1), "axle:left", "silver"),
        labeled(e.sleeve(1), "sleeve:right", "khaki"),
        labeled(e.sleeve(-1), "sleeve:left", "khaki"),
        labeled(d.wheel(1), "wheel:right", "royalblue"),
        labeled(d.wheel(-1), "wheel:left", "magenta"),
        labeled(e.battery_box(), "battery_box", "slategray"),
        labeled(e.battery_lid(), "battery_lid", "lightslategray"),
        labeled(e.breadboard_frame(), "breadboard_frame", "steelblue"),
    ]
    return bd.Compound(children=parts, label="electric_differential")


if __name__ == "__main__":
    electric_assembly()
