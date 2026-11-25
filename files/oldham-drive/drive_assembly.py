"""Differential + Oldham coupling + TT motor on the drive base: assembled and exploded STEP.

The car model's drive module on its own (vehicle coordinates as in the car: x right, y forward, z up from the
ground, rear axle at y -120, z 40.8). The TT motor lies centred across the base, its shaft 12 mm left of the
differential's input axle; the three-part Oldham coupling joins the two. Rear axles, collars, spacers and LEGO
motorcycle wheels are included so the module stands and runs on the bench.

`drive_parts(spin)` places every part with the differential carrier turned `spin` degrees (pinion, input axle and
coupling turn 4x), for the checks. `exploded()` pulls the parts apart along their assembly directions.
Run with cadgen's python from oldham_drive/:  python src/drive_assembly.py
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from cadgen import build123d as bd  # noqa: E402
from cadgen import step  # noqa: E402

import drive_parts as c  # noqa: E402
from drive_parts import d, e  # noqa: E402

SIDE = {1: "right", -1: "left"}
TYRE = "#1c1c1c"


def labeled(shape: bd.Part, label: str, color: str) -> bd.Part:
    shape.label = label
    shape.color = bd.Color(color)
    return shape


def diff_parts(spin: float = 0.0) -> dict[str, bd.Part]:
    """Differential and drive train. Differential side +1 is the vehicle's left."""
    def carried(shape):  # parts that turn with the carrier
        return c.to_vehicle(bd.Rot(spin, 0, 0) * shape)

    v = c.to_vehicle
    p = {
        "carrier": labeled(carried(d.carrier_left()), "carrier", "limegreen"),
        "ring gear": labeled(carried(e.ring_half()), "ring gear 44T", "orange"),
        "cross pin": labeled(carried(d.cross_pin()), "cross pin", "white"),
        "spider top": labeled(carried(d.spider(1)), "spider gear:top", "deepskyblue"),
        "spider bottom": labeled(carried(d.spider(-1)), "spider gear:bottom", "deepskyblue"),
        "pinion": labeled(v(bd.Rot(0, 4 * spin, 0) * e.pinion()), "pinion 11T", "red"),
        "input axle": labeled(c.spin_y(c.input_axle(), 0.0, 4 * spin), "input axle 2L", "silver"),
        "hub motor": labeled(c.spin_y(c.oldham_hub_motor(), c.TT_SHAFT_X, 4 * spin), "Oldham hub (motor side)", "white"),
        "slider": labeled(c.oldham_slider(4 * spin), "Oldham slider", "tomato"),
        "hub axle": labeled(c.spin_y(c.oldham_hub_axle(), 0.0, 4 * spin), "Oldham hub (axle side)", "white"),
        "motor frame": labeled(c.motor_place(v(e.motor_frame())), "TT motor frame", "lightgray"),
        "tt motor": labeled(c.motor_place(v(e.tt_motor())), "TT motor (not printed)", "yellow"),
    }
    for ds in (1, -1):
        side = SIDE[-ds]
        p[f"side gear {side}"] = labeled(v(d.side_gear(ds)), f"side gear:{side}", "gold")
        p[f"inner sleeve {side}"] = labeled(v(e.sleeve(ds)), f"inner sleeve:{side}", "khaki")
    return p


def rear_parts() -> dict[str, bd.Part]:
    p = {}
    for s in (1, -1):
        side = SIDE[s]
        p[f"rear axle {side}"] = labeled(c.rear_axle(s), f"rear axle 10L:{side}", "silver")
        p[f"collar {side}"] = labeled(c.collar(s), f"axle collar:{side}", "khaki")
        p[f"spacer {side}"] = labeled(c.spacer(s), f"wheel spacer:{side}", "khaki")
        p[f"rear wheel {side}"] = labeled(c.rear_rim(s), f"rear wheel LEGO 81.6:{side}", "white")
        p[f"rear tyre {side}"] = labeled(c.rear_tyre(s), f"rear tyre LEGO 81.6:{side}", TYRE)
        p[f"outer collar {side}"] = labeled(c.collar(s, outer=True), f"outer collar:{side}", "gold")
    return p


def drive_parts(spin: float = 0.0) -> dict[str, bd.Part]:
    return {"drive base": labeled(c.drive_base(), "drive base", "dimgray"), **diff_parts(spin), **rear_parts()}


# exploded offsets (mm), the same directions as the animation's assembly steps
DIFF = (0.0, -20.0, 100.0)
EXPLODE = {
    "pinion": (0.0, -40.0, 0.0), "input axle": (0.0, -40.0, 0.0), "hub axle": (0.0, 0.0, 60.0),
    "slider": (-45.0, 0.0, 0.0),
    "motor frame": (0.0, 0.0, 70.0), "tt motor": (0.0, 0.0, 70.0), "hub motor": (0.0, 0.0, 70.0),
    **{k: DIFF for k in ("carrier", "ring gear", "cross pin", "spider top", "spider bottom", "side gear left",
                         "side gear right", "inner sleeve left", "inner sleeve right")},
}


def exploded() -> dict[str, bd.Part]:
    out = {}
    for k, s in drive_parts().items():
        off = EXPLODE.get(k)
        if off is None and k.rpartition(" ")[2] in ("left", "right") and not k.startswith(("side gear", "inner")):
            off = ((-1 if k.endswith("left") else 1) * 45.0, 0.0, 0.0)  # rear axle line pulled outboard
        m = bd.Pos(*(off or (0.0, 0.0, 0.0))) * s
        m.label, m.color = s.label, s.color
        out[k] = m
    return out


@step(out="../STEP/oldham_drive_assembly.step")
def oldham_drive_assembly():
    return bd.Compound(children=list(drive_parts().values()), label="oldham_drive")


@step(out="../STEP/oldham_drive_exploded.step")
def oldham_drive_exploded():
    return bd.Compound(children=list(exploded().values()), label="oldham_drive_exploded")


if __name__ == "__main__":
    oldham_drive_assembly()
    oldham_drive_exploded()
