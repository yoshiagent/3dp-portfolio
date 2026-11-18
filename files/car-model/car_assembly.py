"""Car model, full assembly: MG90S front steering + motor-driven differential + battery box + breadboard.

`posed_parts(theta, spin)` places every part: theta = steering crank angle (deg, + = counter-clockwise from above,
as in ../../mg90s_steering), spin = differential carrier turned about the rear axle (deg), for the checks.
Vehicle frame: x right, y forward, z up from the ground (see car_parts.py).
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "mg90s_steering" / "src"))

from cadgen import build123d as bd  # noqa: E402
from cadgen import step  # noqa: E402

import car_parts as c  # noqa: E402
from car_parts import d, e  # noqa: E402

SIDE = {1: "right", -1: "left"}


def labeled(shape: bd.Part, label: str, color: str) -> bd.Part:
    shape.label = label
    shape.color = bd.Color(color)
    return shape


def diff_parts(spin: float = 0.0) -> dict[str, bd.Part]:
    """Differential and drive train, in vehicle coordinates. Differential side +1 is the vehicle's left."""
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
        "input axle": labeled(v(e.pinion_shaft()), "input axle 3L", "silver"),
        "coupler": labeled(v(e.coupler()), "TT coupler", "white"),
        "motor frame": labeled(v(e.motor_frame()), "TT motor frame", "lightgray"),
        "tt motor": labeled(v(e.tt_motor()), "TT motor (not printed)", "yellow"),
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
        p[f"rear wheel {side}"] = labeled(c.rear_wheel(s), f"rear wheel 80:{side}",
                                          "royalblue" if s > 0 else "magenta")
    return p


def body_parts() -> dict[str, bd.Part]:
    return {
        "chassis": labeled(c.chassis(), "chassis", "dimgray"),
        "battery box": labeled(c.battery_box(), "battery box 3xAAA", "slategray"),
        "battery lid": labeled(c.battery_lid(), "battery lid", "lightslategray"),
        "post front": labeled(c.deck_post(c.POSTS_Y[0]), "deck post:front", "gray"),
        "post rear": labeled(c.deck_post(c.POSTS_Y[1]), "deck post:rear", "gray"),
        "deck": labeled(c.deck(), "deck", "darkgray"),
        "breadboard frame": labeled(c.breadboard_frame(), "breadboard frame", "steelblue"),
    }


def steering_parts(theta: float = 0.0) -> dict[str, bd.Part]:
    from steering_assembly import posed_parts as steer

    p = steer(theta)
    out = {}
    for k, shape in p.items():
        name = k.replace("wheel", "front wheel")
        if name.endswith(("left", "right")):
            base, _, side = name.rpartition(" ")
            shape.label = f"{shape.label.replace(' ' + side, '')}:{side}"
        out[name] = shape
    return out


def posed_parts(theta: float = 0.0, spin: float = 0.0) -> dict[str, bd.Part]:
    return {**body_parts(), **steering_parts(theta), **diff_parts(spin), **rear_parts()}


@step(out="../STEP/car_assembly.step")
def car_assembly():
    return bd.Compound(children=list(posed_parts().values()), label="car_model")


if __name__ == "__main__":
    car_assembly()
