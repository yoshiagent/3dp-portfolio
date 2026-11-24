"""Car model, full assembly: MG90S front steering + motor-driven differential + breadboard + two 18650 cells.

Three groups, each assembled on its own and then pinned together with vertical LEGO Technic 2L pins:
  drive module     drive base + motor, coupler, differential, rear axle line, rear wheels (drive_parts)
  steering module  servo holder + steering frame + linkage + front wheels (steering_parts)
  body             chassis (middle level), four posts, breadboard frame on top, and an 18650 tray hung on each
                   side of the breadboard frame by horizontal pins (body_parts)
`posed_parts(theta, spin)` places every part: theta = steering crank angle (deg, + = counter-clockwise from above,
as in ../../mg90s_steering), spin = differential carrier turned about the rear axle (deg), for the checks.
`exploded()` pulls the groups apart along z to show the pin joints.
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
TYRE = "#1c1c1c"


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
        p[f"rear wheel {side}"] = labeled(c.rear_rim(s), f"rear wheel LEGO 81.6:{side}", "white")
        p[f"rear tyre {side}"] = labeled(c.rear_tyre(s), f"rear tyre LEGO 81.6:{side}", TYRE)
        p[f"outer collar {side}"] = labeled(c.collar(s, outer=True), f"outer collar:{side}", "gold")
    return p


def drive_parts(spin: float = 0.0) -> dict[str, bd.Part]:
    """The drive module: everything that comes off the car with the drive base."""
    return {"drive base": labeled(c.drive_base(), "drive base", "dimgray"), **diff_parts(spin), **rear_parts()}


def body_parts() -> dict[str, bd.Part]:
    p = {"chassis": labeled(c.chassis(), "chassis", "#5a6470")}
    for s in (1, -1):
        side = SIDE[s]
        for r, where in enumerate(("front", "rear")):
            p[f"post {where} {side}"] = labeled(c.deck_post(c.POSTS_Y[r], c.POSTS_Z0[(s, r)], s),
                                                f"deck post {where}:{side}", "gray")
    p["breadboard frame"] = labeled(c.breadboard_frame(), "breadboard frame", "steelblue")
    p.update(battery_parts())
    return p


def battery_parts() -> dict[str, bd.Part]:
    """The two 18650 side trays with their holders and cells (they go on sideways, after the breadboard frame)."""
    p = {}
    for s in (1, -1):
        side = SIDE[s]
        p[f"battery tray {side}"] = labeled(c.battery_tray(s), f"battery tray 18650:{side}", "slategray")
        p[f"battery holder {side}"] = labeled(c.battery_holder(s), f"18650 holder (not printed):{side}", "#202020")
        p[f"battery cell {side}"] = labeled(c.battery_cell(s), f"18650 cell (not printed):{side}", "#2f6fd6")
    return p


def steering_parts(theta: float = 0.0) -> dict[str, bd.Part]:
    """The steering module as in ../../mg90s_steering (LEGO front wheels drawn there as rim + tyre)."""
    from steering_assembly import posed_parts as steer

    out = {}
    for k, shape in steer(theta).items():
        name = k.replace("wheel", "front wheel").replace("tyre", "front tyre")
        if name.endswith(("left", "right")):
            side = name.rpartition(" ")[2]
            label = shape.label.replace(" " + side, "")
            shape.label = f"{'front ' if k.startswith('tyre') else ''}{label}:{side}"
        out[name] = shape
    return out


def posed_parts(theta: float = 0.0, spin: float = 0.0) -> dict[str, bd.Part]:
    return {**body_parts(), **steering_parts(theta), **drive_parts(spin)}


EXPLODE = {"drive": -60.0, "chassis": 0.0, "steering": 50.0, "top": 110.0}  # z offset of each group
EXPLODE_SIDE = 40.0  # battery trays also pulled outward, along their horizontal pins


def exploded() -> dict[str, bd.Part]:
    """Groups pulled apart along z: drive module down, steering module up, posts/breadboard/batteries further up
    (the batteries also outward)."""
    groups = [(drive_parts(), lambda k: "drive"), (steering_parts(), lambda k: "steering"),
              (body_parts(), lambda k: "chassis" if k == "chassis" else "top")]
    out = {}
    for parts, group in groups:
        for k, s in parts.items():
            dx = 0.0
            if k.startswith("battery"):
                dx = EXPLODE_SIDE if k.endswith("right") else -EXPLODE_SIDE
            m = bd.Pos(dx, 0, EXPLODE[group(k)]) * s
            m.label, m.color = s.label, s.color
            out[k] = m
    return out


@step(out="../STEP/car_assembly.step")
def car_assembly():
    return bd.Compound(children=list(posed_parts().values()), label="car_model")


if __name__ == "__main__":
    car_assembly()
