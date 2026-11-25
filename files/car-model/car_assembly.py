"""Car model, full assembly: MG90S front steering + motor-driven differential + breadboard + two 18650 cells.

On the one printed chassis, everything goes on with vertical LEGO Technic 2L pins or along its own axle:
  drive train      TT motor in its 9L frame (pinned on the chassis), coupler, 3L axle, pinion, differential,
                   rear axle line, rear wheels (drive_parts)
  steering module  servo holder + steering frame + linkage + front wheels (steering_parts), pinned on the nose
  body             chassis, four posts, and on them the breadboard tray with the two-cell 18650 tray pinned back
                   to back under it (body_parts)
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
        "input axle": labeled(v(bd.Rot(0, 4 * spin, 0) * e.pinion_shaft()), "input axle 3L", "silver"),
        "coupler": labeled(v(bd.Rot(0, 4 * spin, 0) * e.coupler()), "TT coupler", "white"),
        "motor frame": labeled(c.motor_frame(), "TT motor frame 9L", "lightgray"),
        "tt motor": labeled(c.tt_motor(), "TT motor (not printed)", "yellow"),
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
        p[f"rear axle {side}"] = labeled(c.rear_axle(s), f"rear axle {c.AXLE_N[s]}L:{side}", "silver")
        p[f"collar {side}"] = labeled(c.collar(s), f"axle collar:{side}", "khaki")
        p[f"spacer {side}"] = labeled(c.spacer(s), f"wheel spacer:{side}", "khaki")
        p[f"rear wheel {side}"] = labeled(c.rear_rim(s), f"rear wheel LEGO 81.6:{side}", "white")
        p[f"rear tyre {side}"] = labeled(c.rear_tyre(s), f"rear tyre LEGO 81.6:{side}", TYRE)
        p[f"outer collar {side}"] = labeled(c.collar(s, outer=True), f"outer collar:{side}", "gold")
    return p


def drive_parts(spin: float = 0.0) -> dict[str, bd.Part]:
    """The drive train on the chassis: motor, coupler, differential, rear axle line."""
    return {**diff_parts(spin), **rear_parts()}


def body_parts() -> dict[str, bd.Part]:
    p = {"chassis": labeled(c.chassis(), "chassis", "#5a6470")}
    for s in (1, -1):
        side = SIDE[s]
        for r, where in enumerate(("front", "rear")):
            p[f"post {where} {side}"] = labeled(c.deck_post(c.POSTS_Y[r], s), f"deck post {where}:{side}", "gray")
    p.update(battery_parts())
    p["breadboard frame"] = labeled(c.breadboard_frame(), "breadboard frame", "steelblue")
    return p


def battery_parts() -> dict[str, bd.Part]:
    """The two-cell 18650 tray turned over under the breadboard tray, with its holder and cells."""
    p = {"battery tray": labeled(c.battery_tray(), "battery tray 2x18650", "slategray"),
         "battery holder": labeled(c.battery_holder(), "2x18650 holder (not printed)", "#202020")}
    for i, cell in enumerate(c.battery_cells()):
        p[f"battery cell {i + 1}"] = labeled(cell, f"18650 cell {i + 1} (not printed)", "#2f6fd6")
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


EXPLODE = {"chassis": -50.0, "drive": 0.0, "steering": 30.0, "posts": 60.0, "battery": 100.0, "top": 140.0}


def exploded() -> dict[str, bd.Part]:
    """Groups pulled apart along z: chassis down, steering module up, then posts, the battery tray and the
    breadboard tray further up, so the vertical pin joints between them can be seen."""
    def body_group(k):
        if k == "chassis":
            return "chassis"
        return "posts" if k.startswith("post") else "battery" if k.startswith("battery") else "top"

    groups = [(drive_parts(), lambda k: "drive"), (steering_parts(), lambda k: "steering"), (body_parts(), body_group)]
    out = {}
    for parts, group in groups:
        for k, s in parts.items():
            m = bd.Pos(0, 0, EXPLODE[group(k)]) * s
            m.label, m.color = s.label, s.color
            out[k] = m
    return out


@step(out="../STEP/car_assembly.step")
def car_assembly():
    return bd.Compound(children=list(posed_parts().values()), label="car_model")


def write_names(path="../STEP/car_assembly_names.json"):
    """Label -> bounding-box centre and size of every part, straight ahead: parts with the same shape (the four posts, the
    side gears, ...) share one product in the STEP, so their names there are not reliable; the animation export
    and freecad/build_assembly.py name each part from its position."""
    import json

    out = {}
    for sh in posed_parts().values():
        bb = sh.bounding_box()
        c, s = bb.center(), bb.size  # size too: a rim and its tyre share the centre
        out[sh.label] = [round(v, 3) for v in (c.X, c.Y, c.Z, s.X, s.Y, s.Z)]
    HERE.joinpath(path).write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    write_names()
    car_assembly()
