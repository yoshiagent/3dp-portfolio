"""Assembly of the universal-joint teaching module, posed for any motor position, motor angle and yoke phase.

parts(x, theta, phase, hand): x = motor shaft position (one of uj_parts.POSITIONS), theta = motor angle (deg, about
+y), phase = the J2-end yoke of the intermediate axle turned 0 (in phase: constant velocity) or 90 deg (ripple
doubled), hand = crank instead of motor + coupler. The joints are solved with vectors (as the MechanicalAI
universal joint page): each spider's arms are the two yokes' journal directions, at right angles.
Run with cadgen's python from ujoint_diff/:  python src/uj_assembly.py   (writes the STEP files)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cadgen import build123d as bd  # noqa: E402
from cadgen import step  # noqa: E402

import uj_parts as u  # noqa: E402
from uj_parts import d  # noqa: E402

SIDE = {1: "right", -1: "left"}


# --- vectors ---
def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mul(a, s):
    return tuple(x * s for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(dot(a, a))
    return mul(a, 1 / n)


def rot(v, k, deg):
    t = math.radians(deg)
    kv, kd = cross(k, v), dot(k, v)
    return tuple(v[i] * math.cos(t) + kv[i] * math.sin(t) + k[i] * kd * (1 - math.cos(t)) for i in range(3))


def angle_about(v, axis, ref):
    return math.degrees(math.atan2(dot(v, cross(axis, ref)), dot(v, ref)))


Y = (0.0, 1.0, 0.0)
X0 = (1.0, 0.0, 0.0)


def solve(x: float, theta: float, phase: float = 0.0) -> dict:
    """Journal directions and shaft angles. a1 = a3 = +y (motor and pinion axes), a2 = J1 -> J2."""
    j1, j2 = u.j1(x), u.J2
    a2 = norm(sub(j2, j1))
    u1 = rot(X0, Y, theta)  # motor-side yoke journals
    v2 = norm(cross(a2, u1))  # intermediate yoke at J1
    p = rot(v2, a2, phase)  # intermediate yoke at J2
    q = norm(cross(Y, p))  # input-side yoke
    v20 = norm(cross(a2, X0))
    q0 = norm(cross(Y, rot(v20, a2, phase)))
    return {"j1": j1, "j2": j2, "a2": a2, "u1": u1, "v2": v2, "p": p, "q": q,
            "mid": angle_about(v2, a2, v20), "out": angle_about(q, Y, q0)}


def out_angle(x, theta, phase=0.0):
    return solve(x, theta, phase)["out"]


def labeled(shape, label, color):
    shape.label = label
    shape.color = bd.Color(color)
    return shape


def at(shape, origin, x_dir, z_dir):
    return u.frame_at(origin, x_dir, z_dir) * shape


def spin_y(shape, cx, deg):
    """Turn about the line along y through (cx, 0)."""
    return bd.Pos(cx, 0, 0) * bd.Rot(0, deg, 0) * bd.Pos(-cx, 0, 0) * shape


class _D:
    """Cached differential model parts (the gears take a while to build)."""

    def __getattr__(self, name):
        fn = getattr(d, name)
        cache = {}

        def wrapped(*args):
            if args not in cache:
                cache[args] = fn(*args)
            return cache[args]

        object.__setattr__(self, name, wrapped)
        return wrapped


D = _D()
CARRIER_SIGN = -1.0  # carrier turns CARRIER_SIGN * out / 3 about +x (checked: ring and pinion stay meshed)


def parts(x: float = 0.0, theta: float = 0.0, phase: float = 0.0, hand: bool = False) -> dict:
    s = solve(x, theta, phase)
    j1, j2, a2, u1, v2, p, q = s["j1"], s["j2"], s["a2"], s["u1"], s["v2"], s["p"], s["q"]
    out = {}
    out["base"] = labeled(u.base(), "base", "dimgray")
    out["bearing tower"] = labeled(u.bearing_tower_at(x), "bearing tower", "slategray")
    if hand:
        out["crank"] = labeled(u.crank(x, theta), "hand crank", "orange")
    else:
        out["motor frame"] = labeled(u.motor_frame(x), "TT motor frame", "lightgray")
        out["tt motor"] = labeled(u.tt_motor(x), "TT motor (not printed)", "yellow")
        out["coupler"] = labeled(u.coupler(x, theta), "TT coupler", "white")
    out["axle A1"] = labeled(at(u.lego_axle(u.lego_len(u.A1_N)), (x, u.A1_Y0, 0), u1, Y), "axle A1 4L", "silver")
    out["dial motor"] = labeled(at(u.dial(), (x, u.DIAL_M_Y[1], 0), u1, mul(Y, -1)), "dial (motor)", "deepskyblue")
    out["yoke motor"] = labeled(at(u.yoke(), (x, u.YOKE_M_TAIL, 0), u1, Y), "yoke (motor)", "tomato")
    out["spider J1"] = labeled(at(u.spider(), j1, u1, cross(u1, v2)), "spider J1", "gold")
    t1 = add(j1, mul(a2, u.Z_C))
    out["yoke mid J1"] = labeled(at(u.yoke(), t1, v2, mul(a2, -1)), "yoke (intermediate, J1 end)", "mediumpurple")
    n_mid, e_each = u.mid_axle(x)
    out["axle mid"] = labeled(at(u.lego_axle(u.lego_len(n_mid)), add(j1, mul(a2, u.Z_C - e_each)), v2, a2),
                              f"intermediate axle {n_mid}L", "silver")
    mid = mul(add(j1, j2), 0.5)
    out["dial mid"] = labeled(at(u.dial(), add(mid, mul(a2, -u.DIAL_T / 2)), v2, a2), "dial (intermediate)",
                              "violet")
    t2 = add(j2, mul(a2, -u.Z_C))
    out["yoke mid J2"] = labeled(at(u.yoke(), t2, p, a2), "yoke (intermediate, J2 end)", "mediumpurple")
    out["spider J2"] = labeled(at(u.spider(), j2, p, cross(p, q)), "spider J2", "gold")
    out["yoke input"] = labeled(at(u.yoke(), (u.APEX_X, u.YOKE_IN_TAIL, 0), q, mul(Y, -1)), "yoke (input)", "tomato")
    out["dial input"] = labeled(at(u.dial(), (u.APEX_X, u.DIAL_IN_Y[0], 0), q, Y), "dial (input)", "deepskyblue")
    out["axle input"] = labeled(at(u.lego_axle(u.lego_len(u.INPUT_N)), (u.APEX_X, u.INPUT_Y0, 0), q, Y),
                                "input axle 4L", "silver")
    # journals: LEGO 2L axles from each ear's outer face into the spider
    # (each journal's cross lines up with the cross hole of the ear it is pushed into: its yoke's axis)
    for jn, jc, dirs in (("J1", j1, ((u1, Y), (v2, a2))), ("J2", j2, ((p, a2), (q, Y)))):
        for k, (dv, yoke_axis) in enumerate(dirs):
            for sgn in (1, -1):
                dd = mul(dv, sgn)
                shape = at(u.lego_axle(u.JOURNAL_L), add(jc, mul(dd, u.EAR_OUT)), yoke_axis, mul(dd, -1))
                out[f"journal {jn} {k}{'+' if sgn > 0 else '-'}"] = labeled(
                    shape, f"journal 2L:{jn}{'ab'[k]}{'+' if sgn > 0 else '-'}", "silver")
    # differential: pinion with the input; carrier group 1/3 of it
    out_ang = s["out"]
    car = CARRIER_SIGN * out_ang / 3

    def carried(shape):
        return bd.Rot(car, 0, 0) * shape

    out["pinion"] = labeled(spin_y(D.pinion(), u.APEX_X, out_ang), "pinion 11T", "red")
    out["carrier"] = labeled(carried(D.carrier_left()), "carrier", "limegreen")
    out["ring gear"] = labeled(carried(D.ring_half()), "ring gear 33T", "orange")
    out["cross pin"] = labeled(carried(D.cross_pin()), "cross pin", "white")
    out["spider top"] = labeled(carried(D.spider(1)), "spider gear:top", "deepskyblue")
    out["spider bottom"] = labeled(carried(D.spider(-1)), "spider gear:bottom", "deepskyblue")
    for sd in (1, -1):
        side = SIDE[sd]
        out[f"side gear {side}"] = labeled(carried(D.side_gear(sd)), f"side gear:{side}", "gold")
        out[f"sleeve {side}"] = labeled(D.sleeve(sd), f"sleeve:{side}", "khaki")
        out[f"axle {side}"] = labeled(carried(D.axle(sd)), f"axle 4L:{side}", "silver")
        out[f"wheel {side}"] = labeled(carried(D.wheel(sd)), f"wheel:{side}", "#3b82f6" if sd < 0 else "#fb923c")
    return out


def exploded() -> dict:
    off = {"motor frame": (0, -40, 0), "tt motor": (0, -40, 0), "coupler": (0, -22, 0), "axle A1": (0, -14, 0),
           "bearing tower": (0, 0, 40), "dial motor": (0, 0, 45), "yoke motor": (0, 0, 60),
           "spider J1": (0, 0, 75), "yoke mid J1": (0, 0, 60), "axle mid": (0, 0, 90), "dial mid": (0, 0, 105),
           "yoke mid J2": (0, 0, 60), "spider J2": (0, 0, 75), "yoke input": (0, 0, 60), "dial input": (0, 0, 45),
           "axle input": (0, 0, 30)}
    res = {}
    for k, sh in parts().items():
        o = off.get(k)
        if o is None and k.startswith("journal"):
            o = (0, 0, 75)
        m = bd.Pos(*(o or (0, 0, 0))) * sh
        m.label, m.color = sh.label, sh.color
        res[k] = m
    return res


@step(out="../STEP/ujoint_diff_assembly.step")
def ujoint_diff_assembly():
    return bd.Compound(children=list(parts().values()), label="ujoint_diff")


@step(out="../STEP/ujoint_diff_exploded.step")
def ujoint_diff_exploded():
    return bd.Compound(children=list(exploded().values()), label="ujoint_diff_exploded")


@step(out="../STEP/ujoint_diff_x24.step")
def ujoint_diff_x24():
    """Motor at its last position (x = -24, joint angle 40.3 deg), turned 30 deg."""
    return bd.Compound(children=list(parts(-24.0, 30.0).values()), label="ujoint_diff_x24")


def write_names(path="../STEP/ujoint_diff_assembly_names.json"):
    """Label -> bounding-box centre of every part at rest: parts with the same shape share one product in the STEP,
    so FreeCAD would give them one name; freecad/build_assembly.py renames each from its position."""
    import json

    out = {}
    for sh in parts().values():
        c = sh.bounding_box().center()
        out[sh.label] = [round(c.X, 3), round(c.Y, 3), round(c.Z, 3)]
    Path(__file__).resolve().parent.joinpath(path).write_text(json.dumps(out, indent=1), encoding="utf-8")


if __name__ == "__main__":
    write_names()
    ujoint_diff_assembly()
    ujoint_diff_exploded()
    ujoint_diff_x24()
