"""Oldham coupling turning: hubs and slider every 15 deg over a turn; no overlaps with each other or with the motor
frame, drive base and pinion; each tongue stays fully inside its hub (the slot runs across the whole hub).
Run with cadgen's python from car_model/:  python tmp/check_oldham.py

Slider centre (x, z relative to the motor hub's axis, both shafts turned phi about +y as three.js makeRotationY):
the motor hub's slot direction u = (sin phi, cos phi), the axle hub's v = (cos phi, -sin phi), and the axle hub's axis
is 12 to the right, B - A = (12, 0). The centre P lies on both slots, so P - A = ((B - A) . u) u = 12 sin phi u:
x = -12 + 12 sin^2 phi, z = 12 sin phi cos phi (a 12 mm circle through both axes).
"""
import sys

sys.path.insert(0, "src")
from cadgen import build123d as bd  # noqa: E402

import car_parts as c  # noqa: E402

hm0, ha0 = c.oldham_hub_motor(), c.oldham_hub_axle()
fixed = {"motor frame": c.motor_place(c.to_vehicle(c.e.motor_frame())), "drive base": c.drive_base(),
         "pinion": c.to_vehicle(c.e.pinion())}
bad = []


def ov(a, b):
    if not a.bounding_box().overlaps(b.bounding_box(), 0.01):
        return 0.0
    r = a & b
    return 0.0 if r is None else r.volume


for phi in range(0, 360, 15):
    hm = c.spin_y(hm0, c.TT_SHAFT_X, phi)
    ha = c.spin_y(ha0, 0.0, phi)
    sl = c.oldham_slider(phi)
    for na, a, nb, b in (("hub motor", hm, "slider", sl), ("slider", sl, "hub axle", ha), ("hub motor", hm, "hub axle", ha)):
        v = ov(a, b)
        if v > 1e-3:
            bad.append((phi, na, nb, round(v, 3)))
    for nm, m in (("hub motor", hm), ("slider", sl), ("hub axle", ha)):
        for nf, f in fixed.items():
            v = ov(m, f)
            if v > 1e-3:
                bad.append((phi, nm, nf, round(v, 3)))
    # each tongue inside its hub's outer cylinder (front tongue in the motor hub, back tongue in the axle hub)
    for hx, y_lo, y_hi, name in ((c.TT_SHAFT_X, c.OD_Y0 - c.OD_S0, c.OD_Y0 - c.OD_S0 + c.OD_TONGUE_H, "front tongue"),
                                 (0.0, c.OD_Y0 - c.OD_B0 - c.OD_TONGUE_H, c.OD_Y0 - c.OD_B0 + 0.0, "back tongue")):
        h = y_hi - y_lo
        slab = bd.Pos(-6, (y_lo + y_hi) / 2, c.AXLE_Z) * bd.Box(90, h - 0.02, 90)
        band = bd.Pos(hx, (y_lo + y_hi) / 2, c.AXLE_Z) * bd.Rot(90, 0, 0) * bd.Cylinder(c.OD_R, h + 1)
        tongue = sl & slab
        out = tongue - band if tongue is not None else None
        if out is not None and out.volume > 1e-3:
            bad.append((phi, name, "outside its hub", round(out.volume, 3)))
print("Oldham 0..345 deg every 15:", bad or "no overlaps; both tongues always inside their hubs")
