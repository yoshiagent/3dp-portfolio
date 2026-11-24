"""Probe battery18650x2_technic (13 x 9 x 3, pocket 76 x 42.6 on a 7.8 floor) and its back-to-back fit with
breadboard_technic (13 x 9 x 2). Run with cadgen python from tt_motor_adapter/."""
from cadgen import build123d as bd
from cadgen import read_step

part = read_step("out/battery18650x2_technic.step")
bb = part.bounding_box()
print("bbox", round(bb.size.X, 2), round(bb.size.Y, 2), round(bb.size.Z, 2), "volume", round(part.volume, 1),
      "valid", part.is_valid)
H = 23.6
ZB, PZ = -H / 2, -H / 2 + 7.8  # bottom face, pocket floor
L, W = 76.0, 42.6
holder = bd.Pos(0, 0, PZ + 9) * bd.Box(L, W, 18)
inter = part & holder
print("holder overlap", 0 if inter is None else round(inter.volume, 4), " holder top above frame top",
      round(PZ + 18 - H / 2, 2))
inside = lambda p: part.is_inside(bd.Vector(*p))  # noqa: E731
empty, solid, bad = [], [], []


def want_empty(p):
    (bad if inside(p) else empty).append(p)


def want_solid(p):
    (solid if inside(p) else bad).append(p)


for p in [(L / 2 - 0.05, 2, 0), (-L / 2 + 0.05, 2, 0), (2, W / 2 - 0.05, 0), (2, -W / 2 + 0.05, 0), (2, 2, PZ + 0.05)]:
    want_empty(p)
for p in [(L / 2 + 0.05, 2, 0), (2, W / 2 + 0.05, 0), (4, 4, PZ - 0.05), (4, 4, ZB + 0.05)]:
    want_solid(p)  # (4, 4): floor between holes
cols = [i * 8 for i in range(-6, 7)]
rows = [j * 8 for j in range(-4, 5)]
ring = [(x, s * 32) for x in cols for s in (1, -1)] + [(s * 48, y) for s in (1, -1) for y in rows[1:-1]]
for x, y in ring:
    for z in (ZB + 0.3, -8, 0, 8, H / 2 - 0.3):
        want_empty((x, y, z))
floor = [(x, y) for x in cols[2:-2] for y in (-16, -8, 0, 8, 16)]
for x, y in floor:
    for z in (ZB + 0.3, ZB + 4, PZ - 0.3):
        want_empty((x, y, z))
for x in cols[2:-2]:
    for s in (1, -1):
        want_solid((x, s * 24, ZB + 4))  # rows +-24 left out
print(f"probes: {len(empty)} empty OK, {len(solid)} solid OK, bad {bad or 'none'}")

# back to back: the breadboard tray as modelled (z -7.9 .. 7.9), the battery tray turned over below it, bottom
# faces 0.2 apart; every grid point where both bottoms have a hole takes a 2L pin
bbt = read_step("out/breadboard_technic.step")
gap = 0.2
tray = bd.Pos(0, 0, -7.9 - gap - H / 2) * bd.Rot(180, 0, 0) * part
zi = -7.9 - gap / 2
pins, blocked = [], []
for x in cols:
    for y in rows:
        up = all(not bbt.is_inside(bd.Vector(x, y, zi + t)) for t in (0.3, 2.0, 4.0, 6.0))
        down = all(not tray.is_inside(bd.Vector(x, y, zi - t)) for t in (0.3, 2.0, 4.0, 6.0))
        (pins if up and down else blocked).append((x, y))
want = set(ring) | set(floor)
print(f"back-to-back pin positions: {len(pins)} (expected {len(want)}), match {set(pins) == want}")
ov = bbt & tray
print("breadboard / battery tray overlap", 0 if ov is None else round(ov.volume, 4))
bd.export_step(bd.Compound(children=[bbt, tray]), "tmp/battery18650x2_back_to_back.step")
