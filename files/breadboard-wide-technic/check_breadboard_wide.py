"""Probe breadboard_wide_technic (13 x 15 x 2, pocket 85.24 x 101 x 8.5). Run with cadgen python from tt_motor_adapter/."""
from cadgen import build123d as bd
from cadgen import read_step

part = read_step("out/breadboard_wide_technic.step")
bb = part.bounding_box()
print("bbox", round(bb.size.X, 2), round(bb.size.Y, 2), round(bb.size.Z, 2), "volume", round(part.volume, 1),
      "valid", part.is_valid, "solids", len(part.solids()))
H, L, W, D = 15.8, 85.24, 101.0, 8.5
PZ = H / 2 - D  # pocket floor
boards = bd.Pos(0, 0, PZ + D / 2) * bd.Box(L, W, D)
inter = part & boards
print("boards overlap", 0 if inter is None else round(inter.volume, 4))
inside = lambda p: part.is_inside(bd.Vector(*p))  # noqa: E731
empty, solid, bad = [], [], []
want = lambda p, e: ((empty if e else solid) if inside(p) != e else bad).append(p)  # noqa: E731
# pocket edges and floor
for p in [(L / 2 - 0.05, 2, 4), (-L / 2 + 0.05, 2, 4), (4, W / 2 - 0.05, 2), (4, -W / 2 + 0.05, 2), (4, 4, PZ + 0.05)]:
    want(p, True)
for p in [(L / 2 + 0.05, 4, 2), (4, W / 2 + 0.05, 2), (4, 4, PZ - 0.05), (4, 4, -H / 2 + 0.05)]:
    want(p, False)
cols = [i * 8 for i in range(-6, 7)]
rows = [j * 8 for j in range(-7, 8)]
ring = [(x, s * 56) for x in cols for s in (1, -1)] + [(s * 48, y) for s in (1, -1) for y in rows[1:-1]]
for x, y in ring:
    for z in (-H / 2 + 0.3, 0, H / 2 - 0.3):
        want((x, y, z), True)
floor = [(x, y) for x in cols[1:-1] for y in rows[1:-1]]
for x, y in floor:
    for z in (-H / 2 + 0.3, PZ - 0.3):
        want((x, y, z), True)
# side holes: upper row (z 4) through to the pocket, lower row (z -4) one pin deep, blind
for x in cols:
    for s in (1, -1):
        for t in (59.4, 55.0, 51.0):
            want((x, s * t, 4), True)
            want((x, s * (t - 0.0), -4), True) if t > 51.3 else None
        want((x, s * 50.9, -4), False)  # past the one-pin-deep end, before the floor holes
for y in rows[1:-1]:
    for s in (1, -1):
        for t in (51.4, 47.0, 43.0):
            want((s * t, y, 4), True)
        want((s * 43.0, y, -4), False)
print(f"probes: {len(empty)} empty OK, {len(solid)} solid OK, bad {bad[:10] or 'none'} ({len(bad)})")
print(f"vertical holes: ring {len(ring)}, floor {len(floor)}, total {len(ring) + len(floor)}")
