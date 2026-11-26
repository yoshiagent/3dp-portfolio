"""Probe the MG90S frame and horn adapter STEPs: expected holes open, walls solid."""
from cadgen import read_step
from cadgen import build123d as bd

def probe(path, empty, solid):
    s = read_step(path).solids()[0]
    inside = lambda p: s.is_inside(bd.Vector(*p))
    bad = [("should be empty", p) for p in empty if inside(p)] + [("should be solid", p) for p in solid if not inside(p)]
    bb = s.bounding_box()
    print(f"{path}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}, valid {s.is_valid}, "
          f"{len(empty)} empty + {len(solid)} solid probes -> {'PASS' if not bad else 'FAIL'}")
    for b in bad[:12]:
        print("   ", b)
    return not bad

G = (-24, -16, -8, 0, 8, 16)
rows = (-3.9, -11.9, -19.9)
e, s = [], []
for x in G:
    for y in (-16, 16):
        e += [(x, y, z) for z in (-0.4, -12, -23.4)]                 # long-wall through holes
for x, ys in ((-24, (-8, 0, 8)), (16, (-8, 8))):
    for y in ys:
        e += [(x, y, z) for z in (-0.4, -12, -23.4)]                 # end-wall through holes
for x in (-16, -8, 0):
    e += [(x, 0, -23.4), (x, 0, -16)]                                # floor holes y = 0, one pin deep
for x in (-16, -8, 0, 8):
    for y in (-8, 8):
        e += [(x, y, -23.4), (x, y, -19.2)]                          # bottom holes y = +-8, to the pocket floor
        s += [(x, y, -18.3)]                                         # ... and solid above them
    s += [(x, 6.6, -17.5), (x, -6.6, -17.5)] if x < 8 else [(8, 8, -0.4), (6.6, 6.6, -5), (6.6, -6.6, -5)]
for x in G:
    for z in rows:
        for sy in (1, -1):
            e += [(x, sy * 19.3, z), (x, sy * 12, z)]                # long-face side holes
            s += [(x + 4, sy * 12, z)] if x < 16 else []               # wall between them
left = [(y, z) for z in rows for y in (-8, 0, 8) if not (y == 0 and z == -3.9)]
right = [(y, z) for z in rows for y in (-8, 0, 8) if not (y == 0 and z != -3.9)]
e += [(-27.3, y, z) for y, z in left] + [(-20, y, z) for y, z in left if y]
e += [(19.3, y, z) for y, z in right] + [(12, y, z) for y, z in right if y]
s += [(-27.3, 0, -3.9), (-26, 4, -8)]                                # skipped hole (left pilot), wall
e += [(-5.4, 0, -10), (-16.9, 5.9, -2), (6.1, -5.9, -18.6)]          # pocket, corners
s += [(-4, 0, -21), (-17.5, 0, -10), (6.7, 3, -9.4)]                  # floor, end walls past the pocket
e += [(12, 0, -20), (19.3, 0, -15), (7, 1.8, -22)]                   # cable slot: 8.6 wide, 9 above the floor
e += [(12, 4.0, -10.2), (7, -4.0, -18.5), (19.3, -4.0, -12), (6.0, 4.0, -10.2)]
s += [(12, 0, -9.4), (12, 4.6, -20), (16, 4.6, -12), (16, -4.6, -23.4), (8, 4.6, -21)]
e += [(-19.4, 0, -4), (8.6, 0, -4), (-19.4, 0.7, -7.5)]              # screw pilots
s += [(8.6, 0, -9.5), (-19.4, 0, -8.7)]  # under the pilots (left: above the y = 0 side hole)
ok1 = probe("out/mg90s_technic_frame.step", e, s)

c = 0.70710678
e = [(0, 0, 11.5), (2.3, 0, 8), (0, -2.3, 8), (2.3, 0, 3), (10 * c, 10 * c, 1), (-10 * c, 10 * c, 1),
     (8 * c, 8 * c, 6), (-8 * c, -8 * c, 11.8), (8, 0, 6), (0, -8, 0.2), (-8, 0, 11.9)]
s = [(1.6, 1.6, 8), (10 * c, 10 * c, 3), (0, 11.5, 6), (4.2, 4.2, 6), (12.3, 0, 1)]
ok2 = probe("out/mg90s_horn_adapter.step", e, s)
print("RESULT", "PASS" if ok1 and ok2 else "FAIL")
