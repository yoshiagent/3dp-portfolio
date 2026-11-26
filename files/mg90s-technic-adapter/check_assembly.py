from itertools import combinations
from cadgen import build123d as bd, read_scene
from cadgen.geometry import closest_points, overlap_volume
scene = read_scene("STEP/mg90s_assembly.step")
b = {}
for leaf in scene.leaves():
    (sel,) = scene.resolve(leaf.ref)
    sol = sel.shape().solids()
    b[leaf.label] = sol[0] if len(sol) == 1 else bd.Compound(sol)
for (la, a), (lb, c) in combinations(b.items(), 2):
    g = closest_points(a, c).distance
    v = overlap_volume(a, c) if g < 1e-6 else 0.0
    print(f"{la:26s} {lb:26s} gap {g:6.3f} overlap {v:7.3f}")
fb = b["frame"].bounding_box(); ab = b["horn_adapter"].bounding_box()
print(f"adapter top z {ab.max.Z:.2f}, frame z {fb.min.Z:.2f}..{fb.max.Z:.2f}")
