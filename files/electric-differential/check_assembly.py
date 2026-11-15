"""Fit and motion checks for STEP/electric_assembly.step (run from the project root)."""
from __future__ import annotations

import sys
from itertools import combinations
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cadgen import build123d as bd
from cadgen import read_scene
from cadgen.geometry import closest_points, overlap_volume

import electric_parts as e

# touching on purpose: thrust faces, seats, parts sitting on the base, the axle bottoming in the pinion
CONTACT_OK = {frozenset(p) for p in [
    ("side_gear:right", "ring_half"), ("side_gear:left", "carrier_left"),
    ("spider:top", "carrier_left"), ("spider:bottom", "carrier_left"),
    ("carrier_left", "ring_half"), ("pinion", "frame"), ("pinion", "pinion_shaft"),
    ("motor_frame", "frame"), ("battery_box", "frame"), ("battery_lid", "battery_box"),
    ("breadboard_frame", "frame"), ("tt_motor (not printed)", "motor_frame"),
    ("tt_motor (not printed)", "coupler"),
]}
MIN_GAP = 0.1
CLOSE_FITS = {frozenset(("carrier_left", "cross_pin")): 0.03, frozenset(("ring_half", "pinion")): 0.05,
              frozenset(("pinion_shaft", "coupler")): 0.05} | {
    frozenset((s, g)): 0.05 for s in ("spider:top", "spider:bottom") for g in ("side_gear:right", "side_gear:left")
} | {frozenset((ax, part)): 0.05 for side in ("right", "left")
     for ax, part in ((f"axle:{side}", f"side_gear:{side}"), (f"axle:{side}", f"wheel:{side}"))}

scene = read_scene("STEP/electric_assembly.step")
bodies = {}
for leaf in scene.leaves():
    (sel,) = scene.resolve(leaf.ref)
    solids = sel.shape().solids()
    bodies[leaf.label] = solids[0] if len(solids) == 1 else bd.Compound(solids)
    if len(solids) != 1:
        print(f"note: {leaf.label} has {len(solids)} solids")

bb = bd.Compound(list(bodies.values())).bounding_box()
print(f"overall size: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
fb = bodies["frame"].bounding_box()
print(f"base print size: {fb.size.X:.1f} x {fb.size.Y:.1f} x {fb.size.Z:.1f} mm")

print("\n== 1. static pose: overlap and gap ==")
failures = []
for (la, a), (lb, b) in combinations(bodies.items(), 2):
    ba, bb2 = a.bounding_box(optimal=False), b.bounding_box(optimal=False)
    if any(tuple(ba.min)[i] > tuple(bb2.max)[i] + 1 or tuple(bb2.min)[i] > tuple(ba.max)[i] + 1 for i in range(3)):
        continue
    gap = closest_points(a, b).distance
    vol = overlap_volume(a, b) if gap < 1e-6 else 0.0
    contact = frozenset((la, lb)) in CONTACT_OK
    ok = vol <= 0.001 and (contact or gap >= CLOSE_FITS.get(frozenset((la, lb)), MIN_GAP))
    if not ok:
        failures.append((la, lb, round(vol, 3), round(gap, 3)))
    print(f"  {'ok  ' if ok else 'FAIL'} {la:22s} {lb:22s} overlap {vol:7.3f} gap {gap:6.3f}{'  (seat)' if contact else ''}",
          flush=True)


def about(shape, angle, axis_dir, point=(0, 0, 0)):
    p = bd.Vector(*point)
    return bd.Pos(p) * bd.Rotation(*(v * angle for v in axis_dir)) * bd.Pos(-p) * shape


def clip(shape, center, size=16.0):
    box = (bd.Pos(*center) * bd.Box(size, size, size)).solids()[0]
    out = shape & box
    return out.solids()[0] if out and len(out.solids()) == 1 else bd.Compound(out.solids()) if out else None


print("\n== 2. ring/pinion mesh sweep (motor turns, carrier follows) ==")
steps, pitch = 12, 360 / e.RING_TEETH
zone = (e.PINION.r * 0.85, -e.RING.r * 0.85, 0)
worst_vol, worst_gap = 0.0, 99.0
for k in range(steps):
    a = pitch * k / steps
    ring = clip(about(bodies["ring_half"], a, (1, 0, 0)), zone)
    # mesh point (x > 0, y < 0): ring about +X moves it toward -z, so the pinion turns positively about +Y
    pin = clip(about(bodies["pinion"], a * e.RING_TEETH / e.PINION_TEETH, (0, 1, 0)), zone)
    gap = closest_points(ring, pin).distance
    vol = sum(overlap_volume(x, y) for x in ring.solids() for y in pin.solids()) if gap < 1e-6 else 0.0
    worst_vol = max(worst_vol, vol)
    worst_gap = min(worst_gap, gap) if vol == 0 else worst_gap
    print(f"    step {k + 1}/{steps}: overlap {vol:.4f}, gap {gap:.3f}", flush=True)
print(f"  ring/pinion: max overlap {worst_vol:.4f} mm^3, min gap {worst_gap:.3f} mm")

print("\n== 3. turning carrier vs everything that stands still ==")
moving = ["ring_half", "carrier_left", "spider:top", "spider:bottom", "cross_pin"]
still = ["frame", "pinion", "pinion_shaft", "sleeve:right", "sleeve:left", "motor_frame", "coupler",
         "breadboard_frame", "battery_box"]
sweep_fail = []
for k in range(24):
    a = 360 / 24 * k
    worst = 99.0
    for m in moving:
        pm = about(bodies[m], a, (1, 0, 0))
        for s in still:
            if m == "ring_half" and s == "pinion":
                continue  # meshing, checked in 2
            g = closest_points(pm, bodies[s]).distance
            if g < MIN_GAP:
                sweep_fail.append((a, m, s, round(g, 3)))
            worst = min(worst, g)
    print(f"    {a:5.1f} deg: min gap {worst:.3f}", flush=True)
print(f"  carrier sweep: {'PASS' if not sweep_fail else sweep_fail}")

print("\nRESULT:", "PASS" if not failures and worst_vol == 0 and not sweep_fail
      else f"FAIL static={failures} mesh={worst_vol} sweep={sweep_fail[:5]}")
