"""2 x 2 x 2 LEGO Technic cube: pin holes through it in all three directions, counterbored at both ends.
Saves technic_cube_2x2.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_cube_technic.py

Cube 15.8 mm on each side (2 units of 8 mm less 0.2, like the height of the other Technic parts here), origin
at the centre. Each direction has 2 x 2 through holes on the 8 mm grid (+-4 mm), 12 holes in all; the holes
of the three directions cross inside the cube at the eight points (+-4, +-4, +-4). Every hole end has a
counterbore so a pin's collar sinks in and parts pinned on any face sit flush.
Each direction's hole is built once and copied with a 2 x 2 PartDesign LinearPattern.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

PARAMS = [
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("cube", "15.8", "方塊邊長：2 格減 0.2 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：每個孔兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
]

P = "Params."
# Counterbores go this much deeper than cb_depth. At exactly cb_depth, the edge of one face's counterbore
# (4 + cb_d / 2 = 7.1 from the centre) only touches the floor of the neighbouring face's counterbore
# (7.9 - 0.8 = 7.1), and that tangency leaves non-manifold edges in exported meshes.
CB_OVERLAP = 0.02

# hole direction -> (rotation tipping the cylinder's +z onto it, the two axes its 2 x 2 pattern spreads along)
DIRS = {
    "X": (App.Rotation(App.Vector(0, 1, 0), 90), ("Y", "Z")),
    "Y": (App.Rotation(App.Vector(1, 0, 0), -90), ("X", "Z")),
    "Z": (App.Rotation(), ("X", "Y")),
}


def cylinder(body, name, rot, radius, height, start):
    """Subtractive cylinder of `radius` and `height` (expressions), growing from `start` (x, y, z expressions)
    along the direction `rot` turns +z onto."""
    cyl = body.newObject("PartDesign::SubtractiveCylinder", name)
    cyl.Placement = App.Placement(App.Vector(), rot)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    for axis, expr in zip("xyz", start):
        bind(cyl, f".Placement.Base.{axis}", expr)
    return cyl


def through_hole(body, d, s1=-1, s2=-1, tag=""):
    """One pin hole along direction `d`, counterbored at both ends, at s1 * 4 and s2 * 4 mm on the other two
    axes (in x, y, z order). Returns its three features."""
    rot, _ = DIRS[d]
    off = iter(f"{s} * {P}pitch / 2" for s in (s1, s2))
    across = {a: next(off) for a in "XYZ" if a != d}
    out = f"-{P}cube / 2 - 1"                       # 1 mm outside the near face
    inner = f"{P}cube / 2 - {P}cb_depth - {CB_OVERLAP}"  # far counterbore floor
    def at(along):  # noqa: E306  # start point: `along` on the hole axis
        return tuple(along if a == d else across[a] for a in "XYZ")
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    return [cylinder(body, f"Hole{d}{tag}", rot, f"{P}pin_d / 2", f"{P}cube + 2", at(out)),
            cylinder(body, f"Hole{d}{tag}CbA", rot, f"{P}cb_d / 2", cb_h, at(out)),
            cylinder(body, f"Hole{d}{tag}CbB", rot, f"{P}cb_d / 2", cb_h, at(inner))]


def build(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "Cube")
    half = f"-{P}cube / 2"
    box(body, "Block", False, P + "cube", P + "cube", P + "cube", x=half, y=half, z=half)
    # vertical holes: one hole and a 2 x 2 LinearPattern
    core.pattern(body, "HolesZ", through_hole(body, "Z"), "X", 2, P + "pitch", "Y", 2, P + "pitch")
    # horizontal holes one by one: a LinearPattern of the tipped-over cylinders cut only slivers
    # (FreeCAD 1.1), so each of the 4 positions gets its own three features
    for d in ("X", "Y"):
        for i, (s1, s2) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1)), start=1):
            through_hole(body, d, s1, s2, tag=str(i))
    core.finish(doc, body, stem)


STEM = "technic_cube_2x2"
build(STEM)
core.gui_resave(STEM)
