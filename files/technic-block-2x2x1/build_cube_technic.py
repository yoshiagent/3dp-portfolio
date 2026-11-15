"""LEGO Technic blocks with pin holes in all three directions, counterbored at both ends:
  cube:  2 x 2 x 2 units, technic_cube_2x2.FCStd
  half:  2 x 2 x 1 units (the cube cut in half), technic_block_2x2x1.FCStd
Each saves its FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_cube_technic.py      (CUBE_PARTS="half" or "cube" builds only that part)

Sides are whole units of 8 mm less 0.2 (15.8 for 2 units, 7.8 for 1), like the height of the other Technic
parts here; origin at the centre. Holes sit on the 8 mm grid:
  cube: each direction 2 x 2 through holes at +-4 mm, 12 in all, crossing at the eight points (+-4, +-4, +-4)
  half: 2 x 2 vertical holes at x, y = +-4; 2 holes along X (y = +-4) and 2 along Y (x = +-4), at mid-height
        z = 0, 8 in all, crossing the vertical holes at (+-4, +-4, 0)
Every hole end has a counterbore so a pin's collar sinks in and parts pinned on any face sit flush.
The vertical holes are one hole and a 2 x 2 PartDesign LinearPattern; the horizontal ones are built one by one
(a LinearPattern of the tipped-over cylinders cut only slivers in FreeCAD 1.1).
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

COMMON = [
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：每個孔兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
]
PARTS = {
    "cube": {
        "stem": "technic_cube_2x2",
        "params": [("pitch", "8", "Technic 孔距 (mm)"), ("cube", "15.8", "方塊邊長：2 格減 0.2 (mm)")] + COMMON,
        "size": {"X": "cube", "Y": "cube", "Z": "cube"},   # spreadsheet alias of the side along each axis
        # horizontal holes: (direction, offsets on the other two axes in x, y, z order, in half pitches)
        "side_holes": [(d, s) for d in "XY" for s in ((-1, -1), (1, -1), (-1, 1), (1, 1))],
    },
    "half": {
        "stem": "technic_block_2x2x1",
        "params": [("pitch", "8", "Technic 孔距 (mm)"), ("size", "15.8", "長、寬：2 格減 0.2 (mm)"),
                   ("height", "7.8", "高：1 格減 0.2 (mm)")] + COMMON,
        "size": {"X": "size", "Y": "size", "Z": "height"},
        "side_holes": [("X", (-1, 0)), ("X", (1, 0)), ("Y", (-1, 0)), ("Y", (1, 0))],
    },
}

P = "Params."
# Counterbores go this much deeper than cb_depth. At exactly cb_depth, the edge of one face's counterbore
# (4 + cb_d / 2 = 7.1 from the centre, or 0 + 3.1 at mid-height) only touches the floor of the neighbouring
# face's counterbore, and that tangency leaves non-manifold edges in exported meshes.
CB_OVERLAP = 0.02

# hole direction -> rotation tipping the cylinder's +z onto it
ROT = {
    "X": App.Rotation(App.Vector(0, 1, 0), 90),
    "Y": App.Rotation(App.Vector(1, 0, 0), -90),
    "Z": App.Rotation(),
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


def through_hole(body, size, d, offsets=(-1, -1), tag=""):
    """One pin hole along direction `d` through the block, counterbored at both ends. `offsets` place it on the
    other two axes (x, y, z order) in half pitches (-1, 0, 1 -> -4, 0, 4 mm). Returns its three features."""
    off = iter(offsets)
    across = {a: f"{next(off)} * {P}pitch / 2" for a in "XYZ" if a != d}
    length = P + size[d]
    out = f"-{length} / 2 - 1"                        # 1 mm outside the near face
    inner = f"{length} / 2 - {P}cb_depth - {CB_OVERLAP}"  # far counterbore floor
    def at(along):  # noqa: E306  # start point: `along` on the hole axis
        return tuple(along if a == d else across[a] for a in "XYZ")
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    return [cylinder(body, f"Hole{d}{tag}", ROT[d], f"{P}pin_d / 2", f"{length} + 2", at(out)),
            cylinder(body, f"Hole{d}{tag}CbA", ROT[d], f"{P}cb_d / 2", cb_h, at(out)),
            cylinder(body, f"Hole{d}{tag}CbB", ROT[d], f"{P}cb_d / 2", cb_h, at(inner))]


def build(part):
    spec = PARTS[part]
    size = spec["size"]
    doc = core.make_doc(spec["stem"], spec["params"])
    body = doc.addObject("PartDesign::Body", "Cube" if part == "cube" else "HalfCube")
    sx, sy, sz = (P + size[a] for a in "XYZ")
    box(body, "Block", False, sx, sy, sz, x=f"-{sx} / 2", y=f"-{sy} / 2", z=f"-{sz} / 2")
    # vertical holes: one hole and a 2 x 2 LinearPattern
    core.pattern(body, "HolesZ", through_hole(body, size, "Z"), "X", 2, P + "pitch", "Y", 2, P + "pitch")
    # horizontal holes one by one
    count = {}
    for d, offsets in spec["side_holes"]:
        count[d] = count.get(d, 0) + 1
        through_hole(body, size, d, offsets, tag=str(count[d]))
    core.finish(doc, body, spec["stem"])
    return spec["stem"]


todo = [p for p in os.environ.get("CUBE_PARTS", "cube;half").split(";") if p]
core.gui_resave(*(build(p) for p in todo))
