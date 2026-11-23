"""LEGO Technic tray for a half-size (400 point) breadboard, 85 x 55.8 x 8.5, in the style of
tt_motor_technic_SideTop.FCStd. Saves breadboard_technic.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_breadboard_technic.py

The breadboard sits in a pocket that is open at the top, its top face flush with the frame top. It is held
by a close fit and its own adhesive backing; push it out from below through the floor holes.
Frame: 13 x 9 x 2 Technic units (103.4 x 71.4 x 15.8), origin at the centre, z up.
Pin holes, all on the 8 mm grid:
  - every vertical hole has a counterbore at both ends (top and bottom), like a LEGO beam
  - each hole is built once and copied with a PartDesign LinearPattern (keeps the file quick to open)
  - top: one ring of vertical holes through the walls, top to bottom
  - bottom: vertical holes through the floor on every grid point inside that ring (11 x 7), so the
    whole bottom is a full 13 x 9 Technic grid
  - sides: two rows of horizontal holes (z = +-4) on all four sides, one pin deep. They cross the vertical
    holes at right angles, like a LEGO Technic cross-hole block. The upper row goes through the walls
    into the pocket on all four sides (a fully inserted pin in a long side touches the breadboard by
    about 0.2 mm, accepted as a light clamp); the lower row is blind in the solid floor. The short sides skip the corner column, where the
    long-side holes already are.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

PARAMS = [
    # frame on the Technic grid
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("frame_l", "103.4", "框總長：13 格 (mm)"),
    ("frame_w", "71.4", "框總寬：9 格 (mm)"),
    ("frame_h", "15.8", "框總高：2 格 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側面孔深度：一根銷的長度（長邊上排穿透到槽內）(mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：側孔外端、垂直孔上下兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # breadboard, standard half size
    ("bb_l", "85", "麵包板長 (mm)"),
    ("bb_w", "55.8", "麵包板寬 (mm)"),
    ("bb_h", "8.5", "麵包板高 (mm)"),
    ("bb_clear", "0.12", "麵包板四周各留間隙 (mm)"),
    ("pocket_l", "=bb_l + 2 * bb_clear", "槽長（自動計算）"),
    ("pocket_w", "=bb_w + 2 * bb_clear", "槽寬（自動計算）"),
    ("pocket_z", "=frame_h / 2 - bb_h", "槽底 z：麵包板頂面與框頂齊平（自動計算）"),
]

P = "Params."
CB_OVERLAP = 0.02  # mm, see vertical_hole()
X0, Y0, Z0, ZT = f"-{P}frame_l / 2", f"-{P}frame_w / 2", f"-{P}frame_h / 2", f"{P}frame_h / 2"


def cylinder(body, name, cut, radius, height, x, y, z):
    cyl = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    bind(cyl, ".Placement.Base.x", x)
    bind(cyl, ".Placement.Base.y", y)
    bind(cyl, ".Placement.Base.z", z)
    return cyl


def drilled_hole(body, name, axis, angle, x, y, z, depth=P + "pin_depth"):
    """Pin hole `depth` deep with counterbore, drilled from (x, y, z) 0.5 mm outside a face.
    Cylinders grow along +z; App.Rotation(axis, angle) tips that axis to point into the part.
    Returns the two features, to be patterned."""
    feats = []
    for suffix, dia, d in (("", "pin_d", f"{depth} + 0.5"), ("Cb", "cb_d", f"{P}cb_depth + 0.5")):
        cyl = body.newObject("PartDesign::SubtractiveCylinder", f"{name}{suffix}")
        cyl.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(*axis), angle))
        bind(cyl, "Radius", f"{P}{dia} / 2")
        bind(cyl, "Height", d)
        bind(cyl, ".Placement.Base.x", x)
        bind(cyl, ".Placement.Base.y", y)
        bind(cyl, ".Placement.Base.z", z)
        feats.append(cyl)
    return feats


def vertical_hole(body, name, x, y, z_bot, z_top):
    """Vertical pin hole from z_bot to z_top with a counterbore at both ends, so a pin's collar sinks in
    and parts sit flush. Returns the three features, to be patterned.
    The counterbores go CB_OVERLAP deeper than cb_depth: at exactly cb_depth their floor only touches the
    side-hole counterbores below it (z = side row + cb_d / 2) along a line, and that tangency leaves
    non-manifold edges in exported meshes."""
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    return [cylinder(body, name, True, f"{P}pin_d / 2", f"({z_top}) - ({z_bot}) + 2", x, y, f"{z_bot} - 1"),
            cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x, y, f"{z_bot} - 1"),
            cylinder(body, f"{name}CbT", True, f"{P}cb_d / 2", cb_h, x, y,
                     f"{z_top} - {P}cb_depth - {CB_OVERLAP}")]


def build(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "Frame")

    # outline with rounded vertical corners
    box(body, "Block", False, P + "frame_l", P + "frame_w", P + "frame_h", x=X0, y=Y0, z=Z0)
    for tag, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)):
        cx = f"{sx} * ({P}frame_l / 2 - {P}corner_r)"
        cy = f"{sy} * ({P}frame_w / 2 - {P}corner_r)"
        box(body, f"Corner{tag}", True, f"{P}corner_r + 1", f"{P}corner_r + 1", f"{P}frame_h + 2",
            x=cx if sx > 0 else f"{cx} - {P}corner_r - 1", y=cy if sy > 0 else f"{cy} - {P}corner_r - 1",
            z=f"{Z0} - 1")
        cylinder(body, f"Round{tag}", False, P + "corner_r", P + "frame_h", cx, cy, Z0)

    # breadboard pocket, open at the top
    box(body, "Pocket", True, P + "pocket_l", P + "pocket_w", f"{P}bb_h + 1",
        x=f"-{P}pocket_l / 2", y=f"-{P}pocket_w / 2", z=P + "pocket_z")
    # grid positions in pitches: 13 columns along x (-6..6), 9 along y (-4..4); the walls are the outer ring.
    # Each hole is made once at its first grid position and repeated with its own LinearPattern.
    g = lambda n: f"{n} * {P}pitch"        # noqa: E731
    # floor: vertical holes through the floor on every grid point inside the ring (11 x 7)
    core.pattern(body, "FloorGrid", vertical_hole(body, "Floor", g(-5), g(-3), Z0, P + "pocket_z"),
                 "X", 11, g(10), "Y", 7, g(6))
    # top: vertical holes through the walls, in a ring around the pocket; long walls 13, end walls 7
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"TopLong{t}", vertical_hole(body, f"Top{t}", g(-6), g(4 * s), Z0, ZT),
                     "X", 13, g(12))
    for t, s in (("F", 1), ("B", -1)):
        core.pattern(body, f"TopEnd{t}", vertical_hole(body, f"Top{t}", g(6 * s), g(-3), Z0, ZT),
                     "Y", 7, g(6))
    # two rows of side holes, half a pitch above and below the middle
    for row, rz in (("U", 1), ("D", -1)):
        z = f"{rz} * {P}pitch / 2"
        # long sides: 13 holes each, along y
        for t, s in (("L", 1), ("R", -1)):
            core.pattern(body, f"Side{row}{t}", drilled_hole(body, f"Side{row}{t}h", (1, 0, 0), 90 * s, g(-6),
                                                             f"{s} * ({P}frame_w / 2 + 0.5)", z), "X", 13, g(12))
        # short ends: 7 holes each (corner columns skipped), along x; the upper row goes through the
        # end wall into the pocket (a blind hole would leave too thin a skin), the lower row is blind
        end_depth = f"({P}frame_l - {P}pocket_l) / 2 + 0.5" if rz > 0 else P + "pin_depth"
        for t, s in (("F", 1), ("B", -1)):
            core.pattern(body, f"End{row}{t}", drilled_hole(body, f"End{row}{t}h", (0, 1, 0), -90 * s,
                                                            f"{s} * ({P}frame_l / 2 + 0.5)", g(-3), z, end_depth),
                         "Y", 7, g(6))

    core.finish(doc, body, stem)


build("breadboard_technic")
core.gui_resave("breadboard_technic")
