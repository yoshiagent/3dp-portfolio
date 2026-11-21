"""LEGO Technic tray for a single 18650 battery holder (78 x 22 x 18), in the style of breadboard_technic.FCStd.
Saves battery18650_technic.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_18650_technic.py

The holder sits in a pocket that is open at the top, on a thin floor; the frame is 2 Technic units high like the
breadboard tray, so the holder stands about 4.4 mm proud of the frame top. No wire slot: the leads come out over
the open top. The pocket is exactly 78 x 22 (no clearance, as asked), so the holder is a snug fit; push it out from below
through the floor holes.
Frame: 13 x 5 x 2 Technic units (103.4 x 39.4 x 15.8), as long as the breadboard tray; origin at the centre, x along
the holder, z up. The long walls are 8.7 and the end walls 12.7 thick. Hole positions follow nx / ny in Params (centred on the grid), and the
pattern counts are bound to them, so a different length or width only needs those two numbers.
Pin holes, all on the 8 mm grid:
  - top: one ring of vertical holes through the walls, top to bottom, counterbored at both ends
  - floor: through holes on every grid point inside that ring (11 x 3), so the whole bottom is a 13 x 5 grid; the
    floor is too thin for a counterbore, they are plain holes (for pushing the holder out, or a pin's tip). The
    column next to each end wall (x = +-40) lies partly under the wall: there the hole is a 2.2 mm recess
  - sides: two rows of horizontal holes (z = +-4) on all four sides, through the 8.3 mm walls into the pocket;
    the short ends skip the corner rows, where the long-side holes already are
Each hole is built once and copied with a PartDesign LinearPattern.
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
    ("nx", "13", "長邊格數：13 格和麵包板底座同長"),
    ("ny", "5", "短邊格數"),
    ("frame_l", "=nx * pitch - 0.6", "框總長（自動計算）：13 格 103.4"),
    ("frame_w", "=ny * pitch - 0.6", "框總寬（自動計算）：5 格 39.4"),
    ("frame_h", "15.8", "框總高：2 格，和麵包板底座同高 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：側孔外端、垂直孔上下兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # 18650 holder
    ("hd_l", "78", "18650 電池盒長 (mm)"),
    ("hd_w", "22", "18650 電池盒寬 (mm)"),
    ("hd_h", "18", "18650 電池盒高 (mm)"),
    ("hd_clear", "0", "電池盒四周各留間隙：0，槽剛好 78 × 22；太緊時調大 (mm)"),
    ("pocket_l", "=hd_l + 2 * hd_clear", "槽長（自動計算）"),
    ("pocket_w", "=hd_w + 2 * hd_clear", "槽寬（自動計算）"),
    ("floor_t", "2.2", "底板厚：電池盒頂高出框頂約 4.4 mm (mm)"),
    ("pocket_z", "=-frame_h / 2 + floor_t", "槽底 z（自動計算）"),
    ("side_depth", "=(frame_w - pocket_w) / 2 + 0.5", "長邊側孔深：穿透牆到槽內（自動計算）"),
    ("end_depth", "=(frame_l - pocket_l) / 2 + 0.5", "短邊側孔深：穿透牆到槽內（自動計算）"),
]

P = "Params."
CB_OVERLAP = 0.02  # mm: counterbores reach a hair past cb_depth, see breadboard_technic vertical_hole()
X0, Y0, Z0, ZT = f"-{P}frame_l / 2", f"-{P}frame_w / 2", f"-{P}frame_h / 2", f"{P}frame_h / 2"


def cylinder(body, name, cut, radius, height, x, y, z):
    cyl = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    bind(cyl, ".Placement.Base.x", x)
    bind(cyl, ".Placement.Base.y", y)
    bind(cyl, ".Placement.Base.z", z)
    return cyl


def drilled_hole(body, name, axis, angle, x, y, z, depth):
    """Pin hole `depth` deep with counterbore, drilled from (x, y, z) 0.5 mm outside a face (as in breadboard)."""
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


def vertical_hole(body, name, x, y, z_bot, z_top, counterbore=True):
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    feats = [cylinder(body, name, True, f"{P}pin_d / 2", f"({z_top}) - ({z_bot}) + 2", x, y, f"{z_bot} - 1")]
    if counterbore:
        feats += [cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x, y, f"{z_bot} - 1"),
                  cylinder(body, f"{name}CbT", True, f"{P}cb_d / 2", cb_h, x, y,
                           f"{z_top} - {P}cb_depth - {CB_OVERLAP}")]
    return feats


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

    # holder pocket, open at the top
    box(body, "Pocket", True, P + "pocket_l", P + "pocket_w", f"{P}frame_h",
        x=f"-{P}pocket_l / 2", y=f"-{P}pocket_w / 2", z=P + "pocket_z")

    # grid centred on the frame: column i (0 .. nx - 1) at (i - (nx - 1) / 2) * pitch, row j likewise with ny.
    # Each hole is made at its first position and repeated by a LinearPattern whose count is bound to nx / ny.
    gx = lambda i: f"({i} - ({P}nx - 1) / 2) * {P}pitch"   # noqa: E731
    gy = lambda j: f"({j} - ({P}ny - 1) / 2) * {P}pitch"   # noqa: E731

    def grid(name, feats, axis, count, axis2=None, count2=None):
        """Pattern with its counts bound to Params (count / count2 are expressions in nx, ny)."""
        n = lambda e: f"({e} - 1) * {P}pitch"  # noqa: E731
        pat = core.pattern(body, name, feats, axis, 2, n(count), axis2, 2 if axis2 else 1, n(count2) if axis2 else "0")
        bind(pat, "Occurrences", count)
        if axis2:
            bind(pat, "Occurrences2", count2)
        return pat

    nx, ny = P + "nx", P + "ny"
    # floor: plain through holes inside the ring ((nx - 2) x (ny - 2))
    grid("FloorGrid", vertical_hole(body, "Floor", gx(1), gy(1), Z0, P + "pocket_z", False),
         "X", f"{nx} - 2", "Y", f"{ny} - 2")
    # top: vertical holes through the walls in a ring; long walls nx each, end walls ny - 2 (corners are in the long walls)
    for t, s in (("L", 1), ("R", -1)):
        grid(f"TopLong{t}", vertical_hole(body, f"Top{t}", gx(0), f"{s} * {gy(0)}", Z0, ZT), "X", nx)
    for t, s in (("F", 1), ("B", -1)):
        grid(f"TopEnd{t}", vertical_hole(body, f"Top{t}", f"{s} * {gx(0)}", gy(1), Z0, ZT), "Y", f"{ny} - 2")
    # two rows of side holes, half a pitch above and below the middle, through the walls into the pocket
    for row, rz in (("U", 1), ("D", -1)):
        z = f"{rz} * {P}pitch / 2"
        for t, s in (("L", 1), ("R", -1)):
            grid(f"Side{row}{t}", drilled_hole(body, f"Side{row}{t}h", (1, 0, 0), 90 * s, gx(0),
                                               f"{s} * ({P}frame_w / 2 + 0.5)", z, P + "side_depth"), "X", nx)
        for t, s in (("F", 1), ("B", -1)):
            grid(f"End{row}{t}", drilled_hole(body, f"End{row}{t}h", (0, 1, 0), -90 * s,
                                              f"{s} * ({P}frame_l / 2 + 0.5)", gy(1), z, P + "end_depth"),
                 "Y", f"{ny} - 2")

    core.finish(doc, body, stem)


build("battery18650_technic")
core.gui_resave("battery18650_technic")
