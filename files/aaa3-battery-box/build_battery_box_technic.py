"""LEGO Technic container for a 3 x AAA battery box (measured 63 x 37 x 16), in the style of
breadboard_technic.FCStd, plus a flat Technic lid. Saves aaa3_battery_box_technic.FCStd and
aaa3_battery_lid_technic.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_battery_box_technic.py

Container: 10 x 7 x 3 Technic units (79.4 x 55.4 x 23.8), origin at the centre, z up. The battery box lies
in a pocket open at the top on a floor raised by `lift` (2 mm), so its top stands 1.6 mm above the frame top.
Pin holes, all on the 8 mm grid (x on half pitches, since the length is an even 10 units); every
vertical hole, in the container and the lid, has a counterbore at both ends like a LEGO beam:
  - top: one ring of vertical holes through the walls, top to bottom
  - bottom: vertical holes through the floor on every grid point inside that ring (8 x 5), so the whole
    bottom is a full 10 x 7 Technic grid
  - sides: three rows of horizontal holes (z = -8, 0, 8) on all four sides, crossing the vertical holes at
    right angles. The middle and upper rows go through the walls into the pocket, the lower row is blind in
    the solid floor. The end faces skip the corner column; the long-side holes in that column stay blind
    so they do not meet the end holes.
Repeated holes are built once and copied with PartDesign LinearPatterns, which keeps the feature
count low enough for FreeCAD to open the files quickly.
Lid: 7 x 7 x 1 units (55.4 x 55.4 x 7.8), a full 7 x 7 grid of through holes and 5 blind side holes with
counterbores in each of its four edges (corners skipped). It covers the container from
one end, its holes in line with the container's, and is pinned onto the ring holes with standard Technic
pins. The other 3 units of the container stay open for the switch and the wire.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

BOX_PARAMS = [
    # frame on the Technic grid
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("frame_l", "79.4", "框總長：10 格 (mm)"),
    ("frame_w", "55.4", "框總寬：7 格 (mm)"),
    ("frame_h", "23.8", "框總高：3 格 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "下排側孔深度：一根銷的長度 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：側孔外端、垂直孔上下兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # battery box, measured
    ("box_l", "63", "電池盒長：實測 (mm)"),
    ("box_w", "37", "電池盒寬：實測 (mm)"),
    ("box_h", "16", "電池盒高：實測 (mm)"),
    ("box_clear", "0.4", "電池盒四周各留間隙 (mm)"),
    ("top_gap", "0.4", "未上抬時電池盒頂面低於框頂 (mm)"),
    ("lift", "2", "底板往上加厚：電池盒上抬量，總高不變 (mm)"),
    ("pocket_l", "=box_l + 2 * box_clear", "槽長（自動計算）"),
    ("pocket_w", "=box_w + 2 * box_clear", "槽寬（自動計算）"),
    ("pocket_z", "=frame_h / 2 - top_gap - box_h + lift", "槽底 z（自動計算）"),
]

LID_PARAMS = [
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("lid_l", "55.4", "上蓋長：7 格 (mm)"),
    ("lid_w", "55.4", "上蓋寬：7 格 (mm)"),
    ("lid_h", "7.8", "上蓋厚：1 格 (mm)"),
    ("corner_r", "3.7", "上蓋四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側面盲孔深度：一根銷的長度 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：側孔外端、垂直孔上下兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
]

P = "Params."
CB_OVERLAP = 0.02  # mm, see vertical_hole()


def cylinder(body, name, cut, radius, height, x, y, z):
    cyl = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    bind(cyl, ".Placement.Base.x", x)
    bind(cyl, ".Placement.Base.y", y)
    bind(cyl, ".Placement.Base.z", z)
    return cyl


def drilled_hole(body, name, axis, angle, x, y, z, depth):
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


def rounded_block(body, length, width, height, z0=None):
    """Block centred on the origin in x and y, `height` (an expression) tall from z0 (default
    -height/2), vertical corners rounded."""
    x0, y0 = f"-{P}{length} / 2", f"-{P}{width} / 2"
    height = height if " " in height else P + height
    z0 = z0 or f"-({height}) / 2"
    box(body, "Block", False, P + length, P + width, height, x=x0, y=y0, z=z0)
    for tag, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)):
        cx = f"{sx} * ({P}{length} / 2 - {P}corner_r)"
        cy = f"{sy} * ({P}{width} / 2 - {P}corner_r)"
        box(body, f"Corner{tag}", True, f"{P}corner_r + 1", f"{P}corner_r + 1", f"{height} + 2",
            x=cx if sx > 0 else f"{cx} - {P}corner_r - 1", y=cy if sy > 0 else f"{cy} - {P}corner_r - 1",
            z=f"{z0} - 1")
        cylinder(body, f"Round{tag}", False, P + "corner_r", height, cx, cy, z0)


def build_box(stem):
    doc = core.make_doc(stem, BOX_PARAMS)
    body = doc.addObject("PartDesign::Body", "Frame")
    Z0, ZT = f"-{P}frame_h / 2", f"{P}frame_h / 2"
    rounded_block(body, "frame_l", "frame_w", "frame_h")

    # battery pocket, open at the top
    box(body, "Pocket", True, P + "pocket_l", P + "pocket_w", f"{ZT} - {P}pocket_z + 1",
        x=f"-{P}pocket_l / 2", y=f"-{P}pocket_w / 2", z=P + "pocket_z")

    # grid: x = k * pitch / 2 for odd k in -9..9 (10 columns), y = j * pitch for j in -3..3 (7 rows).
    # Each hole is made once at its first grid position and repeated with its own LinearPattern.
    gx = lambda k: f"{k} * {P}pitch / 2"   # noqa: E731
    gy = lambda j: f"{j} * {P}pitch"       # noqa: E731
    span = lambda n: f"{n} * {P}pitch"     # noqa: E731  # n pitches

    # floor: vertical holes through the floor inside the ring, 8 x 5
    core.pattern(body, "FloorGrid", vertical_hole(body, "Floor", gx(-7), gy(-2), Z0, P + "pocket_z"),
                 "X", 8, span(7), "Y", 5, span(4))
    # top: a ring of vertical holes through the walls; long walls 10 each, end walls 5 each
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"TopLong{t}", vertical_hole(body, f"Top{t}", gx(-9), gy(3 * s), Z0, ZT),
                     "X", 10, span(9))
    for t, s in (("F", 1), ("B", -1)):
        core.pattern(body, f"TopEnd{t}", vertical_hole(body, f"Top{t}", gx(9 * s), gy(-2), Z0, ZT),
                     "Y", 5, span(4))

    # sides: three rows (U/M/D at z = 8, 0, -8) on all four faces, crossing the vertical holes. The middle
    # and upper rows go through into the pocket, the lower row is blind in the solid floor; in the long
    # sides the corner columns stay one pin deep so they do not meet the end holes.
    long_through = f"({P}frame_w - {P}pocket_w) / 2 + 0.5"
    end_through = f"({P}frame_l - {P}pocket_l) / 2 + 0.5"
    for row, rz in (("U", 1), ("M", 0), ("D", -1)):
        z = f"{rz} * {P}pitch"
        for t, s in (("L", 1), ("R", -1)):
            y = f"{s} * ({P}frame_w / 2 + 0.5)"
            if rz >= 0:
                # 8 through holes between the corners, then the two blind corner holes
                core.pattern(body, f"Side{row}{t}", drilled_hole(body, f"Side{row}{t}h", (1, 0, 0), 90 * s,
                                                                 gx(-7), y, z, long_through), "X", 8, span(7))
                core.pattern(body, f"Corner{row}{t}", drilled_hole(body, f"Corner{row}{t}h", (1, 0, 0), 90 * s,
                                                                   gx(-9), y, z, P + "pin_depth"), "X", 2, span(9))
            else:
                core.pattern(body, f"Side{row}{t}", drilled_hole(body, f"Side{row}{t}h", (1, 0, 0), 90 * s,
                                                                 gx(-9), y, z, P + "pin_depth"), "X", 10, span(9))
        # ends: 5 holes per row (corner rows skipped)
        for t, s in (("F", 1), ("B", -1)):
            core.pattern(body, f"End{row}{t}", drilled_hole(
                body, f"End{row}{t}h", (0, 1, 0), -90 * s, f"{s} * ({P}frame_l / 2 + 0.5)", gy(-2), z,
                end_through if rz >= 0 else P + "pin_depth"), "Y", 5, span(4))

    core.finish(doc, body, stem)


def build_lid(stem):
    doc = core.make_doc(stem, LID_PARAMS)
    body = doc.addObject("PartDesign::Body", "Lid")
    rounded_block(body, "lid_l", "lid_w", "lid_h")
    span = lambda n: f"{n} * {P}pitch"     # noqa: E731
    # 7 x 7 vertical through holes, counterbored at both ends
    core.pattern(body, "PinGrid", vertical_hole(body, "Pin", span(-3), span(-3), f"-{P}lid_h / 2",
                                                f"{P}lid_h / 2"), "X", 7, span(6), "Y", 7, span(6))
    # 5 blind side holes in each edge, mid-height, crossing the outer ring of vertical holes;
    # the corner positions are skipped so holes from neighbouring edges never meet
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"Side{t}", drilled_hole(body, f"Side{t}h", (1, 0, 0), 90 * s, span(-2),
                                                    f"{s} * ({P}lid_w / 2 + 0.5)", "0", P + "pin_depth"),
                     "X", 5, span(4))
    for t, s in (("F", 1), ("B", -1)):
        core.pattern(body, f"End{t}", drilled_hole(body, f"End{t}h", (0, 1, 0), -90 * s,
                                                   f"{s} * ({P}lid_l / 2 + 0.5)", span(-2), "0", P + "pin_depth"),
                     "Y", 5, span(4))
    core.finish(doc, body, stem)


# BB_PARTS="lid" or "box" builds only that part
PARTS = {"lid": (build_lid, "aaa3_battery_lid_technic"), "box": (build_box, "aaa3_battery_box_technic")}
todo = [p for p in os.environ.get("BB_PARTS", "lid;box").split(";") if p]
for part in todo:
    PARTS[part][0](PARTS[part][1])
core.gui_resave(*(PARTS[p][1] for p in todo))
