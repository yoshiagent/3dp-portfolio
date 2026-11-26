"""LEGO Technic tray for a widened breadboard: two half-size (400 point) breadboards side by side, one of them
without its outer power rail, so a wide module such as an ESP32 DevKit can straddle them (user's photo,
2026-11-26). Pocket 85.24 x 101 (width given by the user), 8.5 deep; same style as breadboard_technic.
Saves breadboard_wide_technic.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_breadboard_wide_technic.py

The boards sit in a pocket that is open at the top, their top face flush with the frame top, held by a close fit
and their adhesive backing; push them out from below through the floor holes.
Frame: 13 x 15 x 2 Technic units (103.4 x 119.4 x 15.8), origin at the centre, z up; x along the boards'
85 mm length, y across the 101 mm width. Long walls 9.2 thick, end walls 9.08.
Pin holes, all on the 8 mm grid, every one counterbored like a LEGO beam:
  - top: one ring of vertical holes through the walls (long walls nx each, end walls ny - 2)
  - bottom: vertical holes through the floor on every grid point inside that ring ((nx - 2) x (ny - 2)), so the
    bottom is a full 13 x 15 Technic grid
  - sides: two rows of horizontal holes (z = +-4) on all four sides. The upper row goes through the walls into
    the pocket (a one-pin-deep hole would leave a 0.8 mm skin), the lower row is one pin deep in the solid floor.
    The short sides skip the corner columns, where the long-side holes already are.
Each hole is built once and copied with a PartDesign LinearPattern whose count is bound to nx / ny.
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
    ("nx", "13", "長度方向格數：13 格，和半尺寸麵包板底座同長"),
    ("ny", "15", "寬度方向格數：15 格，槽寬 101 加兩邊牆"),
    ("frame_l", "=nx * pitch - 0.6", "框總長（自動計算）：13 格 103.4"),
    ("frame_w", "=ny * pitch - 0.6", "框總寬（自動計算）：15 格 119.4"),
    ("frame_h", "15.8", "框總高：2 格 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側面下排孔深度：一根銷的長度 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑：側孔外端、垂直孔上下兩端 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # two half-size breadboards side by side (one without its outer power rail)
    ("bb_l", "85", "麵包板長 (mm)"),
    ("bb_h", "8.5", "麵包板高 (mm)"),
    ("bb_clear", "0.12", "麵包板長度方向兩端各留間隙 (mm)"),
    ("pocket_l", "=bb_l + 2 * bb_clear", "槽長（自動計算）"),
    ("pocket_w", "101", "槽寬：兩片麵包板並排（其中一片拆掉外側電源條），使用者指定 (mm)"),
    ("pocket_z", "=frame_h / 2 - bb_h", "槽底 z：麵包板頂面與框頂齊平（自動計算）"),
    ("side_depth", "=(frame_w - pocket_w) / 2 + 0.5", "長邊上排側孔深：穿透牆到槽內（自動計算）"),
    ("end_depth", "=(frame_l - pocket_l) / 2 + 0.5", "短邊上排側孔深：穿透牆到槽內（自動計算）"),
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
    """Pin hole `depth` deep with counterbore, drilled from (x, y, z) 0.5 mm outside a face."""
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
    """Vertical pin hole with a counterbore at both ends (see breadboard_technic for the CB_OVERLAP reason)."""
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

    # grid centred on the frame: column i (0 .. nx - 1) at (i - (nx - 1) / 2) * pitch, row j likewise with ny
    gx = lambda i: f"({i} - ({P}nx - 1) / 2) * {P}pitch"   # noqa: E731
    gy = lambda j: f"({j} - ({P}ny - 1) / 2) * {P}pitch"   # noqa: E731
    nx, ny = P + "nx", P + "ny"

    def grid(name, feats, axis, count, axis2=None, count2=None):
        """Pattern with its counts bound to Params (count / count2 are expressions in nx, ny)."""
        n = lambda e: f"({e} - 1) * {P}pitch"  # noqa: E731
        pat = core.pattern(body, name, feats, axis, 2, n(count), axis2, 2 if axis2 else 1, n(count2) if axis2 else "0")
        bind(pat, "Occurrences", count)
        if axis2:
            bind(pat, "Occurrences2", count2)
        return pat

    # floor: through holes on every grid point inside the ring
    grid("FloorGrid", vertical_hole(body, "Floor", gx(1), gy(1), Z0, P + "pocket_z"),
         "X", f"{nx} - 2", "Y", f"{ny} - 2")
    # top: vertical holes through the walls in a ring; long walls nx each, end walls ny - 2
    for t, s in (("L", 1), ("R", -1)):
        grid(f"TopLong{t}", vertical_hole(body, f"Top{t}", gx(0), f"{s} * {gy(0)}", Z0, ZT), "X", nx)
    for t, s in (("F", 1), ("B", -1)):
        grid(f"TopEnd{t}", vertical_hole(body, f"Top{t}", f"{s} * {gx(0)}", gy(1), Z0, ZT), "Y", f"{ny} - 2")
    # two rows of side holes, half a pitch above and below the middle; upper row through into the pocket
    for row, rz in (("U", 1), ("D", -1)):
        z = f"{rz} * {P}pitch / 2"
        long_d = P + ("side_depth" if rz > 0 else "pin_depth")
        end_d = P + ("end_depth" if rz > 0 else "pin_depth")
        for t, s in (("L", 1), ("R", -1)):
            grid(f"Side{row}{t}", drilled_hole(body, f"Side{row}{t}h", (1, 0, 0), 90 * s, gx(0),
                                               f"{s} * ({P}frame_w / 2 + 0.5)", z, long_d), "X", nx)
        for t, s in (("F", 1), ("B", -1)):
            grid(f"End{row}{t}", drilled_hole(body, f"End{row}{t}h", (0, 1, 0), -90 * s,
                                              f"{s} * ({P}frame_l / 2 + 0.5)", gy(1), z, end_d), "Y", f"{ny} - 2")

    core.finish(doc, body, stem)


build("breadboard_wide_technic")
core.gui_resave("breadboard_wide_technic")
