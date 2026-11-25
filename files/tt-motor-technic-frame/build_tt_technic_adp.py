"""Rebuild ../tt-motor-technic_adp.3mf (5 x 10 Technic frame for a TT motor, CC0) with the motor held
the way tt_motor_adapter.FCStd holds it, saved as tt_motor_technic_adp.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_tt_technic_adp.py
Builds tt_motor_technic_Side.FCStd (3 rows of pin holes in each long side) and
tt_motor_technic_SideTop.FCStd (side holes, the frame's vertical holes and 3 x 3 holes in the front end
face); tt_motor_technic_Top.FCStd
has the vertical holes only.

Kept from the frame: the 79.4 x 39.4 x 23.8 outline, the 25 vertical pin holes, the motor pocket open at
the back. The opening under the shaft for double-shaft motors is closed by the floor. The output shaft stays at x = 12, y = 0,
on the Technic grid, but sits 1.7 mm lower than in the original so the lips end flush with the frame top.
Removed: the crush ribs, the two pegs and the open-top notch for the gearbox tab.
Added, from the adapter: a stop face with concave arcs that hug the gearbox's front corners, a closed hole
for the gearbox tab, a lip on each side wall and the floor spring tab under the back of the motor.
As in the adapter, the motor slides in from the open back under the lips until its tab enters the hole.
The lips sit on the solid side walls (no slits), so the frame top is one flat face.

Frame: origin at the frame centre, x toward the front (tab end), z up; the motor shaft points up.

Two trial variants of SideTop (2026-11-25), both with the motor moved toward the solid front end and the
second row of vertical holes behind the front end (the row next to the motor pocket) left out:
  tt_motor_technic_SideTop_9L     motor one pitch forward and the frame one pitch shorter at the open back:
                                  9 x 5 x 3, 71.4 long, shaft on a hole column (x = 16), stop face x = 27.5,
                                  front wall 8.2 so the front end holes are 7.6 deep
  tt_motor_technic_SideTop_fwd45  frame unchanged (10 x 5 x 3), stop face moved 4.5 forward to x = 28 (the
                                  centre line of the old second row), shaft at x = 16.5 (between hole columns)
In both the front end's middle vertical hole is a blind pair (top and bottom, one pin deep) and its middle
front hole is left out: the gearbox tab hole now lies under / behind them.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

PARAMS = [
    # frame, measured from tt-motor-technic_adp.3mf
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("frame_l", "79.4", "框總長：10 格 (mm)"),
    ("frame_w", "39.4", "框總寬：5 格 (mm)"),
    ("frame_h", "23.8", "框總高：3 層樑 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑（同原框）(mm)"),
    ("cb_d", "6.2", "側孔沉頭直徑：容納銷的凸環 (mm)"),
    ("cb_depth", "0.8", "側孔沉頭深度 (mm)"),
    ("top_depth", "8", "方孔上下兩個盲孔的深度：一根銷的長度 (mm)"),
    ("pocket_w", "22.76", "馬達槽內寬（同 adapter）(mm)"),
    ("stop_fx", "23.5", "馬達前端擋面 x：輸出軸在 x = 12 (mm)"),
    ("motor_t", "18.8", "TT 馬達厚度 (mm)"),
    ("motor_z", "=frame_h / 2 - lip_h - lip_gap - motor_t",
     "馬達底面 z（架橋頂面）：比原框低 1.7，讓卡扣唇與框頂平齊（自動計算）"),
    ("bridge1_x", "=stop_fx - 36.7", "後架橋起點 x：跟著擋面（原版 -13.2）(mm)"),
    ("bridge1_l", "10.2", "後架橋長 (mm)"),
    ("bridge2_x", "=stop_fx - 6.5", "前架橋起點 x（到擋面）：跟著擋面（原版 17）(mm)"),
    ("front_depth", "=(frame_w - pocket_w) / 2", "前端面孔深（預設同側牆厚）(mm)"),
    # motor retention, taken over from tt_motor_adapter.FCStd
    ("stop_w", "21.22", "adapter 擋面凸台寬，決定圓弧伸出長度 (mm)"),
    ("fillet_r", "3.65", "擋面兩側內角圓弧半徑 (mm)"),
    ("fillet_y", "7.41", "圓弧在擋面上的起點離中線距離 (mm)"),
    ("fillet_l", "=fillet_r - sqrt(fillet_r ^ 2 - (stop_w / 2 - fillet_y) ^ 2)", "圓弧往後伸出長度（自動計算）"),
    ("hole_w", "6.1", "前端方孔寬：卡齒輪箱凸片 (mm)"),
    ("hole_h", "3.8", "前端方孔高 (mm)"),
    ("hole_up", "7.45", "方孔底邊離馬達底面高度（同 adapter）(mm)"),
    ("hole_d", "5.1", "方孔深度（同原框凹槽深）(mm)"),
    ("post_from", "25.03", "卡扣唇起點離擋面距離（同 adapter）(mm)"),
    ("post_l", "10.8", "卡扣唇長 (mm)"),
    ("lip_d", "1.74", "卡扣唇往內伸出量 (mm)"),
    ("lip_gap", "0.1", "卡扣唇底面與馬達頂面間隙 (mm)"),
    ("lip_h", "1.9", "卡扣唇厚，唇頂與框頂平齊 (mm)"),
    # floor spring tab under the back of the motor, as in the adapter (lengths measured from the stop face)
    ("floor_back", "46.23", "後段底板從擋面往後的長度（同 adapter）(mm)"),
    ("floor_front_x", "=bridge2_x", "後段底板前緣 x：接到前架橋，底下封閉（自動計算）"),
    ("tslit_from", "20.23", "彈片切縫起點離擋面距離 (mm)"),
    ("tslit_y", "7.8", "彈片切縫內緣離中線距離 (mm)"),
    ("tslit_w", "0.4", "彈片切縫寬 (mm)"),
    ("tip_from", "39.83", "彈片末端薄區起點離擋面距離 (mm)"),
    ("tip_w", "13.04", "彈片末端薄區寬 (mm)"),
    ("tip_gap", "1.15", "彈片末端薄區頂面低於馬達底面的量（同 adapter）(mm)"),
    ("win_from", "41.73", "彈片方窗起點離擋面距離 (mm)"),
    ("win_l", "2.51", "彈片方窗長 (mm)"),
    ("win_w", "5.5", "彈片方窗寬 (mm)"),
    ("ramp_drop", "0.79", "彈片末端斜面下降量 (mm)"),
    ("ramp_len", "=floor_back - win_from - win_l", "彈片末端斜面長（自動計算）"),
    ("ramp_angle", "=atan(ramp_drop / ramp_len)", "彈片末端斜面角度（自動計算）"),
]

P = "Params."
X0, Y0, Z0, ZT = f"-{P}frame_l / 2", f"-{P}frame_w / 2", f"-{P}frame_h / 2", f"{P}frame_h / 2"


def cylinder(body, name, cut, radius, height, x, y, z):
    cyl = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    bind(cyl, ".Placement.Base.x", x)
    bind(cyl, ".Placement.Base.y", y)
    bind(cyl, ".Placement.Base.z", z)


def drilled_hole(body, name, axis, angle, x, y, z, depth=f"({P}frame_w - {P}pocket_w) / 2"):
    """Pin hole (depth = side wall thickness unless given) with counterbore, drilled from (x, y, z) just outside
    a face. Cylinders grow along +z; App.Rotation(axis, angle) tips that axis to point into the part."""
    for suffix, dia, depth in (("", "pin_d", f"{depth} + 0.5"),
                               ("Cb", "cb_d", f"{P}cb_depth + 0.5")):
        cyl = body.newObject("PartDesign::SubtractiveCylinder", f"{name}{suffix}")
        cyl.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(*axis), angle))
        bind(cyl, "Radius", f"{P}{dia} / 2")
        bind(cyl, "Height", depth)
        bind(cyl, ".Placement.Base.x", x)
        bind(cyl, ".Placement.Base.y", y)
        bind(cyl, ".Placement.Base.z", z)


def side_hole(body, name, x, z, s):
    """Hole from the side face (s = +1 for +y, -1 for -y) through the wall: it opens into the motor
    pocket, and is blind where the frame is solid (front end)."""
    drilled_hole(body, name, (1, 0, 0), 90 * s, x, f"{s} * ({P}frame_w / 2 + 0.5)", z)


def front_hole(body, name, y, z):
    """Blind hole from the front end face (+x), as deep as the side holes."""
    drilled_hole(body, name, (0, 1, 0), -90, f"{P}frame_l / 2 + 0.5", y, z, depth=f"{P}front_depth")


def build(stem, top_holes, side_holes, front_holes, units=10, stop_fx=None, front_depth=None, second_row=True):
    """units: frame length in pitches; stop_fx: motor stop face x (default 23.5, shaft 11.5 behind it);
    second_row False: no vertical holes in the row next to the motor pocket, the front end's middle vertical
    hole a blind pair and no middle front hole (the tab hole sits there)."""
    over = {"frame_l": f"{units * 8 - 0.6:g}"}
    if stop_fx is not None:
        over["stop_fx"] = f"{stop_fx:g}"
    if front_depth is not None:
        over["front_depth"] = f"{front_depth:g}"
    doc = core.make_doc(stem, [(n, over.get(n, v), c) for n, v, c in PARAMS])
    body = doc.addObject("PartDesign::Body", "Frame")

    # outline with rounded vertical corners: cut each corner square, add back a quarter-round cylinder
    box(body, "Block", False, P + "frame_l", P + "frame_w", P + "frame_h", x=X0, y=Y0, z=Z0)
    for tag, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)):
        cx = f"{sx} * ({P}frame_l / 2 - {P}corner_r)"
        cy = f"{sy} * ({P}frame_w / 2 - {P}corner_r)"
        box(body, f"Corner{tag}", True, f"{P}corner_r + 1", f"{P}corner_r + 1", f"{P}frame_h + 2",
            x=cx if sx > 0 else f"{cx} - {P}corner_r - 1", y=cy if sy > 0 else f"{cy} - {P}corner_r - 1",
            z=f"{Z0} - 1")
        cylinder(body, f"Round{tag}", False, P + "corner_r", P + "frame_h", cx, cy, Z0)

    # motor pocket, open at the back and the top; openings below it except under the two bridges
    box(body, "Pocket", True, f"{P}stop_fx + {P}frame_l / 2 + 1", P + "pocket_w", f"{ZT} - {P}motor_z + 1",
        x=f"{X0} - 1", y=f"-{P}pocket_w / 2", z=P + "motor_z")
    box(body, "OpenBack", True, f"{P}bridge1_x + {P}frame_l / 2 + 1", P + "pocket_w",
        f"{P}motor_z + {P}frame_h / 2 + 1", x=f"{X0} - 1", y=f"-{P}pocket_w / 2", z=f"{Z0} - 1")
    box(body, "OpenShaft", True, f"{P}bridge2_x - {P}bridge1_x - {P}bridge1_l", P + "pocket_w",
        f"{P}motor_z + {P}frame_h / 2 + 1", x=f"{P}bridge1_x + {P}bridge1_l", y=f"-{P}pocket_w / 2",
        z=f"{Z0} - 1")

    # stop face: concave arcs hugging the gearbox's rounded front corners
    for tag, s in (("L", 1), ("R", -1)):
        box(body, f"Fillet{tag}", False, P + "fillet_l", f"{P}pocket_w / 2 - {P}fillet_y",
            f"{ZT} - {P}motor_z", x=f"{P}stop_fx - {P}fillet_l",
            y=P + "fillet_y" if s > 0 else f"-{P}pocket_w / 2", z=P + "motor_z")
        cylinder(body, f"FilletArc{tag}", True, P + "fillet_r", f"{ZT} - {P}motor_z + 1",
                 f"{P}stop_fx - {P}fillet_r", f"{s} * {P}fillet_y", P + "motor_z")
    # closed hole for the gearbox tab
    box(body, "TabHole", True, f"{P}hole_d + 1", P + "hole_w", P + "hole_h",
        x=f"{P}stop_fx - 1", y=f"-{P}hole_w / 2", z=f"{P}motor_z + {P}hole_up")

    # x in half pitches (odd for a 10-unit length, even for 9: holes sit on the grid columns), y and z in pitches
    cols = [2 * i - (units - 1) for i in range(units)]
    f = units - 1  # front column
    if top_holes:
        # vertical pin holes along each long side, plus the front end's columns between them
        holes = [(k, s * 2) for s in (1, -1) for k in cols]
        if second_row:  # 3 across the front, 2 behind them; the 26th (middle behind) is a blind pair
            holes += [(f, -1), (f, 0), (f, 1), (f - 2, -1), (f - 2, 1)]
            mid = f - 2
        else:  # the row next to the pocket left out; the front middle is the blind pair
            holes += [(f, -1), (f, 1)]
            mid = f
        for i, (hx, hy) in enumerate(holes):
            cylinder(body, f"Pin{i + 1:02d}", True, f"{P}pin_d / 2", f"{P}frame_h + 2",
                     f"{hx} * {P}pitch / 2", f"{hy} * {P}pitch", f"{Z0} - 1")
        # the middle spot over the gearbox tab hole: blind holes, one pin deep, from the top and from the
        # bottom, so the tab hole stays closed
        cx = f"{mid} * {P}pitch / 2"
        cylinder(body, "PinMidTop", True, f"{P}pin_d / 2", f"{P}top_depth + 1", cx, "0",
                 f"{ZT} - {P}top_depth")
        cylinder(body, "PinMidBottom", True, f"{P}pin_d / 2", f"{P}top_depth + 1", cx, "0", f"{Z0} - 1")
    if side_holes:
        # 10 columns x 3 rows on each long side (the frame is 3 pitches high, rows centred on it)
        for tag, s in (("L", 1), ("R", -1)):
            for row in (-1, 0, 1):
                for k in cols:
                    side_hole(body, f"Side{tag}{row + 2}_{(k + units + 1) // 2:02d}",
                              f"{k} * {P}pitch / 2", f"{row} * {P}pitch", s)
    if front_holes:
        # 3 columns x 3 rows in the front end face; columns at y = +-16 would break out through the
        # rounded corners and cross the side holes, so they are left out
        for row in (-1, 0, 1):
            for col in (-1, 0, 1):
                if not second_row and row == 0 and col == 0:
                    continue  # would run into the gearbox tab hole
                front_hole(body, f"Front{row + 2}_{col + 3}", f"{col} * {P}pitch", f"{row} * {P}pitch")

    # floor under the back of the motor with the adapter's spring tab: two slits, a thinner free end,
    # a window and a ramp under the tip; the tab's free end points to the open back
    floor_end = f"{P}stop_fx - {P}floor_back"
    box(body, "BackFloor", False, f"{P}floor_front_x - ({floor_end})", P + "pocket_w",
        f"{P}motor_z - ({Z0})", x=floor_end, y=f"-{P}pocket_w / 2", z=Z0)
    for tag, s in (("L", 1), ("R", -1)):
        box(body, f"TabSlit{tag}", True, f"{P}floor_back - {P}tslit_from + 1", P + "tslit_w",
            f"{P}frame_h / 2", x=f"{floor_end} - 1",
            y=P + "tslit_y" if s > 0 else f"-{P}tslit_y - {P}tslit_w", z=f"{Z0} - 1")
    box(body, "TabThin", True, f"{P}floor_back - {P}tip_from + 1", P + "tip_w", f"{P}tip_gap + 1",
        x=f"{floor_end} - 1", y=f"-{P}tip_w / 2", z=f"{P}motor_z - {P}tip_gap")
    win_x = f"{P}stop_fx - {P}win_from - {P}win_l"
    box(body, "TabWindow", True, P + "win_l", P + "win_w", f"{P}frame_h / 2",
        x=win_x, y=f"-{P}win_w / 2", z=f"{Z0} - 1")
    # tip beyond the window slopes down toward the back: a box tilted about y whose bottom face runs
    # from (floor end, tip top - ramp_drop) up to (window, tip top)
    ramp = box(body, "TabRamp", True, f"{P}ramp_len / cos({P}ramp_angle) + 2", P + "win_w", "2",
               y=f"-{P}win_w / 2")
    ramp.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), -10))
    bind(ramp, ".Placement.Rotation.Angle", f"-{P}ramp_angle")
    bind(ramp, ".Placement.Base.x", f"{floor_end} - cos({P}ramp_angle)")
    bind(ramp, ".Placement.Base.z", f"{P}motor_z - {P}tip_gap - {P}ramp_drop - sin({P}ramp_angle)")

    # lips on the solid side walls (no slits), flush with the frame top; the motor slides in under them
    post_x = f"{P}stop_fx - {P}post_from - {P}post_l"
    for tag, s in (("L", 1), ("R", -1)):
        wall = f"{s} * {P}pocket_w / 2"
        box(body, f"Lip{tag}", False, P + "post_l", P + "lip_d", P + "lip_h",
            x=post_x, y=f"{wall} - {P}lip_d" if s > 0 else wall, z=f"{ZT} - {P}lip_h")

    core.finish(doc, body, stem)


# tt_motor_technic_Top.FCStd (top holes only) is the earlier output of build("...", True, False);
# it is not rebuilt here so the existing file stays as it is
# (stem, top holes, side holes, front end holes); env TT_VARIANTS="stem;stem" rebuilds only those
VARIANTS = [("tt_motor_technic_Side", False, True, False), ("tt_motor_technic_SideTop", True, True, True),
            ("tt_motor_technic_SideTop_9L", True, True, True, 9, 27.5, 7.6, False),
            ("tt_motor_technic_SideTop_fwd45", True, True, True, 10, 28.0, None, False)]
only = [s for s in os.environ.get("TT_VARIANTS", "").split(";") if s]
todo = [v for v in VARIANTS if not only or v[0] in only]
for variant in todo:
    build(*variant)
core.gui_resave(*(v[0] for v in todo))
