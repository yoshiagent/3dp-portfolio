"""LEGO Technic frame for a 3 x AAA battery box (measured 63 x 37 x 16, with switch and wire lead),
in the style of tt_motor_technic_SideTop.FCStd. Saves aaa3_battery_technic.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_battery_technic.py

The box lies flat in a pocket that is open at the top and at the back (wire end), on a 3 mm floor like
the TT motor in SideTop. It slides in from the back under two lips that run from just above the box up
to the frame top, until it meets the closed front end. A spring tab in the floor carries a bump with a
ramp: the box rides over it on the way in, the tab springs up behind the box and stops it sliding out.
To remove: press the bump down from the open back (or the tab from below) and push the box out through
the round hole in the floor at the front end.
The lips cover only the front part of the box, so the switch near the wire end stays free.
Pin holes: 3 rows in each long side, 3 rows in the front end, and vertical holes through the walls and
the front block. Frame: origin at the centre, x toward the closed front, z up.
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
    ("frame_l", "79.4", "框總長：10 格 (mm)"),
    ("frame_w", "55.4", "框總寬：7 格 (mm)"),
    ("frame_h", "23.8", "框總高：3 格 (mm)"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "前端盲孔深度：一根銷的長度（側孔穿透側牆）(mm)"),
    ("cb_d", "6.2", "側面 / 前端孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "側面 / 前端孔沉頭深度 (mm)"),
    # battery box, measured
    ("box_l", "63", "電池盒長：實測 (mm)"),
    ("box_w", "37", "電池盒寬：實測 (mm)"),
    ("box_h", "16", "電池盒高：實測 (mm)"),
    ("side_clear", "0.4", "電池盒兩側各留間隙 (mm)"),
    ("len_clear", "0.8", "電池盒前後總間隙（前端牆到彈片凸塊）(mm)"),
    ("pocket_w", "=box_w + 2 * side_clear", "電池槽寬（自動計算）"),
    ("front_t", "10", "前端封閉牆厚 (mm)"),
    ("pocket_fx", "=frame_l / 2 - front_t", "電池槽前端 x（自動計算）"),
    # box as low as the TT motor in tt_motor_technic_SideTop: same 3 mm floor
    ("floor_t", "3", "底板厚：電池盒底面與 TT 馬達座的馬達底面同高 (mm)"),
    ("floor_z", "=-frame_h / 2 + floor_t", "電池槽底面 z（自動計算）"),
    # lips holding the top of the box, from just above the box up to the frame top
    ("lip_d", "2.5", "卡扣唇往內伸出量 (mm)"),
    ("lip_gap", "0.3", "卡扣唇底面與電池盒頂面間隙 (mm)"),
    ("lip_l", "40", "卡扣唇長：從前端牆往後，後段留給開關 (mm)"),
    ("lip_z", "=floor_z + box_h + lip_gap", "卡扣唇底面 z，唇一路到框頂（自動計算）"),
    # spring tab in the floor with a bump behind the box
    ("tab_w", "12", "底板彈片寬 (mm)"),
    ("tab_slit", "0.6", "彈片兩側切縫寬 (mm)"),
    ("tab_len", "30", "彈片長：從後端往前 (mm)"),
    ("tab_t", "2", "彈片厚 (mm)"),
    ("bump_h", "1.2", "彈片凸塊高出槽底量 (mm)"),
    ("bump_top", "1", "凸塊頂部平段長 (mm)"),
    ("ramp_l", "4", "凸塊斜面長：電池盒推入時由此壓下彈片 (mm)"),
    ("bump_x", "=pocket_fx - box_l - len_clear", "凸塊擋面 x：電池盒後端面位置（自動計算）"),
    ("ramp_angle", "=atan(bump_h / ramp_l)", "凸塊斜面角度（自動計算）"),
    # round finger hole in the floor to push the box out, at the front end, far from the tab slits
    ("win_d", "12", "推出圓孔直徑 (mm)"),
    ("win_from", "4", "推出圓孔邊緣離前端牆距離 (mm)"),
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


def drilled_hole(body, name, axis, angle, x, y, z, depth):
    """Pin hole `depth` deep with counterbore, drilled from (x, y, z) 0.5 mm outside a face.
    Cylinders grow along +z; App.Rotation(axis, angle) tips that axis to point into the part."""
    for suffix, dia, depth in (("", "pin_d", f"{depth} + 0.5"), ("Cb", "cb_d", f"{P}cb_depth + 0.5")):
        cyl = body.newObject("PartDesign::SubtractiveCylinder", f"{name}{suffix}")
        cyl.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(*axis), angle))
        bind(cyl, "Radius", f"{P}{dia} / 2")
        bind(cyl, "Height", depth)
        bind(cyl, ".Placement.Base.x", x)
        bind(cyl, ".Placement.Base.y", y)
        bind(cyl, ".Placement.Base.z", z)


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

    # battery pocket, open at the back and the top
    box(body, "Pocket", True, f"{P}pocket_fx + {P}frame_l / 2 + 1", P + "pocket_w", f"{ZT} - {P}floor_z + 1",
        x=f"{X0} - 1", y=f"-{P}pocket_w / 2", z=P + "floor_z")
    # lips over the front part of the box, flush with the frame top
    for tag, s in (("L", 1), ("R", -1)):
        wall = f"{s} * {P}pocket_w / 2"
        box(body, f"Lip{tag}", False, P + "lip_l", P + "lip_d", f"{ZT} - {P}lip_z",
            x=f"{P}pocket_fx - {P}lip_l", y=f"{wall} - {P}lip_d" if s > 0 else wall, z=P + "lip_z")

    # round finger hole through the floor at the front end, to push the box out toward the back
    cylinder(body, "PushHole", True, f"{P}win_d / 2", f"{P}floor_z - ({Z0}) + 2",
             f"{P}pocket_fx - {P}win_from - {P}win_d / 2", "0", f"{Z0} - 1")

    # spring tab: recess from below leaves a thin strip, two slits free its sides, root toward the front
    box(body, "TabRecess", True, f"{P}tab_len + 1", f"{P}tab_w + 2 * {P}tab_slit",
        f"{P}floor_z - {P}tab_t - ({Z0}) + 1", x=f"{X0} - 1", y=f"-{P}tab_w / 2 - {P}tab_slit", z=f"{Z0} - 1")
    for tag, s in (("L", 1), ("R", -1)):
        box(body, f"TabSlit{tag}", True, f"{P}tab_len + 1", P + "tab_slit", f"{P}tab_t + 2",
            x=f"{X0} - 1", y=f"{P}tab_w / 2" if s > 0 else f"-{P}tab_w / 2 - {P}tab_slit",
            z=f"{P}floor_z - {P}tab_t - 1")
    # bump on the tab: square face toward the box, ramp toward the open back
    box(body, "Bump", False, f"{P}bump_top + {P}ramp_l", P + "tab_w", P + "bump_h",
        x=f"{P}bump_x - {P}bump_top - {P}ramp_l", y=f"-{P}tab_w / 2", z=P + "floor_z")
    # box tilted about y whose bottom face runs from (ramp start, floor) up to (flat top start, bump top)
    ramp = box(body, "BumpRamp", True, f"{P}ramp_l / cos({P}ramp_angle)", f"{P}tab_w + 2", "3",
               y=f"-{P}tab_w / 2 - 1")
    ramp.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), -10))
    bind(ramp, ".Placement.Rotation.Angle", f"-{P}ramp_angle")
    bind(ramp, ".Placement.Base.x", f"{P}bump_x - {P}bump_top - {P}ramp_l")
    bind(ramp, ".Placement.Base.z", P + "floor_z")

    # pin holes; x in half pitches (odd: between grid lines of the 10-unit length), y and z in pitches
    # vertical holes through each side wall and across the front block
    vertical = [(k, s * 3) for s in (1, -1) for k in range(-9, 10, 2)]
    vertical += [(9, j) for j in (-2, -1, 0, 1, 2)]
    for i, (hx, hy) in enumerate(vertical):
        cylinder(body, f"Pin{i + 1:02d}", True, f"{P}pin_d / 2", f"{P}frame_h + 2",
                 f"{hx} * {P}pitch / 2", f"{hy} * {P}pitch", f"{Z0} - 1")
    # 10 columns x 3 rows in each long side, through the wall into the pocket (as in SideTop;
    # the wall is too thin for a blind hole with a skin); blind in the solid front block
    wall_t = f"({P}frame_w - {P}pocket_w) / 2"
    for tag, s in (("L", 1), ("R", -1)):
        for row in (-1, 0, 1):
            for k in range(-9, 10, 2):
                drilled_hole(body, f"Side{tag}{row + 2}_{(k + 11) // 2:02d}", (1, 0, 0), 90 * s,
                             f"{k} * {P}pitch / 2", f"{s} * ({P}frame_w / 2 + 0.5)", f"{row} * {P}pitch",
                             wall_t)
    # 5 columns x 3 rows in the front end face, blind
    for row in (-1, 0, 1):
        for col in (-2, -1, 0, 1, 2):
            drilled_hole(body, f"Front{row + 2}_{col + 3}", (0, 1, 0), -90,
                         f"{P}frame_l / 2 + 0.5", f"{col} * {P}pitch", f"{row} * {P}pitch", P + "pin_depth")

    core.finish(doc, body, stem)


build("aaa3_battery_technic")
core.gui_resave("aaa3_battery_technic")
