"""Build the TT motor adapter with a LEGO Technic interface, plus a TT shaft to Technic axle coupler.

Run:  freecadcmd.exe build_tt_technic.py
Outputs tt_motor_technic.FCStd (cradle + Technic side walls) and tt_technic_coupler.FCStd,
each with STEP + STL in out/.

The cradle from tt_adapter_core.py gets thick side walls with Technic pin holes on both sides.
The hole grid (8 mm pitch) is anchored to the motor's output shaft: the shaft sits on the
centre line (y = 0) and on a hole column (x = shaft_x), so Technic parts attached to the sides
line up with the shaft. The TT shaft points up (z); the coupler slides onto it.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

TECHNIC_PARAMS = [
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("shaft_x", "16.8", "TT 馬達輸出軸中心 x：依 TT 規格估計，請實測馬達後修正 (mm)"),
    ("axial_gap", "0.2", "沿銷軸方向與相鄰 Technic 零件的間隙 (mm)"),
    ("edge_gap", "0.4", "Technic 樑寬 7.2，外緣比格線內縮量 (mm)"),
    ("outer_w", "=5 * pitch - 2 * axial_gap", "總寬：5 格（自動計算）"),
    ("back_x", "=shaft_x - 2.5 * pitch + edge_gap", "後端面 x（自動計算）"),
    ("front_x", "=shaft_x + 4.5 * pitch - edge_gap", "前端面 x：共 7 格（自動計算）"),
    ("tower_x", "=shaft_x + 1.5 * pitch - edge_gap", "側邊高牆終點 x：卡扣柱前留空隙（自動計算）"),
    ("tower_h", "=2 * pitch - edge_gap", "側邊高牆高度：2 排孔（自動計算）"),
    ("pin_d", "4.9", "銷孔直徑：LEGO 4.8，FDM 略放大 (mm)"),
    ("pin_depth", "8", "銷孔深度（盲孔，不穿進馬達槽）(mm)"),
    ("cb_d", "6.2", "銷孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "銷孔沉頭深度 (mm)"),
]

COUPLER_PARAMS = [
    ("od", "10", "聯軸器外徑 (mm)"),
    ("tt_d", "5.4", "TT 馬達軸徑 (mm)"),
    ("tt_flat", "3.7", "TT 馬達軸雙平面距 (mm)"),
    ("tt_clear", "0.1", "TT 軸孔每邊間隙 (mm)"),
    ("tt_depth", "7.5", "TT 軸套入深度 (mm)"),
    ("web", "1", "兩孔之間隔板厚 (mm)"),
    ("axle_d", "4.8", "LEGO 十字軸外徑 (mm)"),
    ("axle_arm", "1.8", "LEGO 十字軸臂寬 (mm)"),
    ("lego_clear", "0.15", "十字孔每邊間隙：0.075 時十字軸插不進，改 0.15；換耗材請用 cross_fit_test 重挑 (mm)"),
    ("axle_depth", "8", "十字孔深度 (mm)"),
    ("length", "=tt_depth + web + axle_depth", "聯軸器總長（自動計算）"),
    ("hole_d", "=axle_d + 2 * lego_clear", "十字孔外徑（自動計算）"),
    ("hole_w", "=axle_arm + 2 * lego_clear", "十字孔臂寬（自動計算）"),
    ("wall", "=od / 2 - tt_d / 2 - tt_clear", "壁厚：外形沿內孔等距外擴，外形與內孔同形（自動計算）"),
    ("outer_flat", "=tt_flat + 2 * tt_clear + 2 * wall", "TT 段外形雙平面距，平面與 TT 孔平面平行（自動計算）"),
    ("cross_len", "=hole_d + 2 * wall", "LEGO 段十字外形臂長，臂與十字孔同向（自動計算）"),
    ("cross_w", "=hole_w + 2 * wall", "LEGO 段十字外形臂寬（自動計算）"),
    ("chamfer_r", "=sqrt((od / 2) ^ 2 - (cross_w / 2) ^ 2)",
     "十字臂端 45° 倒角起點：從 TT 段外緣往上斜出，TT 端朝下列印不懸空（自動計算）"),
]

P = "Params."


def pin_hole(body, name, x, z, s):
    """Blind Technic pin hole with its counterbore, drilled from the side face (s = +1 for +y, -1 for -y)."""
    for suffix, radius, depth in (("", "pin_d", "pin_depth"), ("Cb", "cb_d", "cb_depth")):
        cyl = body.newObject("PartDesign::SubtractiveCylinder", f"{name}{suffix}")
        # cylinders grow along +z; tip that axis to point into the wall
        cyl.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(1, 0, 0), 90 * s))
        bind(cyl, "Radius", f"{P}{radius} / 2")
        bind(cyl, "Height", f"{P}{depth} + 0.5")
        bind(cyl, ".Placement.Base.x", x)
        bind(cyl, ".Placement.Base.y", f"{s} * ({P}outer_w / 2 + 0.5)")
        bind(cyl, ".Placement.Base.z", z)


def build_adapter():
    doc = core.make_doc("tt_motor_technic", core.PARAMS + TECHNIC_PARAMS)
    body = doc.addObject("PartDesign::Body", "Adapter")
    core.build_core(body)

    # extend the front block back to the grid, keeping the gearbox tab slot open through it
    box(body, "BackBlock", False, f"-{P}back_x", P + "width", P + "block_h",
        x=P + "back_x", y=f"-{P}width / 2")
    box(body, "BackSlot", True, f"2 - {P}back_x", P + "hole_w", P + "hole_h",
        x=f"{P}back_x - 1", y=f"-{P}hole_w / 2", z=P + "hole_z")

    for tag, s in (("L", 1), ("R", -1)):
        wall_y = f"{P}inner_w / 2" if s > 0 else f"-{P}outer_w / 2"
        wall_w = f"({P}outer_w - {P}inner_w) / 2"
        # low wall along the whole length, tall wall up to the snap posts (posts stay thin so they flex)
        box(body, f"SideWall{tag}", False, f"{P}front_x - {P}back_x", wall_w, P + "rail_h",
            x=P + "back_x", y=wall_y)
        box(body, f"SideTower{tag}", False, f"{P}tower_x - {P}back_x", wall_w, P + "tower_h",
            x=P + "back_x", y=wall_y)
        for k in range(-2, 5):
            pin_hole(body, f"Pin{tag}_A{k + 3}", f"{P}shaft_x + {k} * {P}pitch", f"{P}pitch / 2", s)
        for k in range(-2, 2):
            pin_hole(body, f"Pin{tag}_B{k + 3}", f"{P}shaft_x + {k} * {P}pitch", f"1.5 * {P}pitch", s)

    core.finish(doc, body, "tt_motor_technic")


def build_coupler():
    doc = core.make_doc("tt_technic_coupler", COUPLER_PARAMS)
    body = doc.addObject("PartDesign::Body", "Coupler")

    # The outside follows the hole inside it, with an even wall, so the part shows its own orientation:
    # the LEGO end is a cross lined up with the cross hole (from the web up), the TT end a double-D whose
    # flats are parallel to the TT shaft flats.
    for name, length, width in (("CrossOuterX", "cross_len", "cross_w"), ("CrossOuterY", "cross_w", "cross_len")):
        box(body, name, False, P + length, P + width, f"{P}length - {P}tt_depth",
            x=f"-{P}{length} / 2", y=f"-{P}{width} / 2", z=P + "tt_depth")
    # the cross arms stick out past the double-D below them: bevel each arm end at 45 deg from the
    # double-D's edge so nothing hangs in the air when printed TT end down. Made before the double-D exists,
    # so the cutters' lower parts take nothing away from it.
    for k in range(4):
        cut = body.newObject("PartDesign::SubtractiveBox", f"TipChamfer{k}")
        for prop, value in (("Length", "20"), ("Width", "20"), ("Height", "5")):
            bind(cut, prop, value)
        # local x runs up and outward at 45 deg; the box's top face is the bevel through (chamfer_r, tt_depth)
        cut.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), 90 * k).multiply(
            App.Rotation(App.Vector(0, 1, 0), -45)))
        bx, by = [(f"{P}chamfer_r", "-10"), ("10", f"{P}chamfer_r"), (f"-{P}chamfer_r", "10"),
                  ("-10", f"-{P}chamfer_r")][k]
        bind(cut, ".Placement.Base.x", bx)
        bind(cut, ".Placement.Base.y", by)
        bind(cut, ".Placement.Base.z", f"{P}tt_depth - 5 * sqrt(2)")

    shell = body.newObject("PartDesign::AdditiveCylinder", "Shell")
    bind(shell, "Radius", f"{P}od / 2")
    bind(shell, "Height", P + "tt_depth")
    for tag, s in (("A", 1), ("B", -1)):
        box(body, f"OuterFlat{tag}", True, "5", f"{P}od + 2", f"{P}tt_depth + 1",
            x=f"{P}outer_flat / 2" if s > 0 else f"-{P}outer_flat / 2 - 5", y=f"-({P}od / 2 + 1)", z="-1")

    # TT double-D socket from the bottom: a round bore, then the two flats added back
    bore = body.newObject("PartDesign::SubtractiveCylinder", "TTBore")
    bind(bore, "Radius", f"{P}tt_d / 2 + {P}tt_clear")
    bind(bore, "Height", f"{P}tt_depth + 1")
    bind(bore, ".Placement.Base.z", "-1")
    for tag, s in (("A", 1), ("B", -1)):
        flat_x = f"{P}tt_flat / 2 + {P}tt_clear" if s > 0 else f"-{P}tt_d / 2 - {P}tt_clear - 0.5"
        box(body, f"TTFlat{tag}", False, f"({P}tt_d - {P}tt_flat) / 2 + 0.5", f"{P}tt_d + 2 * {P}tt_clear + 1",
            P + "tt_depth", x=flat_x, y=f"-({P}tt_d / 2 + {P}tt_clear + 0.5)")

    # LEGO cross hole from the top: two crossed bars
    for name, length, width in (("CrossHoleX", "hole_d", "hole_w"), ("CrossHoleY", "hole_w", "hole_d")):
        box(body, name, True, P + length, P + width, f"{P}axle_depth + 1",
            x=f"-{P}{length} / 2", y=f"-{P}{width} / 2", z=f"{P}tt_depth + {P}web")

    core.finish(doc, body, "tt_technic_coupler")


# white PLA: -0.1 .. +0.05 were all too tight, so the second round spans +0.075 .. +0.175
FIT_CLEARS = ["0.075", "0.1", "0.125", "0.15", "0.175"]
FIT_PARAMS = [
    ("axle_d", "4.8", "LEGO 十字軸外徑 (mm)"),
    ("axle_arm", "1.8", "LEGO 十字軸臂寬 (mm)"),
    ("gap", "10", "孔距 (mm)"),
    ("plate_t", "8", "試片厚度 = 十字孔深度 (mm)"),
    ("plate_w", "10", "試片寬度 (mm)"),
] + [(f"c{i + 1}", c, f"第 {i + 1} 孔每邊間隙（孔旁 {i + 1} 個圓點）(mm)") for i, c in enumerate(FIT_CLEARS)]


def build_fit_test():
    """Strip with four through cross holes at different clearances; dots beside each hole count 1..4."""
    doc = core.make_doc("cross_fit_test", FIT_PARAMS)
    body = doc.addObject("PartDesign::Body", "FitTest")
    n = len(FIT_CLEARS)
    box(body, "Plate", False, f"{n} * {P}gap", P + "plate_w", P + "plate_t")
    for i in range(n):
        cx = f"({i} + 0.5) * {P}gap"
        hole_d = f"{P}axle_d + 2 * {P}c{i + 1}"
        hole_w = f"{P}axle_arm + 2 * {P}c{i + 1}"
        for name, length, width in ((f"Cross{i + 1}X", hole_d, hole_w), (f"Cross{i + 1}Y", hole_w, hole_d)):
            box(body, name, True, length, width, f"{P}plate_t + 2",
                x=f"{cx} - ({length}) / 2", y=f"{P}plate_w / 2 - ({width}) / 2", z="-1")
        for j in range(i + 1):
            dot = body.newObject("PartDesign::SubtractiveCylinder", f"Dot{i + 1}_{j + 1}")
            bind(dot, "Radius", "0.5")
            bind(dot, "Height", "1")
            bind(dot, ".Placement.Base.x", f"{cx} + ({j} - {i} / 2) * 1.6")
            bind(dot, ".Placement.Base.y", f"{P}plate_w - 1.2")
            bind(dot, ".Placement.Base.z", f"{P}plate_t - 0.6")
    core.finish(doc, body, "cross_fit_test")


build_adapter()
build_coupler()
build_fit_test()
core.gui_resave("tt_motor_technic", "tt_technic_coupler", "cross_fit_test")
