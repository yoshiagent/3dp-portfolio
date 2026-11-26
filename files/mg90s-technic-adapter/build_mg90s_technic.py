"""LEGO Technic adapter for the MG90S micro servo: a frame that holds the servo body, a lid over it, and a hub
that turns its cross horn into a LEGO cross-axle output. Saves mg90s_technic_frame.FCStd,
mg90s_technic_lid.FCStd and mg90s_horn_adapter.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_mg90s_technic.py      (MG_PARTS="frame", "lid" or "horn", ';'-separated, builds only those)

Lid: 6 x 5 x 1 units (47.4 x 39.4 x 7.8), the frame's footprint, pinned on top of it. Underneath: a recess
that just clears the tabs (0.1), so the lid keeps the servo down, room for the two tab screw heads, a deeper
recess for the case above the tabs, and a hole for the round gear housing (Ø14.6 measured, centred 1.9 from
the shaft toward the body, so it is offset from the output shaft) to stand through. Counterbored
Technic holes in line with the frame's: through holes along both long sides and at x = 16, blind top holes
at x = -24 (y = +-8), side holes at mid-thickness in all four faces (5 mm deep at the left end, where the
tab recess comes close).

Servo sizes follow the TowerPro MG90S datasheet drawing (body 22.5 x 12, tab span 32.5, bottom to tab 21,
35.5 to the top of a fitted horn); the rest are common micro-servo values. Everything that depends on the
servo or its horn is in the Params sheet, so measured values can be typed in and the files recomputed.

Frame: 6 x 5 x 3 Technic units (47.4 x 39.4 x 23.8). The servo drops in from the top and its two mounting
tabs rest on the frame top, held by the servo's own two screws in pilot holes. The output shaft sits on a
grid point (x = 0, y = 0), so a LEGO axle in the horn adapter lines up with the Technic holes around it.
Frame top z = 0 is the underside of the tabs; the frame reaches 23.8 down, the servo body stops 0.3 above a
5 mm floor. Holes, all with counterbores like a LEGO beam:
  - vertical through holes in the long walls (y = +-16) and end walls (x = -24, 16)
  - holes up from the bottom under the servo (y = 0: one pin deep, open into the pocket; y = +-8: 5 mm, to the
    pocket floor level, so they do not cut into the pocket), so the bottom is a full 6 x 5 Technic grid
    except at the cable slot
  - side holes one pin deep in all four faces, three rows (z = -3.9, -11.9, -19.9)
  - a slot under the shaft end, open at the bottom, for the servo cable (it leaves the case low at that end);
    8.6 wide x 9 above the pocket floor, so the 3-pin Dupont plug (7.6 x 2.6 x 14) is put into the empty
    pocket and pushed out through it, flat or on edge, before the servo goes in
Horn adapter: a disc that sits over the servo's 4-arm cross horn (a recess for the arms underneath, a pocket
for the horn screw head), fixed to the horn by small screws through the arm holes; a LEGO cross hole on top on
the shaft axis and four Technic pin holes around it.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

FRAME_PARAMS = [
    # frame on the Technic grid
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("frame_l", "47.4", "框總長：6 格 (mm)"),
    ("frame_w", "39.4", "框總寬：5 格 (mm)"),
    ("frame_h", "23.8", "框總高：3 格 (mm)"),
    ("frame_cx", "=-pitch / 2", "框中心 x：輸出軸在 x = 0 的孔格上（自動計算）"),
    ("corner_r", "3.7", "框四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側孔、底孔深度：一根銷的長度 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # servo (datasheet / common values: measure and correct)
    ("body_l", "22.8", "伺服機本體長：datasheet 22.5，常見實測 22.8 (mm)"),
    ("body_w", "12.2", "伺服機本體寬：datasheet 12，常見實測 12.2 (mm)"),
    ("shaft_end", "6", "輸出軸中心到近端本體端面 (mm)"),
    ("tab_z", "18.5", "本體底面到耳片底面：datasheet 21 為到耳片頂面，耳片厚 2.5 (mm)"),
    ("screw_span", "28", "兩耳片螺絲孔中心距 (mm)"),
    ("pilot_d", "1.6", "耳片螺絲導孔直徑：M2 自攻螺絲 (mm)"),
    ("pilot_depth", "8", "導孔深度 (mm)"),
    ("body_clear", "0.4", "本體四周間隙 (mm)"),
    ("floor_gap", "0.3", "本體底面離槽底 (mm)"),
    ("cable_w", "8.6", "電線槽寬：3pin 杜邦頭 7.6 寬 + 間隙，接頭平放或側立都過得去 (mm)"),
    ("cable_h", "9", "電線槽高出槽底：杜邦頭側立 7.6 + 間隙；再高會接近右耳片導孔 (mm)"),
    # derived
    ("pocket_x0", "=-(body_l - shaft_end) - body_clear", "槽左端 x（自動計算）"),
    ("pocket_x1", "=shaft_end + body_clear", "槽右端 x（自動計算）"),
    ("pocket_w", "=body_w + 2 * body_clear", "槽寬（自動計算）"),
    ("pocket_z", "=-tab_z - floor_gap", "槽底 z（自動計算）"),
    ("body_cx", "=(pocket_x0 + pocket_x1) / 2", "本體中心 x（自動計算）"),
]

HORN_PARAMS = [
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("disc_d", "25.6", "轉接盤直徑 (mm)"),
    ("disc_h", "12", "轉接盤高 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # cross horn (common values: measure and correct)
    ("horn_r", "11", "十字舵盤臂端半徑：中心到最長臂端 (mm)"),
    ("arm_w", "5.4", "舵盤臂寬：取臂根部最寬處 (mm)"),
    ("arm_t", "1.8", "舵盤臂厚 (mm)"),
    ("horn_clear", "0.25", "舵盤凹槽每邊間隙 (mm)"),
    ("head_d", "5.2", "舵盤中心螺絲頭讓位直徑 (mm)"),
    ("head_h", "2", "舵盤中心螺絲頭讓位高度 (mm)"),
    ("screw_r", "8", "固定螺絲在舵盤臂上的半徑：對齊臂上的孔 (mm)"),
    ("screw_d", "2.2", "固定螺絲穿孔直徑：M2 (mm)"),
    ("screw_cb_d", "4.2", "固定螺絲頭沉孔直徑 (mm)"),
    ("screw_cb_h", "1.6", "固定螺絲頭沉孔深度 (mm)"),
    # LEGO cross hole, as on the TT coupler
    ("axle_d", "4.8", "LEGO 十字軸外徑 (mm)"),
    ("axle_arm", "1.8", "LEGO 十字軸臂寬 (mm)"),
    ("lego_clear", "0.075", "十字孔每邊間隙：白色 PLA 試片第 1 孔最合適 (mm)"),
    ("axle_depth", "8", "十字孔深度 (mm)"),
    ("hole_d", "=axle_d + 2 * lego_clear", "十字孔外徑（自動計算）"),
    ("hole_w", "=axle_arm + 2 * lego_clear", "十字孔臂寬（自動計算）"),
    ("slot_l", "=2 * (horn_r + horn_clear)", "舵盤凹槽長（自動計算）"),
    ("slot_w", "=arm_w + 2 * horn_clear", "舵盤凹槽寬（自動計算）"),
    ("slot_h", "=arm_t + 0.2", "舵盤凹槽深（自動計算）"),
]

LID_PARAMS = [
    # lid on the Technic grid, same footprint as the frame
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("lid_l", "47.4", "上蓋長：6 格 (mm)"),
    ("lid_w", "39.4", "上蓋寬：5 格 (mm)"),
    ("lid_h", "7.8", "上蓋厚：1 格 (mm)"),
    ("lid_cx", "=-pitch / 2", "上蓋中心 x：與固定座對齊（自動計算）"),
    ("corner_r", "3.7", "四角圓角半徑 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側孔深度：一根銷的長度 (mm)"),
    ("end_depth", "5", "左端側孔深度：再深會碰到耳片凹槽 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # servo above the tabs (datasheet / common values: measure and correct)
    ("body_l", "22.8", "伺服機本體長 (mm)"),
    ("body_w", "12.2", "伺服機本體寬 (mm)"),
    ("shaft_end", "6", "輸出軸中心到近端本體端面 (mm)"),
    ("tab_span", "32.5", "耳片總長：datasheet (mm)"),
    ("tab_t", "2.5", "耳片厚 (mm)"),
    ("case_top", "4.7", "耳片底面到上殼頂面：照片推估 (mm)"),
    ("housing_d", "14.6", "圓形齒輪罩直徑：實測 (mm)"),
    ("housing_inset", "0.7", "齒輪罩離輸出軸端端面：照片推估 (mm)"),
    ("housing_clear", "0.3", "齒輪罩四周間隙 (mm)"),
    ("screw_span", "28", "兩耳片螺絲孔中心距 (mm)"),
    ("head_d", "4.6", "耳片螺絲頭讓位直徑 (mm)"),
    ("head_h", "1.8", "耳片螺絲頭讓位高度：耳片頂面以上 (mm)"),
    ("body_clear", "0.4", "本體與凸台四周間隙 (mm)"),
    ("tab_clear", "0.3", "耳片兩端間隙 (mm)"),
    ("tab_gap", "0.1", "耳片頂面到上蓋的間隙：上蓋輕壓住耳片 (mm)"),
    ("top_gap", "0.3", "本體上蓋頂面到凹槽頂的間隙 (mm)"),
    # derived
    ("body_cx", "=shaft_end - body_l / 2", "本體中心 x（自動計算）"),
    ("recess_w", "=body_w + 2 * body_clear", "凹槽寬（自動計算）"),
    ("tab_recess_l", "=tab_span + 2 * tab_clear", "耳片凹槽長（自動計算）"),
    ("tab_recess_h", "=tab_t + tab_gap", "耳片凹槽深（自動計算）"),
    ("body_recess_l", "=body_l + 2 * body_clear", "本體凹槽長（自動計算）"),
    ("body_recess_h", "=case_top + top_gap", "本體凹槽深（自動計算）"),
    ("housing_cx", "=shaft_end - housing_inset - housing_d / 2", "齒輪罩中心 x（自動計算）"),
]

P = "Params."
CB_OVERLAP = 0.02  # counterbores this much deeper, so neighbouring counterbores never just touch
COS45 = "0.70710678"


def cylinder(body, name, cut, radius, height, x, y, z, rot=None):
    cyl = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    if rot is not None:
        cyl.Placement = App.Placement(App.Vector(), rot)
    bind(cyl, "Radius", radius)
    bind(cyl, "Height", height)
    bind(cyl, ".Placement.Base.x", x)
    bind(cyl, ".Placement.Base.y", y)
    bind(cyl, ".Placement.Base.z", z)
    return cyl


def vertical_hole(body, name, x, y, z_bot, z_top, cb_bot=True, cb_top=True):
    """Vertical pin hole from z_bot to z_top, counterbored at the chosen ends. Returns its features.
    The cylinder starts 1 mm below z_bot; it runs 1 mm past z_top only when that end is open (cb_top), so a
    blind hole stops exactly at z_top."""
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    over = 2 if cb_top else 1
    feats = [cylinder(body, name, True, f"{P}pin_d / 2", f"({z_top}) - ({z_bot}) + {over}", x, y, f"{z_bot} - 1")]
    if cb_bot:
        feats.append(cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x, y, f"{z_bot} - 1"))
    if cb_top:
        feats.append(cylinder(body, f"{name}CbT", True, f"{P}cb_d / 2", cb_h, x, y,
                              f"{z_top} - {P}cb_depth - {CB_OVERLAP}"))
    return feats


def side_hole(body, name, axis, angle, x, y, z):
    """Pin hole one pin deep with counterbore, drilled from (x, y, z) 0.5 mm outside a face;
    App.Rotation(axis, angle) tips the cylinder's +z to point into the part. Returns its features."""
    rot = App.Rotation(App.Vector(*axis), angle)
    return [cylinder(body, name, True, f"{P}pin_d / 2", f"{P}pin_depth + 0.5", x, y, z, rot),
            cylinder(body, f"{name}Cb", True, f"{P}cb_d / 2", f"{P}cb_depth + {CB_OVERLAP} + 0.5", x, y, z, rot)]


def rounded_block(body, pre, z0):
    """Block `pre`_l x `pre`_w x `pre`_h (spreadsheet aliases) from z0 up, centred on (`pre`_cx, 0),
    vertical corners rounded."""
    L, W, H, CX = (f"{P}{pre}_{s}" for s in ("l", "w", "h", "cx"))
    x0, y0 = f"{CX} - {L} / 2", f"-{W} / 2"
    box(body, "Block", False, L, W, H, x=x0, y=y0, z=z0)
    for tag, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)):
        cx = f"{CX} + {sx} * ({L} / 2 - {P}corner_r)"
        cy = f"{sy} * ({W} / 2 - {P}corner_r)"
        box(body, f"Corner{tag}", True, f"{P}corner_r + 1", f"{P}corner_r + 1", f"{H} + 2",
            x=cx if sx > 0 else f"{cx} - {P}corner_r - 1", y=cy if sy > 0 else f"{cy} - {P}corner_r - 1",
            z=f"{z0} - 1")
        cylinder(body, f"Round{tag}", False, P + "corner_r", H, cx, cy, z0)


def build_frame(stem):
    doc = core.make_doc(stem, FRAME_PARAMS)
    body = doc.addObject("PartDesign::Body", "Frame")
    rounded_block(body, "frame", f"-{P}frame_h")
    ZB, ZT = f"-{P}frame_h", "0"
    g = lambda k: f"{k} * {P}pitch"  # noqa: E731  # grid coordinate
    row_z = {"U": f"-{P}pitch / 2 + 0.1", "M": f"-1.5 * {P}pitch + 0.1", "D": f"-2.5 * {P}pitch + 0.1"}  # -3.9 ...

    # servo pocket, open at the top; mounting-screw pilots in the end walls under the tabs
    box(body, "Pocket", True, f"{P}pocket_x1 - {P}pocket_x0", P + "pocket_w", f"-{P}pocket_z + 1",
        x=P + "pocket_x0", y=f"-{P}pocket_w / 2", z=P + "pocket_z")
    for tag, s in (("L", -1), ("R", 1)):
        cylinder(body, f"Pilot{tag}", True, f"{P}pilot_d / 2", f"{P}pilot_depth + 1",
                 f"{P}body_cx + {s} * {P}screw_span / 2", "0", f"-{P}pilot_depth")
    # cable slot under the shaft end, through the end wall and open at the bottom
    box(body, "CableSlot", True, f"{P}frame_cx + {P}frame_l / 2 - {P}pocket_x1 + 2", P + "cable_w",
        f"{P}frame_h + {P}pocket_z + {P}cable_h + 1",
        x=f"{P}pocket_x1 - 1", y=f"-{P}cable_w / 2", z=f"{ZB} - 1")

    # vertical through holes: long walls (6 per side) ...
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"VLong{t}", vertical_hole(body, f"VLong{t}h", g(-3), g(2 * s), ZB, ZT), "X", 6, g(5))
    # ... end walls: x = -24 (y = -8, 0, 8) and x = 16 (y = +-8; y = 0 is the cable slot)
    core.pattern(body, "VEndL", vertical_hole(body, "VEndLh", g(-3), g(-1), ZB, ZT), "Y", 3, g(2))
    core.pattern(body, "VEndR", vertical_hole(body, "VEndRh", g(2), g(-1), ZB, ZT), "Y", 2, g(2))
    # bottom holes under the servo. Holes at y = +-8 (and x = 8) would cut into the pocket's sides and
    # corners above the floor, so they stop at the pocket floor level (5 mm); the y = 0 row goes one pin deep
    # and opens into the pocket.
    core.pattern(body, "VFloorMid", vertical_hole(body, "VFloorMidh", g(-2), "0", ZB, f"{ZB} + {P}pin_depth",
                                                  cb_top=False), "X", 3, g(2))
    core.pattern(body, "VFloorSide", vertical_hole(body, "VFloorSideh", g(-2), g(-1), ZB, P + "pocket_z",
                                                   cb_top=False), "X", 4, g(3), "Y", 2, g(2))

    # side holes in the long faces: 3 rows x 6 columns per face
    for row, z in row_z.items():
        for t, s in (("L", 1), ("R", -1)):
            core.pattern(body, f"S{row}{t}", side_hole(body, f"S{row}{t}h", (1, 0, 0), 90 * s, g(-3),
                                                       f"{s} * ({P}frame_w / 2 + 0.5)", z), "X", 6, g(5))
    # side holes in the end faces, y = -8, 0, 8, leaving out the ones that would meet the left pilot
    # (top row, y = 0) or the cable slot (lower rows, y = 0, right end)
    xl, xr = f"{P}frame_cx - {P}frame_l / 2 - 0.5", f"{P}frame_cx + {P}frame_l / 2 + 0.5"
    for row, z in row_z.items():
        for t, s, x in (("L", -1, xl), ("R", 1, xr)):
            full = (t == "L" and row != "U") or (t == "R" and row == "U")
            core.pattern(body, f"E{row}{t}", side_hole(body, f"E{row}{t}h", (0, 1, 0), -90 * s, x, g(-1), z),
                         "Y", 3 if full else 2, g(2))
    core.finish(doc, body, stem)
    return stem


def build_horn(stem):
    doc = core.make_doc(stem, HORN_PARAMS)
    body = doc.addObject("PartDesign::Body", "HornAdapter")
    # disc, bottom (z = 0) level with the underside of the horn arms
    cylinder(body, "Disc", False, f"{P}disc_d / 2", P + "disc_h", "0", "0", "0")
    # recess for the 4-arm horn underneath, arms at 45 deg so the pin holes fit between them
    for tag, ang in (("A", 45), ("B", -45)):
        slot = box(body, f"HornSlot{tag}", True, P + "slot_l", P + "slot_w", f"{P}slot_h + 1")
        slot.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), ang))
        s = 1 if ang > 0 else -1
        # corner of a box centred on the origin and turned by +-45 deg
        bind(slot, ".Placement.Base.x", f"-({P}slot_l - {s} * {P}slot_w) / 2 * {COS45}" if s > 0
             else f"-({P}slot_l + {P}slot_w) / 2 * {COS45}")
        bind(slot, ".Placement.Base.y", f"-({P}slot_l + {P}slot_w) / 2 * {COS45}" if s > 0
             else f"({P}slot_l - {P}slot_w) / 2 * {COS45}")
        bind(slot, ".Placement.Base.z", "-1")
    # room for the horn's centre screw head
    cylinder(body, "HeadPocket", True, f"{P}head_d / 2", f"{P}slot_h + {P}head_h", "0", "0", "-0.5")
    # LEGO cross hole from the top, down into the screw-head pocket
    for name, length, width in (("CrossX", "hole_d", "hole_w"), ("CrossY", "hole_w", "hole_d")):
        box(body, name, True, P + length, P + width, f"{P}axle_depth + 1",
            x=f"-{P}{length} / 2", y=f"-{P}{width} / 2", z=f"{P}disc_h - {P}axle_depth")
    # screws through the arm holes, heads sunk into the top
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)), start=1):
        x, y = f"{sx} * {P}screw_r * {COS45}", f"{sy} * {P}screw_r * {COS45}"
        cylinder(body, f"Screw{i}", True, f"{P}screw_d / 2", f"{P}disc_h + 2", x, y, "-1")
        cylinder(body, f"Screw{i}Cb", True, f"{P}screw_cb_d / 2", f"{P}screw_cb_h + 1", x, y,
                 f"{P}disc_h - {P}screw_cb_h")
    # four Technic pin holes on the axes, counterbored on top
    for i, (x, y) in enumerate(((f"{P}pitch", "0"), ("0", f"{P}pitch"), (f"-{P}pitch", "0"), ("0", f"-{P}pitch")),
                               start=1):
        vertical_hole(body, f"Pin{i}", x, y, "0", P + "disc_h", cb_bot=False)
    core.finish(doc, body, stem)
    return stem


def build_lid(stem):
    """Lid on top of the frame (z = 0 sits on the frame top): recesses underneath for the tabs, the screw heads
    and the upper case, a hole for the round gear housing, and Technic holes in line with the frame's."""
    doc = core.make_doc(stem, LID_PARAMS)
    body = doc.addObject("PartDesign::Body", "Lid")
    rounded_block(body, "lid", "0")
    ZT = P + "lid_h"
    g = lambda k: f"{k} * {P}pitch"  # noqa: E731
    # underneath: tab recess (lid just clears the tabs, so it keeps the servo down), deeper recess for the
    # case above the tabs, room for the two tab screw heads; gear-housing hole through
    box(body, "TabRecess", True, P + "tab_recess_l", P + "recess_w", f"{P}tab_recess_h + 1",
        x=f"{P}body_cx - {P}tab_recess_l / 2", y=f"-{P}recess_w / 2", z="-1")
    box(body, "BodyRecess", True, P + "body_recess_l", P + "recess_w", f"{P}body_recess_h + 1",
        x=f"{P}body_cx - {P}body_recess_l / 2", y=f"-{P}recess_w / 2", z="-1")
    for tag, s in (("L", -1), ("R", 1)):
        cylinder(body, f"ScrewHead{tag}", True, f"{P}head_d / 2", f"{P}tab_recess_h + {P}head_h + 1",
                 f"{P}body_cx + {s} * {P}screw_span / 2", "0", "-1")
    cylinder(body, "HousingHole", True, f"{P}housing_d / 2 + {P}housing_clear", f"{ZT} + 2", P + "housing_cx", "0",
             "-1")

    # through holes in line with the frame: long sides (6 each) and x = 16 (y = +-8)
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"VLong{t}", vertical_hole(body, f"VLong{t}h", g(-3), g(2 * s), "0", ZT), "X", 6, g(5))
    core.pattern(body, "VEndR", vertical_hole(body, "VEndRh", g(2), g(-1), "0", ZT), "Y", 2, g(2))
    # x = -24, y = +-8 would cut into the tab recess as through holes, so they are blind from the top and stop
    # 1.2 mm above it (vertical_hole starts its cylinder 1 mm below z_bot). (-24, 0) would meet the screw-head
    # recess and (8, +-8) the case recess, so those are left out.
    core.pattern(body, "VTopL", vertical_hole(body, "VTopLh", g(-3), g(-1), f"{P}tab_recess_h + 2.2", ZT,
                                              cb_bot=False), "Y", 2, g(2))

    # side holes at mid-thickness: long faces one pin deep; right end one pin deep; left end shallower
    z = f"{ZT} / 2"
    for t, s in (("L", 1), ("R", -1)):
        core.pattern(body, f"S{t}", side_hole(body, f"S{t}h", (1, 0, 0), 90 * s, g(-3),
                                              f"{s} * ({P}lid_w / 2 + 0.5)", z), "X", 6, g(5))
    core.pattern(body, "ER", side_hole(body, "ERh", (0, 1, 0), -90, f"{P}lid_cx + {P}lid_l / 2 + 0.5", g(-1), z),
                 "Y", 3, g(2))
    left = side_hole(body, "ELh", (0, 1, 0), 90, f"{P}lid_cx - {P}lid_l / 2 - 0.5", g(-1), z)
    bind(left[0], "Height", f"{P}end_depth + 0.5")
    core.pattern(body, "EL", left, "Y", 3, g(2))
    core.finish(doc, body, stem)
    return stem


PARTS = {"frame": (build_frame, "mg90s_technic_frame"), "horn": (build_horn, "mg90s_horn_adapter"),
         "lid": (build_lid, "mg90s_technic_lid")}
todo = [p for p in os.environ.get("MG_PARTS", "frame;horn;lid").split(";") if p]
core.gui_resave(*(PARTS[p][0](PARTS[p][1]) for p in todo))
