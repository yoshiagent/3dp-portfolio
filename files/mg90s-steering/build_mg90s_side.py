"""MG90S LEGO Technic holder, side-loading version: a base whose inside is the servo's side profile, opened
toward one long side, and a side cover that closes it. To compare with the top-loading frame + lid
(build_mg90s_technic.py). Saves mg90s_side_base.FCStd and mg90s_side_cover.FCStd (+ STEP/STL in out/).

Run:  freecadcmd.exe build_mg90s_side.py      (SIDE_PARTS="base" or "cover", ';'-separated, builds only those)

Same coordinates as the top-loading set: output shaft on z through (0, 0), z = 0 at the underside of the servo
tabs, x along the servo body, y across it. Together the two parts fill 6 x 5 x 4 Technic units
(47.4 x 39.4 x 31.8), the same footprint as frame + lid.

Base: 6 x 4 x 4 units (47.4 x 31.4 x 31.8). The servo slides in along +-y (perpendicular to its label face):
the cavity is the servo's side profile swept from just behind the servo out through the front face (y = 11.7):
  - body and the case above the tabs (0.4 all round, 0.3 above)
  - a slot for the two tabs (0.3 at each end, 0.1 above and below), so the tabs hold the servo up and down
    without screws
  - the round gear housing (Ø14.6, measured) in a round-ended slot through the top, so the housing and shaft
    stand out of the top
  - a groove in the shaft-end wall for the cable, which leaves the case low at that end
Technic holes, counterbored: vertical through the back row (y = -16) and the right end (x = 16), blind ones at
the left end (bottom one pin deep, top 4 mm, above and below the tab slot) and under the servo; horizontal
one pin deep in the back face (4 rows x 6), both ends, and in the front face where the base is solid (left
and right end blocks) for the pins that hold the cover.
Cover: 6 x 1 x 4 units (47.4 x 7.4 x 31.8), 0.6 in front of the base. A plug on its inner face fills the
cavity's opening up to 0.2 from the servo (round cut-out for the gear housing), so the servo cannot slide out.
Through holes across it line up with the base's front-face holes; a notch open at the bottom lets the cable out.

Servo sizes above the tabs come from the user's photo (gear housing 14.6 long, measured) with heights scaled
from the picture; the rest follow the datasheet as in the top-loading set. All are in the Params sheet.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

PARAMS = [
    # the set on the Technic grid
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("set_l", "47.4", "整組長：6 格 (mm)"),
    ("set_w", "39.4", "整組寬：5 格，底座 4 格＋側蓋 1 格 (mm)"),
    ("set_h", "31.8", "整組高：4 格 (mm)"),
    ("set_cx", "=-pitch / 2", "整組中心 x：輸出軸在 x = 0 的孔格上（自動計算）"),
    ("set_z0", "-23.8", "底面 z：耳片底面為 z = 0 (mm)"),
    ("corner_r", "3.7", "外側圓角半徑 (mm)"),
    ("cover_t", "7.4", "側蓋厚：1 格 (mm)"),
    ("cover_gap", "0.6", "側蓋與底座間隙：Technic 孔格間距 (mm)"),
    ("pin_d", "4.94", "銷孔直徑 (mm)"),
    ("pin_depth", "8.4", "側孔、底孔深度：一根銷的長度 (mm)"),
    ("end_depth", "5", "左端最上排側孔深度：再深會碰到耳片槽 (mm)"),
    ("end_depth_r", "7", "右端最上排側孔深度：再深會碰到耳片槽 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    # servo (datasheet, user's photo: measure and correct)
    ("body_l", "22.8", "伺服機本體長 (mm)"),
    ("body_w", "12.2", "伺服機本體寬：滑入方向 (mm)"),
    ("shaft_end", "6", "輸出軸中心到近端本體端面 (mm)"),
    ("tab_z", "18.5", "本體底面到耳片底面 (mm)"),
    ("tab_span", "32.5", "耳片總長 (mm)"),
    ("tab_t", "2.5", "耳片厚 (mm)"),
    ("case_top", "4.7", "耳片底面到上殼頂面：照片推估 (mm)"),
    ("housing_d", "14.6", "圓形齒輪罩直徑：實測 (mm)"),
    ("housing_inset", "0.7", "齒輪罩離輸出軸端端面：照片推估 (mm)"),
    ("housing_top", "9.9", "耳片底面到齒輪罩頂面：照片推估，只用在示意組裝 (mm)"),
    # clearances
    ("body_clear", "0.4", "本體四周間隙 (mm)"),
    ("floor_gap", "0.3", "本體底面離槽底 (mm)"),
    ("tab_clear", "0.3", "耳片兩端間隙 (mm)"),
    ("tab_gap", "0.1", "耳片上下間隙 (mm)"),
    ("top_gap", "0.3", "上殼頂面以上間隙 (mm)"),
    ("housing_clear", "0.3", "齒輪罩四周間隙 (mm)"),
    ("plug_fit", "0.15", "側蓋塞塊與槽四周間隙 (mm)"),
    ("plug_gap", "0.2", "側蓋塞塊到伺服機側面 (mm)"),
    ("cable_t", "2.2", "電線溝深：從槽端面往外 (mm)"),
    ("cable_h", "6", "電線溝高：從槽底往上 (mm)"),
    # derived
    ("base_w", "=set_w - cover_t - cover_gap", "底座寬（自動計算）"),
    ("base_y1", "=-set_w / 2 + base_w", "底座前面 y（自動計算）"),
    ("cover_y0", "=base_y1 + cover_gap", "側蓋內面 y（自動計算）"),
    ("set_z1", "=set_z0 + set_h", "頂面 z（自動計算）"),
    ("body_cx", "=shaft_end - body_l / 2", "本體中心 x（自動計算）"),
    ("cav_x0", "=shaft_end - body_l - body_clear", "本體槽左端 x（自動計算）"),
    ("cav_x1", "=shaft_end + body_clear", "本體槽右端 x（自動計算）"),
    ("cav_y0", "=-body_w / 2 - body_clear", "槽後壁 y（自動計算）"),
    ("cav_z0", "=-tab_z - floor_gap", "槽底 z（自動計算）"),
    ("case_z1", "=case_top + top_gap", "上殼槽頂 z（自動計算）"),
    ("tab_x0", "=body_cx - tab_span / 2 - tab_clear", "耳片槽左端 x（自動計算）"),
    ("tab_l", "=tab_span + 2 * tab_clear", "耳片槽長（自動計算）"),
    ("tab_z0", "=-tab_gap", "耳片槽底 z（自動計算）"),
    ("tab_h", "=tab_t + 2 * tab_gap", "耳片槽高（自動計算）"),
    ("housing_cx", "=shaft_end - housing_inset - housing_d / 2", "齒輪罩中心 x（自動計算）"),
    ("housing_r", "=housing_d / 2 + housing_clear", "齒輪罩槽半徑（自動計算）"),
    ("plug_y0", "=body_w / 2 + plug_gap", "塞塊內端 y（自動計算）"),
]

P = "Params."
CB_OVERLAP = 0.02  # counterbores this much deeper, so neighbouring counterbores never just touch
ZB, ZT = f"{P}set_z0", f"{P}set_z1"
HZ = f"({P}case_top - {P}top_gap)"   # bottom of the housing slot


def g(k):
    """Grid coordinate k pitches from the origin."""
    return f"{k} * {P}pitch"


def zrow(i):
    """Side-hole row i (0 = lowest): -19.9, -11.9, -3.9, 4.1."""
    return f"{P}set_z0 + {P}pitch / 2 - 0.1 + {i} * {P}pitch"


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
    """Vertical pin hole from z_bot to z_top, counterbored at the chosen ends (open ends run 1 mm past)."""
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    under = 1 if cb_bot else 0
    over = 1 if cb_top else 0
    feats = [cylinder(body, name, True, f"{P}pin_d / 2", f"({z_top}) - ({z_bot}) + {under + over}", x, y,
                      f"({z_bot}) - {under}")]
    if cb_bot:
        feats.append(cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x, y, f"({z_bot}) - 1"))
    if cb_top:
        feats.append(cylinder(body, f"{name}CbT", True, f"{P}cb_d / 2", cb_h, x, y,
                              f"({z_top}) - {P}cb_depth - {CB_OVERLAP}"))
    return feats


def side_hole(body, name, axis, angle, x, y, z, depth=None):
    """Counterbored pin hole drilled from (x, y, z) 0.5 mm outside a face, `depth` deep (one pin by default);
    App.Rotation(axis, angle) tips the cylinder's +z to point into the part."""
    rot = App.Rotation(App.Vector(*axis), angle)
    return [cylinder(body, name, True, f"{P}pin_d / 2", f"({depth or P + 'pin_depth'}) + 0.5", x, y, z, rot),
            cylinder(body, f"{name}Cb", True, f"{P}cb_d / 2", f"{P}cb_depth + {CB_OVERLAP} + 0.5", x, y, z, rot)]


def rounded_box(body, tag, x0, y0, z0, L, W, H, corners):
    """Block from (x0, y0, z0), L x W x H, with the vertical edges at `corners` ((sx, sy) signs) rounded."""
    box(body, f"{tag}Block", False, L, W, H, x=x0, y=y0, z=z0)
    for sx, sy in corners:
        cx = f"({x0}) + {L} / 2 + {sx} * ({L} / 2 - {P}corner_r)"
        cy = f"({y0}) + {W} / 2 + {sy} * ({W} / 2 - {P}corner_r)"
        name = f"{tag}{'R' if sx > 0 else 'L'}{'F' if sy > 0 else 'B'}"
        box(body, f"Corner{name}", True, f"{P}corner_r + 1", f"{P}corner_r + 1", f"{H} + 2",
            x=cx if sx > 0 else f"{cx} - {P}corner_r - 1", y=cy if sy > 0 else f"{cy} - {P}corner_r - 1",
            z=f"{z0} - 1")
        cylinder(body, f"Round{name}", False, P + "corner_r", H, cx, cy, z0)


X0 = f"{P}set_cx - {P}set_l / 2"   # left end face, x = -27.7
X1 = f"{P}set_cx + {P}set_l / 2"   # right end face, x = 19.7


def build_base(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "SideBase")
    rounded_box(body, "Base", X0, f"-{P}set_w / 2", ZB, P + "set_l", P + "base_w", P + "set_h",
                ((1, -1), (-1, -1)))
    front = f"{P}base_y1 + 1"          # cuts run 1 mm out of the front face
    # servo profile swept from behind the servo out through the front face
    box(body, "BodyCavity", True, f"{P}cav_x1 - {P}cav_x0", f"{front} - {P}cav_y0", f"{P}case_z1 - {P}cav_z0",
        x=P + "cav_x0", y=P + "cav_y0", z=P + "cav_z0")
    box(body, "TabSlot", True, P + "tab_l", f"{front} - {P}cav_y0", P + "tab_h",
        x=P + "tab_x0", y=P + "cav_y0", z=P + "tab_z0")
    # housing slot from top_gap below the housing's underside: the housing is wider than the body, so its
    # underside overhangs the case
    cylinder(body, "HousingRound", True, P + "housing_r", f"{ZT} + 1 - {HZ}", P + "housing_cx", "0", HZ)
    box(body, "HousingSlot", True, f"2 * {P}housing_r", front, f"{ZT} + 1 - {HZ}",
        x=f"{P}housing_cx - {P}housing_r", y="0", z=HZ)
    box(body, "CableGroove", True, f"{P}cable_t + 0.5", f"{front} - {P}cav_y0", P + "cable_h",
        x=f"{P}cav_x1 - 0.5", y=P + "cav_y0", z=P + "cav_z0")

    # vertical holes: back row through (6), right end through (y = -8, 0, 8)
    core.pattern(body, "VBack", vertical_hole(body, "VBackh", g(-3), g(-2), ZB, ZT), "X", 6, g(5))
    core.pattern(body, "VEndR", vertical_hole(body, "VEndRh", g(2), g(-1), ZB, ZT), "Y", 3, g(2))
    # left end (y = -8, 0, 8): the tab slot runs through there, so one pin deep from the bottom and 4 mm from
    # the top (1.2 mm above the slot)
    core.pattern(body, "VEndLB", vertical_hole(body, "VEndLBh", g(-3), g(-1), ZB, f"{ZB} + {P}pin_depth",
                                               cb_top=False), "Y", 3, g(2))
    core.pattern(body, "VEndLT", vertical_hole(body, "VEndLTh", g(-3), g(-1), f"{P}tab_z0 + {P}tab_h + 1.2", ZT,
                                               cb_bot=False), "Y", 3, g(2))
    # under the servo: y = 0 one pin deep (opens into the cavity), y = +-8 up to the cavity floor level
    core.pattern(body, "VFloorMid", vertical_hole(body, "VFloorMidh", g(-2), "0", ZB, f"{ZB} + {P}pin_depth",
                                                  cb_top=False), "X", 3, g(2))
    core.pattern(body, "VFloorSide", vertical_hole(body, "VFloorSideh", g(-2), g(-1), ZB, P + "cav_z0",
                                                   cb_top=False), "X", 3, g(2), "Y", 2, g(2))

    # back face (y = -19.7): 4 rows x 6, one pin deep
    for i in range(4):
        core.pattern(body, f"SB{i}", side_hole(body, f"SB{i}h", (1, 0, 0), -90, g(-3), f"-{P}set_w / 2 - 0.5",
                                               zrow(i)), "X", 6, g(5))
    # front face where the base is solid, for the cover pins: x = -24 (3 lower rows), x = 16 (all 4)
    yf = f"{P}base_y1 + 0.5"
    core.pattern(body, "SFL", side_hole(body, "SFLh", (1, 0, 0), 90, g(-3), yf, zrow(0)), "Z", 3, g(2))
    core.pattern(body, "SFR", side_hole(body, "SFRh", (1, 0, 0), 90, g(2), yf, zrow(0)), "Z", 4, g(3))
    # end faces, y = -16 .. 8: three lower rows one pin deep; the top row (level with the tab slot) shallower
    # except at y = -16, behind the slot
    for t, s, x in (("L", -1, f"{X0} - 0.5"), ("R", 1, f"{X1} + 0.5")):
        for i in range(3):
            core.pattern(body, f"E{t}{i}", side_hole(body, f"E{t}{i}h", (0, 1, 0), -90 * s, x, g(-2), zrow(i)),
                         "Y", 4, g(3))
        side_hole(body, f"E{t}3Back", (0, 1, 0), -90 * s, x, g(-2), zrow(3))
        depth = P + ("end_depth" if t == "L" else "end_depth_r")
        core.pattern(body, f"E{t}3", side_hole(body, f"E{t}3h", (0, 1, 0), -90 * s, x, g(-1), zrow(3), depth),
                     "Y", 3, g(2))
    core.finish(doc, body, stem)
    return stem


def build_cover(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "SideCover")
    rounded_box(body, "Cover", X0, P + "cover_y0", ZB, P + "set_l", P + "cover_t", P + "set_h",
                ((1, 1), (-1, 1)))
    # plug filling the cavity's opening, 0.15 inside its walls, from 0.2 in front of the servo to the cover
    F = P + "plug_fit"
    plug_w = f"{P}cover_y0 + 0.1 - {P}plug_y0"
    box(body, "PlugBody", False, f"{P}cav_x1 - {P}cav_x0 - 2 * {F}", plug_w, f"{P}case_z1 - {P}cav_z0 - 2 * {F}",
        x=f"{P}cav_x0 + {F}", y=P + "plug_y0", z=f"{P}cav_z0 + {F}")
    box(body, "PlugTab", False, f"{P}tab_l - 2 * {F}", plug_w, f"{P}tab_h - 2 * {F}",
        x=f"{P}tab_x0 + {F}", y=P + "plug_y0", z=f"{P}tab_z0 + {F}")
    box(body, "PlugHousing", False, f"2 * ({P}housing_r - {F})", plug_w,
        f"{ZT} - ({P}case_z1 - {F} - 0.5)",
        x=f"{P}housing_cx - {P}housing_r + {F}", y=P + "plug_y0", z=f"{P}case_z1 - {F} - 0.5")
    cylinder(body, "HousingClear", True, P + "housing_r", f"{ZT} + 1 - {HZ}", P + "housing_cx", "0", HZ)
    # cable notch, open at the bottom, in line with the base's cable groove
    box(body, "CableNotch", True, P + "cable_t", f"{P}cover_t + 2", f"{P}cav_z0 + {P}cable_h - {ZB} + 1",
        x=P + "cav_x1", y=f"{P}cover_y0 - 1", z=f"{ZB} - 1")

    # through holes across the cover, in line with the base's front-face holes, counterbored on both faces
    yo, yi = f"{P}set_w / 2 + 0.5", f"{P}cover_y0 - 0.5"
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 0.5"
    # (the inner counterbores get their own pattern: one pattern of hole + both counterbores copied only the hole)
    rot_in, rot_out = App.Rotation(App.Vector(1, 0, 0), 90), App.Rotation(App.Vector(1, 0, 0), -90)
    for t, x, n in (("L", g(-3), 3), ("R", g(2), 4)):
        feats = [cylinder(body, f"Y{t}h", True, f"{P}pin_d / 2", f"{P}cover_t + 1", x, yo, zrow(0), rot_in),
                 cylinder(body, f"Y{t}hCbO", True, f"{P}cb_d / 2", cb_h, x, yo, zrow(0), rot_in)]
        core.pattern(body, f"Y{t}", feats, "Z", n, g(n - 1))
        core.pattern(body, f"Y{t}I", [cylinder(body, f"Y{t}hCbI", True, f"{P}cb_d / 2", cb_h, x, yi, zrow(0),
                                               rot_out)], "Z", n, g(n - 1))
    # vertical through holes along the cover (x = 8 left out: cable notch)
    core.pattern(body, "VCover", vertical_hole(body, "VCoverh", g(-3), g(2), ZB, ZT), "X", 4, g(3))
    vertical_hole(body, "VCoverR", g(2), g(2), ZB, ZT)
    # end faces, 4 rows, one pin deep
    for t, s, x in (("L", -1, f"{X0} - 0.5"), ("R", 1, f"{X1} + 0.5")):
        core.pattern(body, f"E{t}", side_hole(body, f"E{t}h", (0, 1, 0), -90 * s, x, g(2), zrow(0)), "Z", 4, g(3))
    core.finish(doc, body, stem)
    return stem


PARTS = {"base": (build_base, "mg90s_side_base"), "cover": (build_cover, "mg90s_side_cover")}
todo = [p for p in os.environ.get("SIDE_PARTS", "base;cover").split(";") if p]
core.gui_resave(*(PARTS[p][0](PARTS[p][1]) for p in todo))
