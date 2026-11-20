"""MG90S front steering for LEGO Technic: steering frame, two knuckles, tie rod, servo crank (FreeCAD, parametric).
Saves <part>.FCStd here and STEP/STL in out/.

Run:  freecadcmd.exe build_steering.py
      (STEER_PARTS="frame;knuckle_r;knuckle_l;tierod;crank;crank_direct", ';'-separated)

Vehicle coordinates: x to the right, y forward, z up from the ground; the servo output shaft is the z axis.
The MG90S sits in the side-loading holder (../tt_motor_adapter/build_mg90s_side.py), shaft up, its short end
forward, holder bottom 8 above the ground (tab underside z = 31.8). Layout (linkage geometry from
tools/linkage.py, Ackermann for a 160 wheelbase):
  - frame: a bulkhead pinned to the holder's front face (5 x 3 Technic holes) and a 1-unit plate (z 24.2-31.6)
    out to the kingpins at (+-40, 40); Technic holes all over for mounting it in a LEGO chassis
  - knuckles: C-shaped, lower and upper ears around the frame's kingpin boss (3L frictionless pin as kingpin),
    an outboard upright with a round hole for the wheel axle at z = 27.9 (56 mm wheel), and an 18 mm
    steering arm on the upper ear pointing rearward and 19 deg inward
  - tie rods (2, identical): 26.45 between pin holes, at z 40.2-47.6, 2L frictionless pins at both ends
  - crank: sized to the printed double-arm horn (../mg90s_arm, measured from MG90SSovoArm.3mf): the horn's
    plate sits in a recess under the crank, 2 M2 self-tapping screws go down through the crank into the horn's
    4th D1.5 hole each way, a D5 hole over the centre; two tie-rod pin holes at (+-8, 27), the arm relieved
    underneath so the tie rods clear it by 0.6
  - crank_direct (steer_crank_direct, alternative): no horn; the horn's measured 20-tooth spline hole is in the
    crank's hub, which sits straight on the servo shaft, fixed by the servo's own horn screw; same pin holes
Every part is 7.4 thick with 0.6 gaps on an 8 mm pitch, so standard Technic pins join them.
"""
import os
import subprocess
import sys

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
ADAPTER = os.path.join(os.path.dirname(HERE), "tt_motor_adapter")
sys.path.insert(0, ADAPTER)
import tt_adapter_core as core  # noqa: E402
from tt_adapter_core import bind, box  # noqa: E402

core.HERE = HERE  # finish() saves <stem>.FCStd and out/ next to this script

PARAMS = [
    ("pitch", "8", "Technic 孔距 (mm)"),
    ("part_t", "7.4", "零件厚：1 格 (mm)"),
    ("part_gap", "0.6", "疊放間隙 (mm)"),
    ("pin_d", "4.94", "銷孔直徑：軸也能在裡面轉 (mm)"),
    ("cb_d", "6.2", "孔沉頭直徑 (mm)"),
    ("cb_depth", "0.8", "孔沉頭深度 (mm)"),
    ("end_r", "=part_t / 2", "零件圓頭半徑（自動計算）"),
    # layout (tools/linkage.py)
    ("kp_x", "40", "主銷離中心線 x (mm)"),
    ("kp_y", "40", "主銷在伺服軸前方 y (mm)"),
    ("arm_a", "18", "轉向臂長：主銷到拉桿銷 (mm)"),
    ("arm_beta", "19", "轉向臂內傾角：阿克曼，對 160 軸距 (deg)"),
    ("crank_c", "8", "曲柄拉桿銷離中心線 x (mm)"),
    ("crank_j", "27", "曲柄拉桿銷在伺服軸前方 y (mm)"),
    ("rod_len", "=sqrt((kp_x - arm_a * sin(arm_beta) - crank_c) ^ 2 + (kp_y - arm_a * cos(arm_beta) - crank_j) ^ 2)",
     "拉桿兩孔中心距（自動計算）"),
    # heights above the ground
    ("z_low", "16.2", "轉向節下耳底面 z (mm)"),
    ("z_plate", "=z_low + pitch", "車架平板底面 z（自動計算）"),
    ("z_up", "=z_low + 2 * pitch", "轉向節上耳底面 z（自動計算）"),
    ("z_rod", "=z_low + 3 * pitch", "拉桿底面 z（自動計算）"),
    ("wheel_z", "=z_plate + part_t / 2", "輪軸高度：56 mm 輪（自動計算）"),
    # frame
    ("bulk_w", "39.4", "隔板寬：伺服座寬 (mm)"),
    ("bulk_y0", "20.3", "隔板背面 y：伺服座前面 + 0.6 (mm)"),
    ("bulk_z0", "8", "隔板底面 z：伺服座底面 (mm)"),
    ("plate_hw", "27.7", "平板中段半寬 (mm)"),
    # knuckle
    ("body_in", "5.8", "轉向節立柱內面離主銷：留空間給軸端擋片 (mm)"),
    # crank over the printed double-arm horn (../mg90s_arm/mg90s_arm_double, sized after the MG90S's own horn:
    # plate 2 thick, 33.1 long, outline around a D6.88 hub circle and D3.6 end circles 14.75 out, 6 D1.5 holes a
    # side every 1.85 from 5.5 out). On the servo the horn's
    # boss faces down, so its flat plate is on top; the crank's underside recess fits over that plate.
    ("z_tab", "31.8", "舵機耳下緣離地 z (mm)"),
    ("horn_top", "14.5", "舵臂板頂面離舵機耳下緣：軸頂 13.5 − 齒孔深 3 + 凸台 2 + 板 2 (mm)"),
    ("horn_t", "2", "舵臂板厚 (mm)"),
    ("horn_hub_r", "3.44", "舵臂中心圓半徑：原廠雙臂最大寬 6.88 的一半 (mm)"),
    ("horn_end_r", "1.8", "舵臂末端圓半徑 (mm)"),
    ("horn_len", "14.75", "舵臂中心到末端圓心：全長 33.1 / 2 − 1.8 (mm)"),
    ("horn_clear", "0.2", "舵臂凹槽每邊間隙 (mm)"),
    ("recess_h", "=horn_t - 0.1", "凹槽深：比舵臂板薄 0.1，曲柄壓在板頂上（自動計算）"),
    ("crank_wall", "2", "凹槽外的曲柄壁厚 (mm)"),
    ("crank_z0", "=z_tab + horn_top - recess_h", "曲柄底面離地 z（自動計算）"),
    ("crank_relief", "=z_rod + part_t + part_gap - crank_z0", "曲柄臂下方讓位：拉桿在它下面 0.6（自動計算）"),
    ("crank_h", "=crank_relief + part_t", "曲柄總厚：臂頂與拉桿區同高（自動計算）"),
    ("head_d", "5.2", "舵臂中心螺絲頭讓位直徑 (mm)"),
    ("head_h", "2", "舵臂中心螺絲頭讓位高度 (mm)"),
    ("screw_r", "11.05", "固定螺絲：舵臂第 4 孔離中心（5.5 + 3 × 1.85），左右各 1 顆 (mm)"),
    ("screw_d", "2.2", "固定螺絲穿孔直徑：M2 自攻螺絲鎖進舵臂 Ø1.5 孔 (mm)"),
    ("screw_cb_d", "6.2", "固定螺絲頭沉孔直徑 (mm)"),
    ("center_d", "5", "曲柄中心圓孔直徑：可從上面看到／碰到舵臂中心螺絲 (mm)"),
    ("screw_grip", "3", "螺絲頭下方曲柄留的厚度：M2 × 5 剛好穿過舵臂 (mm)"),
    # direct crank (steer_crank_direct): no horn, the 20-tooth spline hole measured from the horn is in the crank
    ("shaft_top", "13.5", "伺服輸出軸頂離舵機耳下緣：照片推估，請實測 (mm)"),
    ("spline_depth", "3", "齒孔深：與舵臂相同 (mm)"),
    ("spline_teeth", "20", "齒數：SG90 / MG90S 20 齒"),
    ("spline_ro", "2.55", "齒孔外半徑（齒槽底），量自舵臂 (mm)"),
    ("spline_ri", "2.009", "齒孔內半徑（齒尖），量自舵臂 (mm)"),
    ("tooth_tip_deg", "0.853", "齒尖半角寬 (deg)"),
    ("tooth_base_deg", "5.022", "齒根半角寬（在外半徑上）(deg)"),
    ("spline_clear", "0", "齒孔每邊放大量：舵臂原檔是緊配，太緊時調大 (mm)"),
    ("tri", "=spline_ri + spline_clear", "齒尖半徑含間隙（自動計算）"),
    ("tro", "=spline_ro + spline_clear", "齒槽底半徑含間隙（自動計算）"),
    ("d_hub_d", "10", "直接版：下方軸轂直徑 (mm)"),
    ("d_z0", "=z_tab + shaft_top - spline_depth", "直接版：曲柄底面離地 z，軸頂頂到齒孔底（自動計算）"),
    ("d_relief", "=z_rod + part_t + part_gap - d_z0", "直接版：軸轂高＝曲柄臂下方讓位（自動計算）"),
    ("d_h", "=d_relief + part_t", "直接版：曲柄總高（自動計算）"),
    ("d_screw_d", "2.5", "直接版：中心螺絲孔直徑（伺服機附的舵盤螺絲）(mm)"),
    ("d_head_d", "5", "直接版：中心圓孔直徑（舵盤螺絲頭從這裡放進去，頭要小於 5）(mm)"),
    ("d_floor", "2", "直接版：齒孔底到螺絲頭之間留的厚度 (mm)"),
]

P = "Params."
CB_OVERLAP = 0.02
T, R = f"{P}part_t", f"{P}end_r"


def g(k):
    return f"{k} * {P}pitch"


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
    """Vertical pin hole z_bot..z_top (open ends run 1 mm past), counterbored at the chosen ends."""
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    feats = [cylinder(body, name, True, f"{P}pin_d / 2", f"({z_top}) - ({z_bot}) + 2", x, y, f"({z_bot}) - 1")]
    if cb_bot:
        feats.append(cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x, y, f"({z_bot}) - 1"))
    if cb_top:
        feats.append(cylinder(body, f"{name}CbT", True, f"{P}cb_d / 2", cb_h, x, y,
                              f"({z_top}) - {P}cb_depth - {CB_OVERLAP}"))
    return feats


def y_hole(body, name, x, z, y0, y1, far_cb=True):
    """Pin hole along y from y0 to y1 (both faces open), counterbored at y0 and (far_cb) at y1."""
    rot = App.Rotation(App.Vector(1, 0, 0), -90)   # cylinder +z -> +y
    cb_h = f"{P}cb_depth + {CB_OVERLAP} + 1"
    feats = [cylinder(body, name, True, f"{P}pin_d / 2", f"({y1}) - ({y0}) + 2", x, f"({y0}) - 1", z, rot),
             cylinder(body, f"{name}CbA", True, f"{P}cb_d / 2", cb_h, x, f"({y0}) - 1", z, rot)]
    if far_cb:
        feats.append(cylinder(body, f"{name}CbB", True, f"{P}cb_d / 2", cb_h, x,
                              f"({y1}) - {P}cb_depth - {CB_OVERLAP}", z, rot))
    return feats


def link_bar(body, name, x0, y0, length, angle, z0):
    """Additive bar of width part_t from (x0, y0) along `angle` (deg expression), `length` long, round ends."""
    bar = box(body, name, False, length, T, T)
    bar.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), 10))
    bind(bar, ".Placement.Rotation.Angle", angle)
    # box corner = start point offset by half the width, perpendicular to the bar
    bind(bar, ".Placement.Base.x", f"({x0}) + {R} * sin({angle})")
    bind(bar, ".Placement.Base.y", f"({y0}) - {R} * cos({angle})")
    bind(bar, ".Placement.Base.z", z0)
    cylinder(body, f"{name}End0", False, R, T, x0, y0, z0)
    cylinder(body, f"{name}End1", False, R, T, f"({x0}) + ({length}) * cos({angle})",
             f"({y0}) + ({length}) * sin({angle})", z0)
    return bar


# ---------------------------------------------------------------- frame
def build_frame(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "SteerFrame")
    zp0, zp1 = f"{P}z_plate", f"{P}z_plate + {T}"
    y0, y1 = P + "bulk_y0", f"{P}kp_y + {R}"
    # bulkhead pinned to the holder's front face, plate out to the kingpins
    box(body, "Bulkhead", False, P + "bulk_w", T, f"{zp1} - {P}bulk_z0",
        x=f"-{P}bulk_w / 2", y=y0, z=P + "bulk_z0")
    box(body, "Plate", False, f"2 * {P}plate_hw", f"{y1} - {y0}", T, x=f"-{P}plate_hw", y=y0, z=zp0)
    for t, s in (("R", 1), ("L", -1)):
        box(body, f"Arm{t}", False, f"{P}kp_x - {P}plate_hw", T, T,
            x=P + "plate_hw" if s > 0 else f"-{P}kp_x", y=f"{P}kp_y - {R}", z=zp0)
        cylinder(body, f"Boss{t}", False, R, T, f"{s} * {P}kp_x", P + "kp_y", zp0)
    # holes along y: rows 11.9, 19.9 through the bulkhead, row 27.9 on through the plate to its front edge
    for i, zrow in enumerate((f"{P}bulk_z0 + {P}pitch / 2 - 0.1", f"{P}bulk_z0 + 1.5 * {P}pitch - 0.1",
                              P + "wheel_z")):
        y_end = y1 if i == 2 else f"{y0} + {T}"
        # hole + back counterbore in one pattern, the front counterbore in another: a pattern of a tipped hole
        # with both counterbores copied nothing
        core.pattern(body, f"Y{i}", y_hole(body, f"Y{i}h", g(-2), zrow, y0, y_end, far_cb=False), "X", 5, g(4))
        rot = App.Rotation(App.Vector(1, 0, 0), -90)
        cb_b = cylinder(body, f"Y{i}hCbB", True, f"{P}cb_d / 2", f"{P}cb_depth + {CB_OVERLAP} + 1", g(-2),
                        f"({y_end}) - {P}cb_depth - {CB_OVERLAP}", zrow, rot)
        core.pattern(body, f"Y{i}B", [cb_b], "X", 5, g(4))
    # vertical holes: y = 24 through bulkhead and plate (x -16..16), plate only elsewhere
    core.pattern(body, "VBulk", vertical_hole(body, "VBulkh", g(-2), g(3), P + "bulk_z0", zp1), "X", 5, g(4))
    for t, x in (("L", g(-3)), ("R", g(3))):
        vertical_hole(body, f"VSide{t}", x, g(3), zp0, zp1)
    core.pattern(body, "VPlate", vertical_hole(body, "VPlateh", g(-3), g(4), zp0, zp1), "X", 7, g(6), "Y", 2, g(1))
    for t, s in (("R", 1), ("L", -1)):
        vertical_hole(body, f"VArm{t}", f"{s} * 4 * {P}pitch", P + "kp_y", zp0, zp1)
        vertical_hole(body, f"Kingpin{t}", f"{s} * {P}kp_x", P + "kp_y", zp0, zp1)
    core.finish(doc, body, stem)
    return stem


# ---------------------------------------------------------------- knuckle
def build_knuckle(stem, s):
    """Knuckle for side s (+1 right, -1 left), kingpin on the z axis (place it at (s * kp_x, kp_y))."""
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "Knuckle")
    zl, zu = P + "z_low", P + "z_up"
    out = f"{P}body_in + {T}"
    for tag, z in (("Low", zl), ("Up", zu)):
        box(body, f"Ear{tag}", False, out, T, T, x="0" if s > 0 else f"-({out})", y=f"-{R}", z=z)
        cylinder(body, f"EarEnd{tag}", False, R, T, "0", "0", z)
    box(body, "Upright", False, T, T, f"{zu} + {T} - {zl}",
        x=P + "body_in" if s > 0 else f"-({out})", y=f"-{R}", z=zl)
    # steering arm on the upper ear: rearward, turned inward by arm_beta
    angle = f"(270 - {P}arm_beta)" if s > 0 else f"(270 + {P}arm_beta)"   # positive: FreeCAD keeps the z axis
    link_bar(body, "SteerArm", "0", "0", P + "arm_a", angle, zu)
    ax, ay = f"{P}arm_a * cos({angle})", f"{P}arm_a * sin({angle})"
    vertical_hole(body, "ArmPin", ax, ay, zu, f"{zu} + {T}")
    # kingpin through both ears, counterbored outside
    vertical_hole(body, "Kingpin", "0", "0", zl, f"{zu} + {T}")
    # wheel axle: round hole across the upright at wheel height (the axle turns in it)
    rot = App.Rotation(App.Vector(0, 1, 0), 90 * s)   # cylinder +z -> +-x
    cylinder(body, "Axle", True, f"{P}pin_d / 2", f"{T} + 2", f"{s} * ({P}body_in - 1)", "0", P + "wheel_z", rot)
    core.finish(doc, body, stem)
    return stem


# ---------------------------------------------------------------- tie rod
def build_tierod(stem):
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "TieRod")
    link_bar(body, "Rod", "0", "0", P + "rod_len", "0", "0")
    vertical_hole(body, "PinA", "0", "0", "0", T)
    vertical_hole(body, "PinB", P + "rod_len", "0", "0", T)
    core.finish(doc, body, stem)
    return stem


# ---------------------------------------------------------------- crank
def horn_outline(body, name, cut, grow, z0, h):
    """The double-arm horn's plate outline grown by `grow` (an expression), z0..z0 + h: the hub circle plus,
    each way along x, the end circle and the tangent web between them (a PartDesign wedge)."""
    L = P + "horn_len"
    r0, r1 = f"({P}horn_hub_r + {grow})", f"({P}horn_end_r + {grow})"
    cylinder(body, f"{name}Hub", cut, r0, h, "0", "0", z0)
    s = f"({r0} - {r1}) / {L}"  # sine of the tangent lines' tilt
    k = f"sqrt(1 - ({s}) ^ 2)"
    for tag, sign in (("R", 1), ("L", -1)):
        cylinder(body, f"{name}End{tag}", cut, r1, h, f"{sign} * {L}", "0", z0)
        web = body.newObject("PartDesign::SubtractiveWedge" if cut else "PartDesign::AdditiveWedge", f"{name}Web{tag}")
        for prop, expr in (("Ymin", f"{r0} * {s}"), ("Ymax", f"{L} + {r1} * {s}"),
                           ("Xmin", f"-{r0} * {k}"), ("Xmax", f"{r0} * {k}"),
                           ("X2min", f"-{r1} * {k}"), ("X2max", f"{r1} * {k}"),
                           ("Zmin", z0), ("Zmax", f"({z0}) + ({h})"), ("Z2min", z0), ("Z2max", f"({z0}) + ({h})")):
            bind(web, prop, expr)
        # the wedge runs along its local y: turn that onto +x / -x
        web.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), -90 * sign))


def build_crank(stem):
    """Servo crank over the printed double-arm horn: z = 0 is the crank's underside (crank_z0 above the ground),
    the horn's long axis along x, the tie-rod arm toward +y."""
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "SteerCrank")
    H, rel = P + "crank_h", P + "crank_relief"
    # body: the horn outline grown by the clearance and a wall, full height
    horn_outline(body, "Body", False, f"{P}horn_clear + {P}crank_wall", "0", H)
    box(body, "CrankArm", False, f"2 * {P}crank_c", f"{P}crank_j", f"{H} - {rel}",
        x=f"-{P}crank_c", y="0", z=rel)
    box(body, "CrankWeb", False, f"2 * ({P}crank_c + {R})", f"{P}crank_j - {P}horn_hub_r", f"{H} - {rel}",
        x=f"-({P}crank_c + {R})", y=P + "horn_hub_r", z=rel)
    for t, s in (("R", 1), ("L", -1)):
        cylinder(body, f"PinBoss{t}", False, R, f"{H} - {rel}", f"{s} * {P}crank_c", P + "crank_j", rel)
    # recess underneath for the horn plate, 0.1 shallower than the plate so the crank bears on its top
    horn_outline(body, "Recess", True, P + "horn_clear", "-1", f"{P}recess_h + 1")
    cylinder(body, "HeadPocket", True, f"{P}head_d / 2", f"{P}recess_h + {P}head_h + 0.5", "0", "0", "-0.5")
    # round hole through the centre, over the horn's screw
    cylinder(body, "CenterHole", True, f"{P}center_d / 2", f"{H} + 1", "0", "0", "0")
    # 2 M2 self-tapping screws from the top into the horn's 4th hole each way; deep counterbores leave
    # screw_grip of crank under the heads, so M2 x 5 just passes through the horn plate
    for i, x in enumerate((f"{P}screw_r", f"-{P}screw_r"), start=1):
        cylinder(body, f"Screw{i}", True, f"{P}screw_d / 2", f"{H} + 2", x, "0", "-1")
        cylinder(body, f"Screw{i}Cb", True, f"{P}screw_cb_d / 2", f"{H} - {P}recess_h - {P}screw_grip + 1", x, "0",
                 f"{P}recess_h + {P}screw_grip")
    for t, s in (("R", 1), ("L", -1)):
        vertical_hole(body, f"RodPin{t}", f"{s} * {P}crank_c", P + "crank_j", rel, H)
    core.finish(doc, body, stem)
    return stem


def build_crank_direct(stem):
    """Crank fixed straight onto the servo spline, no horn: z = 0 is its underside (d_z0 above the ground), where
    the servo shaft's top bottoms in the spline hole; the tie-rod arm toward +y at the same height as the horn
    version. The servo's own horn screw goes down through the centre into the shaft."""
    doc = core.make_doc(stem, PARAMS)
    body = doc.addObject("PartDesign::Body", "SteerCrankDirect")
    H, rel = P + "d_h", P + "d_relief"
    hub = f"{P}crank_c + {R}"
    cylinder(body, "Hub", False, f"{P}d_hub_d / 2", H, "0", "0", "0")
    cylinder(body, "Disc", False, hub, f"{H} - {rel}", "0", "0", rel)
    box(body, "CrankArm", False, f"2 * {P}crank_c", f"{P}crank_j", f"{H} - {rel}",
        x=f"-{P}crank_c", y="0", z=rel)
    box(body, "CrankWeb", False, f"2 * ({hub})", P + "crank_j", f"{H} - {rel}", x=f"-({hub})", y="0", z=rel)
    for t, s in (("R", 1), ("L", -1)):
        cylinder(body, f"PinBoss{t}", False, R, f"{H} - {rel}", f"{s} * {P}crank_c", P + "crank_j", rel)
    # spline hole from the bottom: a bore at the tooth-space radius with the teeth added back (straight flanks
    # from the tip, r tri, to the root, r tro), as in ../mg90s_arm/build_mg90s_arms.py
    cylinder(body, "SplineBore", True, P + "tro", f"{P}spline_depth + 1", "0", "0", "-1")
    x1, y1 = f"{P}tri * sin({P}tooth_tip_deg)", f"{P}tri * cos({P}tooth_tip_deg)"
    x2, y2 = f"{P}tro * sin({P}tooth_base_deg)", f"{P}tro * cos({P}tooth_base_deg)"
    ext = "0.3"  # flanks carried on past the root radius so each tooth joins the hub
    tooth = body.newObject("PartDesign::AdditiveWedge", "Tooth")
    x2e = f"{x2} + ({x2} - {x1}) / ({y2} - {y1}) * {ext}"
    for prop, expr in (("Ymin", y1), ("Ymax", f"{y2} + {ext}"), ("Xmin", f"-{x1}"), ("Xmax", x1),
                       ("X2min", f"-({x2e})"), ("X2max", x2e),
                       ("Zmin", "0"), ("Zmax", P + "spline_depth"), ("Z2min", "0"), ("Z2max", P + "spline_depth")):
        bind(tooth, prop, expr)
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    teeth = body.newObject("PartDesign::PolarPattern", "Teeth")
    teeth.Originals = [tooth]
    teeth.Axis = (origin["Z_Axis"], [""])
    bind(teeth, "Occurrences", P + "spline_teeth")
    teeth.Angle = 360
    body.Tip = teeth
    # centre screw: through hole above the spline, head counterbore from the top leaving d_floor
    cylinder(body, "ScrewHole", True, f"{P}d_screw_d / 2", f"{H} + 2", "0", "0", "-1")
    cylinder(body, "ScrewHead", True, f"{P}d_head_d / 2", f"{H} - {P}spline_depth - {P}d_floor + 1", "0", "0",
             f"{P}spline_depth + {P}d_floor")
    for t, s in (("R", 1), ("L", -1)):
        vertical_hole(body, f"RodPin{t}", f"{s} * {P}crank_c", P + "crank_j", rel, H)
    core.finish(doc, body, stem)
    return stem


def gui_resave(stems):
    if os.environ.get("NO_GUI_RESAVE"):
        return
    paths = [os.path.join(HERE, f"{stem}.FCStd") for stem in stems]
    log = os.path.join(HERE, "out", f"gui_finish_{stems[0]}.log")
    env = dict(os.environ, GUI_RESAVE=";".join(paths), GUI_RESAVE_LOG=log)
    exe = os.path.join(App.getHomePath(), "bin", "freecad.exe")
    subprocess.run([exe, os.path.join(ADAPTER, "gui_finish.py")], env=env, timeout=5400,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(open(log, encoding="utf-8").read())


PARTS = {
    "frame": lambda: build_frame("steer_frame"),
    "knuckle_r": lambda: build_knuckle("steer_knuckle_right", 1),
    "knuckle_l": lambda: build_knuckle("steer_knuckle_left", -1),
    "tierod": lambda: build_tierod("steer_tierod"),
    "crank": lambda: build_crank("steer_crank"),
    "crank_direct": lambda: build_crank_direct("steer_crank_direct"),
}
todo = [p for p in os.environ.get("STEER_PARTS", ";".join(PARTS)).split(";") if p]
gui_resave([PARTS[p]() for p in todo])
