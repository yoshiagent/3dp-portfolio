"""Rebuild the three servo horns of ../MG90SSovoArm.3mf as parametric FreeCAD models.

Run:  freecadcmd.exe build_mg90s_arms.py
Outputs mg90s_arm_single.FCStd, mg90s_arm_double.FCStd, mg90s_arm_cross.FCStd, each with STEP + STL in out/.

Source: MakerWorld "SovoArm" horns for SG90 / MG90S (SingleArm, DoubleArm, CrossArm). Every size was measured
from that mesh (tmp/sections.py, tmp/outline.py):
  plate 2 mm thick; each arm is the outline around a Ø7.5 circle on the hub and a Ø5 end circle 15.5 mm out
  (the cross arm's short arms: straight, 4 wide, Ø4 ends 7.5 mm out); hub boss Ø7.5 x 2 mm on top;
  20-tooth spline hole 3 mm deep from the top (outer r 2.55, tooth tips r 2.009); Ø2.5 screw hole below it;
  Ø1.5 holes every 2.5 mm from 6.25 mm out (short arms from 5.25 mm).
The double arm is no longer the SovoArm one: it is resized after the MG90S's own double horn (see OVERRIDES):
  33.1 long, hub Ø6.88 (its widest), Ø3.6 ends, 6 holes a side from 5.5 mm out, the last on the end circle's centre.
Frame: hub centre at x = y = 0, z = 0 under the plate (the print bed); the single arm points to -x as in the 3MF.
Every size lives in the `Params` spreadsheet; each feature of the body is bound to it.
"""
import os
import subprocess
import sys

import FreeCAD as App
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
P = "Params."

PARAMS = [
    ("plate_t", "2", "臂板厚 (mm)"),
    ("hub_d", "7.5", "中心凸台直徑，也是臂板繞中心的圓 (mm)"),
    ("hub_h", "2", "中心凸台高出臂板 (mm)"),
    ("arm_len", "15.5", "長臂：中心到末端圓心距離 (mm)"),
    ("end_d", "5", "長臂末端圓直徑 (mm)"),
    ("hole_d", "1.5", "臂上小孔直徑 (mm)"),
    ("hole_pitch", "2.5", "臂上小孔間距 (mm)"),
    ("hole_first", "6.25", "長臂第一個小孔離中心 (mm)"),
    ("hole_n", "5", "長臂每邊小孔數"),
    ("screw_d", "2.5", "中心螺絲孔直徑（在齒孔下方）(mm)"),
    ("spline_depth", "3", "齒孔深度（從凸台頂往下）(mm)"),
    ("spline_teeth", "20", "齒數：SG90 / MG90S 20 齒"),
    ("spline_ro", "2.55", "齒孔外半徑（齒槽底）(mm)"),
    ("spline_ri", "2.009", "齒孔內半徑（齒尖）(mm)"),
    ("tooth_tip_deg", "0.853", "齒尖半角寬 (deg)"),
    ("tooth_base_deg", "5.022", "齒根半角寬（在外半徑上）(deg)"),
    ("spline_clear", "0", "齒孔每邊放大量：原檔是緊配，太緊時調大 (mm)"),
    ("short_len", "7.5", "十字臂短臂：中心到末端圓心距離 (mm)"),
    ("short_d", "4", "十字臂短臂寬（末端圓直徑）(mm)"),
    ("short_hole_first", "5.25", "短臂第一個小孔離中心 (mm)"),
    ("short_hole_n", "2", "短臂每邊小孔數"),
    ("top", "=plate_t + hub_h", "總高（自動計算）"),
    ("tri", "=spline_ri + spline_clear", "齒尖半徑含間隙（自動計算）"),
    ("tro", "=spline_ro + spline_clear", "齒槽底半徑含間隙（自動計算）"),
]

# arms of each horn: (name, angle deg, root radius, end radius, length, first hole, hole count)
LONG = (f"{P}hub_d / 2", f"{P}end_d / 2", P + "arm_len", P + "hole_first", P + "hole_n")
SHORT = (f"{P}short_d / 2", f"{P}short_d / 2", P + "short_len", P + "short_hole_first", P + "short_hole_n")
HORNS = {
    "mg90s_arm_single": [("Arm", 180, *LONG)],
    "mg90s_arm_double": [("ArmA", 0, *LONG), ("ArmB", 180, *LONG)],
    "mg90s_arm_cross": [("ArmA", 0, *LONG), ("ArmB", 180, *LONG), ("ArmC", 90, *SHORT), ("ArmD", 270, *SHORT)],
}


def bind(obj, prop, expr):
    obj.setExpression(prop, expr)


# per-horn changes to PARAMS (same alias replaces the row, a new alias is added at the end).
# The double arm is resized after the horn that comes with the MG90S (black, user's photo, 2026-11-20): 33.1 long,
# 6.88 at its widest (measured); end width, hole count and first hole estimated from the photo
OVERRIDES = {
    "mg90s_arm_double": [
        ("hub_d", "6.88", "中心凸台直徑＝最大寬：MG90S 原廠雙臂實測 (mm)"),
        ("end_d", "3.6", "末端圓直徑：照原廠雙臂照片估計，請實測 (mm)"),
        ("arm_len", "=total_len / 2 - end_d / 2", "中心到末端圓心距離（自動計算）"),
        ("hole_first", "5.5", "第一個小孔離中心：照片估計 (mm)"),
        ("hole_n", "6", "每邊小孔數：與原廠相同"),
        ("hole_pitch", "=(arm_len - hole_first) / (hole_n - 1)", "小孔間距：最後一孔落在末端圓心（自動計算）"),
        ("total_len", "33.1", "全長：MG90S 原廠雙臂實測 (mm)"),
    ],
}


def make_doc(name):
    doc = App.newDocument(name)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "參數說明")
    sheet.set("B1", "數值")
    rows = list(PARAMS)
    for alias, value, desc in OVERRIDES.get(name, []):
        hit = [i for i, r in enumerate(rows) if r[0] == alias]
        if hit:
            rows[hit[0]] = (alias, value, desc)
        else:
            rows.append((alias, value, desc))
    for row, (alias, value, desc) in enumerate(rows, start=2):
        sheet.set(f"A{row}", desc)
        sheet.set(f"B{row}", value)
        sheet.setAlias(f"B{row}", alias)
    doc.recompute()
    return doc


def cylinder(body, name, cut, radius, height, x="0", y="0", z="0"):
    feat = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(feat, "Radius", radius)
    bind(feat, "Height", height)
    bind(feat, ".Placement.Base.x", x)
    bind(feat, ".Placement.Base.y", y)
    bind(feat, ".Placement.Base.z", z)
    return feat


def wedge(body, name, angle, y0, y1, half0, half1, z0, z1):
    """Additive trapezoid prism: along direction `angle` (deg, from +x) from y0 to y1 out from the hub axis,
    half width half0 at y0 and half1 at y1, between heights z0 and z1. All sizes are expressions."""
    feat = body.newObject("PartDesign::AdditiveWedge", name)
    for prop, expr in (("Ymin", y0), ("Ymax", y1), ("Xmin", f"-({half0})"), ("Xmax", half0),
                       ("X2min", f"-({half1})"), ("X2max", half1),
                       ("Zmin", z0), ("Zmax", z1), ("Z2min", z0), ("Z2max", z1)):
        bind(feat, prop, expr)
    # the wedge runs along its local y; turn local y onto the arm direction
    feat.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), angle - 90))
    return feat


def pattern(body, name, original, angle, count, step):
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    feat = body.newObject("PartDesign::LinearPattern", name)
    feat.Originals = [original]
    feat.Direction = (origin["X_Axis" if angle in (0, 180) else "Y_Axis"], [""])
    feat.Reversed = angle in (180, 270)
    bind(feat, "Occurrences", count)
    bind(feat, "Length", f"({count} - 1) * {step}")
    body.Tip = feat
    return feat


def arm(body, name, angle, r0, r1, length, first, count):
    """Plate outline around a circle r0 on the hub and a circle r1 `length` out, plus its row of holes."""
    c = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}[angle]
    cylinder(body, f"{name}End", False, r1, P + "plate_t", x=f"{c[0]} * {length}", y=f"{c[1]} * {length}")
    # tangent lines of the two circles: sin a = (r0 - r1) / length
    s = f"(({r0}) - ({r1})) / {length}"
    k = f"sqrt(1 - ({s}) ^ 2)"
    wedge(body, f"{name}Web", angle, f"({r0}) * {s}", f"{length} + ({r1}) * {s}", f"({r0}) * {k}",
          f"({r1}) * {k}", "0", P + "plate_t")
    return c, first, count


def holes(body, name, angle, c, first, count):
    hole = cylinder(body, f"{name}Hole", True, f"{P}hole_d / 2", f"{P}plate_t + 2",
                    x=f"{c[0]} * {first}", y=f"{c[1]} * {first}", z="-1")
    pattern(body, f"{name}Holes", hole, angle, count, P + "hole_pitch")


def spline(body):
    """20-tooth spline hole from the top: a round bore at the tooth-space radius with the hub's teeth added
    back (straight flanks from the tip, r tri, to the root, r tro), plus the screw hole below it."""
    cylinder(body, "ScrewHole", True, f"{P}screw_d / 2", f"{P}top + 2", z="-1")
    cylinder(body, "SplineBore", True, P + "tro", f"{P}spline_depth + 1", z=f"{P}top - {P}spline_depth")
    x1, y1 = f"{P}tri * sin({P}tooth_tip_deg)", f"{P}tri * cos({P}tooth_tip_deg)"
    x2, y2 = f"{P}tro * sin({P}tooth_base_deg)", f"{P}tro * cos({P}tooth_base_deg)"
    ext = "0.3"  # flanks carried on past the root radius so each tooth joins the hub
    tooth = wedge(body, "Tooth", 90, y1, f"{y2} + {ext}", x1, f"{x2} + ({x2} - {x1}) / ({y2} - {y1}) * {ext}",
                  f"{P}top - {P}spline_depth", P + "top")
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    feat = body.newObject("PartDesign::PolarPattern", "Teeth")
    feat.Originals = [tooth]
    feat.Axis = (origin["Z_Axis"], [""])
    bind(feat, "Occurrences", P + "spline_teeth")
    feat.Angle = 360
    body.Tip = feat


def build(stem, arms):
    doc = make_doc(stem)
    body = doc.addObject("PartDesign::Body", "Horn")
    cylinder(body, "HubPlate", False, f"{P}hub_d / 2", P + "plate_t")
    rows = [(name, angle, *arm(body, name, angle, *rest)) for name, angle, *rest in arms]
    cylinder(body, "HubBoss", False, f"{P}hub_d / 2", P + "hub_h", z=P + "plate_t")
    for name, angle, c, first, count in rows:
        holes(body, name, angle, c, first, count)
    spline(body)
    finish(doc, body, stem)


def finish(doc, body, stem):
    doc.recompute()
    for feature in body.Group:
        if feature.TypeId.startswith("PartDesign::"):
            feature.Visibility = feature == body.Tip
    bad = [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
    shape = body.Shape
    bb = shape.BoundBox
    print(f"{stem}: errors {bad or 'none'}; valid {shape.isValid()}, solids {len(shape.Solids)}, "
          f"volume {shape.Volume:.2f} mm3, bbox {bb.XLength:.3f} x {bb.YLength:.3f} x {bb.ZLength:.3f} mm")
    doc.saveAs(os.path.join(HERE, f"{stem}.FCStd"))
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    shape.exportStep(os.path.join(out, f"{stem}.step"))
    Mesh.Mesh(shape.tessellate(0.01)).write(os.path.join(out, f"{stem}.stl"))


for stem, arms in HORNS.items():
    build(stem, arms)

# re-save through the GUI so the files open with the body visible and carry a thumbnail
if not os.environ.get("NO_GUI_RESAVE"):
    log = os.path.join(HERE, "out", "gui_finish.log")
    env = dict(os.environ, GUI_RESAVE=";".join(os.path.join(HERE, f"{s}.FCStd") for s in HORNS),
               GUI_RESAVE_LOG=log)
    subprocess.run([os.path.join(App.getHomePath(), "bin", "freecad.exe"),
                    os.path.join(HERE, "..", "tt_motor_adapter", "gui_finish.py")], env=env, timeout=1800,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(open(log, encoding="utf-8").read())
sys.stdout.flush()
