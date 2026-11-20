"""LEGO-bottom / DUPLO-top adapter bricks as parametric FreeCAD models (rebuild of ../lego4x8_duplo2x4.scad and
../lego4x4_duplo2x2.scad, same parameters).

Run:  freecadcmd.exe build_lego_duplo.py
Outputs lego4x8_duplo2x4.FCStd and lego4x4_duplo2x2.FCStd here, STEP + STL in out/.

A standard LEGO brick bottom (outer wall, top plate, tubes between the studs) carrying hollow DUPLO studs on top:
the DUPLO pitch (16) is twice the LEGO pitch (8), so a 4 x 8 LEGO footprint is exactly 2 x 4 DUPLO.
Frame: brick centre at x = y = 0, z = 0 under the walls. Every size lives in the `Params` spreadsheet; each
feature of the body is bound to it (stud and tube counts follow units_x / units_y, which must be even).
"""
import os
import subprocess
import sys

import FreeCAD as App
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
P = "Params."

PARAMS = [
    ("units_x", "4", "LEGO 寬（凸點數，須為偶數）"),
    ("units_y", "8", "LEGO 長（凸點數，須為偶數）"),
    ("pitch", "8", "LEGO 凸點間距 (mm)"),
    ("play", "0.1", "每側外緣內縮（標準 0.1）(mm)"),
    ("brick_h", "9.6", "標準磚高（不含凸點）(mm)"),
    ("wall", "1.2", "外牆厚 (mm)"),
    ("top_t", "1", "頂板厚 (mm)"),
    ("tube_od", "6.51", "底部管柱外徑 (mm)"),
    ("tube_id", "4.8", "底部管柱內徑 (mm)"),
    ("d_pitch", "16", "DUPLO 凸點間距 (mm)"),
    ("d_stud_od", "9.55", "DUPLO 凸點外徑：官方約 9.35，Bambu X2D 實測 9.55 最剛好 (mm)"),
    ("d_stud_id", "6.7", "DUPLO 凸點內徑（空心，可接 LEGO 2x2 磚底部）(mm)"),
    ("d_stud_h", "4.5", "DUPLO 凸點高 (mm)"),
    ("size_x", "=units_x * pitch - 2 * play", "外寬（自動計算）"),
    ("size_y", "=units_y * pitch - 2 * play", "外長（自動計算）"),
    ("d_nx", "=units_x / 2", "DUPLO 凸點數 x（自動計算）"),
    ("d_ny", "=units_y / 2", "DUPLO 凸點數 y（自動計算）"),
]

BRICKS = {"lego4x8_duplo2x4": (4, 8), "lego4x4_duplo2x2": (4, 4)}


def bind(obj, prop, expr):
    obj.setExpression(prop, expr)


def make_doc(name, ux, uy):
    doc = App.newDocument(name)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "參數說明")
    sheet.set("B1", "數值")
    values = {"units_x": str(ux), "units_y": str(uy)}
    for row, (alias, value, desc) in enumerate(PARAMS, start=2):
        sheet.set(f"A{row}", desc)
        sheet.set(f"B{row}", values.get(alias, value))
        sheet.setAlias(f"B{row}", alias)
    doc.recompute()
    return doc


def place(feat, x="0", y="0", z="0"):
    bind(feat, ".Placement.Base.x", x)
    bind(feat, ".Placement.Base.y", y)
    bind(feat, ".Placement.Base.z", z)
    return feat


def box(body, name, cut, length, width, height, **at):
    feat = body.newObject("PartDesign::SubtractiveBox" if cut else "PartDesign::AdditiveBox", name)
    bind(feat, "Length", length)
    bind(feat, "Width", width)
    bind(feat, "Height", height)
    return place(feat, **at)


def cylinder(body, name, cut, radius, height, **at):
    feat = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(feat, "Radius", radius)
    bind(feat, "Height", height)
    return place(feat, **at)


def grid(body, name, original, nx, ny, step):
    """2D LinearPattern: nx x ny copies `step` apart toward +x / +y from the original."""
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    feat = body.newObject("PartDesign::LinearPattern", name)
    feat.Originals = [original]
    feat.Direction = (origin["X_Axis"], [""])
    feat.Direction2 = (origin["Y_Axis"], [""])
    bind(feat, "Occurrences", nx)
    bind(feat, "Occurrences2", ny)
    bind(feat, "Length", f"({nx} - 1) * {step}")
    bind(feat, "Length2", f"({ny} - 1) * {step}")
    body.Tip = feat
    return feat


def build(stem, ux, uy):
    doc = make_doc(stem, ux, uy)
    body = doc.addObject("PartDesign::Body", "Adapter")
    inner_h = f"{P}brick_h - {P}top_t"
    box(body, "Shell", False, P + "size_x", P + "size_y", P + "brick_h",
        x=f"-{P}size_x / 2", y=f"-{P}size_y / 2")
    box(body, "Cavity", True, f"{P}size_x - 2 * {P}wall", f"{P}size_y - 2 * {P}wall", f"{inner_h} + 1",
        x=f"-{P}size_x / 2 + {P}wall", y=f"-{P}size_y / 2 + {P}wall", z="-1")
    # tubes between the studs: (units_x - 1) x (units_y - 1), the first at (1 - units / 2) * pitch
    tx, ty = f"(1 - {P}units_x / 2) * {P}pitch", f"(1 - {P}units_y / 2) * {P}pitch"
    nx, ny = f"{P}units_x - 1", f"{P}units_y - 1"
    tube = cylinder(body, "Tube", False, f"{P}tube_od / 2", f"{inner_h} + 0.01", x=tx, y=ty)
    grid(body, "Tubes", tube, nx, ny, P + "pitch")
    bore = cylinder(body, "TubeBore", True, f"{P}tube_id / 2", f"{inner_h} + 1", x=tx, y=ty, z="-1")
    grid(body, "TubeBores", bore, nx, ny, P + "pitch")
    # hollow DUPLO studs on top, the first at -(d_n - 1) / 2 * d_pitch
    sx, sy = f"-({P}d_nx - 1) / 2 * {P}d_pitch", f"-({P}d_ny - 1) / 2 * {P}d_pitch"
    stud = cylinder(body, "Stud", False, f"{P}d_stud_od / 2", P + "d_stud_h", x=sx, y=sy, z=P + "brick_h")
    grid(body, "Studs", stud, P + "d_nx", P + "d_ny", P + "d_pitch")
    hole = cylinder(body, "StudHole", True, f"{P}d_stud_id / 2", f"{P}d_stud_h + 1", x=sx, y=sy, z=P + "brick_h")
    grid(body, "StudHoles", hole, P + "d_nx", P + "d_ny", P + "d_pitch")
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


for stem, (ux, uy) in BRICKS.items():
    build(stem, ux, uy)

if not os.environ.get("NO_GUI_RESAVE"):
    log = os.path.join(HERE, "out", "gui_finish.log")
    env = dict(os.environ, GUI_RESAVE=";".join(os.path.join(HERE, f"{s}.FCStd") for s in BRICKS),
               GUI_RESAVE_LOG=log)
    subprocess.run([os.path.join(App.getHomePath(), "bin", "freecad.exe"),
                    os.path.join(HERE, "..", "tt_motor_adapter", "gui_finish.py")], env=env, timeout=1800,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(open(log, encoding="utf-8").read())
sys.stdout.flush()
