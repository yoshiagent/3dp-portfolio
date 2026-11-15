"""Shared pieces of the TT motor adapter models: the core cradle, parameters and export.

Imported by build_tt_adapter.py (cradle only) and build_tt_technic.py (cradle + LEGO Technic interface).
Redesign of the motor cradle of ../TTMotorAdapter.3mf ("20:1 Gearbox for TT Motor" base by Jim Larsen),
with the two gearbox towers removed. Sizes were measured from that mesh.
Every size lives in the `Params` spreadsheet; each feature in the `Adapter` body is bound to it.

Frame: x = 0 at the back face of the front block, motor pocket toward +x;
y = 0 on the centre line; z = 0 under the floor.
"""
import os

import FreeCAD as App
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))

# alias, value or formula, description (column A is shown to the user)
PARAMS = [
    ("length", "51.69", "底板總長 (mm)"),
    ("width", "30.9", "總寬 (mm)"),
    ("floor_t", "2.65", "底板厚 (mm)"),
    ("block_t", "4.99", "前擋塊厚 (mm)"),
    ("block_h", "20", "前擋塊高 (mm)"),
    ("stop_x", "5.46", "馬達前端擋面位置 x (mm)"),
    ("stop_w", "21.22", "擋面凸台寬 (mm)"),
    ("hole_w", "6.1", "前擋塊方孔寬 (mm)"),
    ("hole_h", "3.8", "前擋塊方孔高 (mm)"),
    ("hole_z", "10.1", "前擋塊方孔底邊高度 (mm)"),
    ("wall_t", "4.07", "側牆厚 (mm)"),
    ("inner_w", "=width - 2 * wall_t", "馬達槽內寬（自動計算）"),
    ("rail_h", "8.2", "側牆高（含底板）(mm)"),
    ("fillet_r", "3.65", "擋面兩側內角圓弧半徑 (mm)"),
    ("fillet_y", "7.41", "圓弧在擋面上的起點離中線距離 (mm)"),
    ("fillet_l", "=fillet_r - sqrt(fillet_r ^ 2 - (stop_w / 2 - fillet_y) ^ 2)", "圓弧往後伸出長度（自動計算）"),
    ("post_x", "30.49", "卡扣柱起點 x (mm)"),
    ("post_l", "10.8", "卡扣柱長 (mm)"),
    ("post_h", "24", "卡扣柱高（含底板）(mm)"),
    ("lip_d", "1.74", "卡扣唇往內伸出量 (mm)"),
    ("lip_h", "2", "卡扣唇厚 (mm)"),
    ("slit_x", "25.69", "底板彈片切縫起點 x (mm)"),
    ("slit_y", "7.8", "彈片切縫內緣離中線距離 (mm)"),
    ("slit_w", "0.4", "彈片切縫寬 (mm)"),
    ("tab_x", "45.29", "彈片末端薄區起點 x (mm)"),
    ("tab_w", "13.04", "彈片末端薄區寬 (mm)"),
    ("tab_t", "1.5", "彈片末端薄區厚 (mm)"),
    ("win_x", "47.19", "彈片方窗起點 x (mm)"),
    ("win_l", "2.51", "彈片方窗長 (mm)"),
    ("win_w", "5.5", "彈片方窗寬 (mm)"),
    ("ramp_drop", "0.79", "彈片末端斜面下降量 (mm)"),
    ("ramp_len", "=length - win_x - win_l", "彈片末端斜面長（自動計算）"),
    ("ramp_angle", "=atan(ramp_drop / ramp_len)", "彈片末端斜面角度（自動計算）"),
]


def bind(obj, prop, expr):
    obj.setExpression(prop, expr)


def box(body, name, cut, length, width, height, x="0", y="0", z="0"):
    feat = body.newObject("PartDesign::SubtractiveBox" if cut else "PartDesign::AdditiveBox", name)
    bind(feat, "Length", length)
    bind(feat, "Width", width)
    bind(feat, "Height", height)
    bind(feat, ".Placement.Base.x", x)
    bind(feat, ".Placement.Base.y", y)
    bind(feat, ".Placement.Base.z", z)
    return feat


def pattern(body, name, originals, axis, count, length, axis2=None, count2=1, length2="0"):
    """PartDesign LinearPattern of `originals`: `count` copies spread over `length` (an expression) along
    the body's `axis` ("X"/"Y"/"Z"), optionally also `count2` over `length2` along `axis2` (a 2D grid).
    The originals sit at the first position; the copies go toward +axis.
    Give each separate hole its own pattern, made right after the hole: a LinearPattern copies only the
    first solid of its originals, and a pattern placed after another one fails on originals made before it."""
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    feat = body.newObject("PartDesign::LinearPattern", name)
    feat.Originals = list(originals)
    feat.Direction = (origin[f"{axis}_Axis"], [""])
    feat.Occurrences = count
    bind(feat, "Length", length)
    if axis2:
        feat.Direction2 = (origin[f"{axis2}_Axis"], [""])
        feat.Occurrences2 = count2
        bind(feat, "Length2", length2)
    body.Tip = feat  # newObject does not move the Tip onto a pattern; later features must follow it
    return feat


def make_doc(name, params):
    """New document with a `Params` spreadsheet: column A describes, column B holds the aliased value."""
    doc = App.newDocument(name)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "參數說明")
    sheet.set("B1", "數值")
    for row, (alias, value, desc) in enumerate(params, start=2):
        sheet.set(f"A{row}", desc)
        sheet.set(f"B{row}", value)
        sheet.setAlias(f"B{row}", alias)
    doc.recompute()
    return doc


def build_core(body):
    """The motor cradle measured from TTMotorAdapter.3mf, without the gearbox towers."""
    P = "Params."

    box(body, "Floor", False, P + "length", P + "width", P + "floor_t", y=f"-{P}width / 2")
    box(body, "FrontBlock", False, P + "block_t", P + "width", P + "block_h", y=f"-{P}width / 2")
    box(body, "StopPad", False, P + "stop_x", P + "stop_w", P + "block_h", y=f"-{P}stop_w / 2")

    # side walls, snap posts and their inward lips; side +1 is +y, -1 is -y
    for tag, s in (("L", 1), ("R", -1)):
        wall_y = f"{P}inner_w / 2" if s > 0 else f"-{P}width / 2"
        lip_y = f"{P}inner_w / 2 - {P}lip_d" if s > 0 else f"-{P}inner_w / 2"
        box(body, f"Rail{tag}", False, P + "post_x", P + "wall_t", P + "rail_h", y=wall_y)
        box(body, f"Post{tag}", False, P + "post_l", P + "wall_t", P + "post_h", x=P + "post_x", y=wall_y)
        box(body, f"Lip{tag}", False, P + "post_l", P + "lip_d", P + "lip_h",
            x=P + "post_x", y=lip_y, z=f"{P}post_h - {P}lip_h")

        # concave arc on each side of the stop face: a filler box with a circle taken out
        corner_y = P + "fillet_y" if s > 0 else f"-{P}stop_w / 2"
        box(body, f"Fillet{tag}", False, P + "fillet_l", f"{P}stop_w / 2 - {P}fillet_y",
            f"{P}block_h - {P}floor_t", x=P + "stop_x", y=corner_y, z=P + "floor_t")
        # below the rail top, close the gap between the arc's end and the wall
        foot_y = f"{P}stop_w / 2" if s > 0 else f"-{P}inner_w / 2"
        box(body, f"FilletFoot{tag}", False, f"{P}stop_x + {P}fillet_l - {P}block_t",
            f"({P}inner_w - {P}stop_w) / 2", f"{P}rail_h - {P}floor_t",
            x=P + "block_t", y=foot_y, z=P + "floor_t")
        arc = body.newObject("PartDesign::SubtractiveCylinder", f"FilletArc{tag}")
        bind(arc, "Radius", P + "fillet_r")
        bind(arc, "Height", f"{P}block_h - {P}floor_t + 2")
        bind(arc, ".Placement.Base.x", f"{P}stop_x + {P}fillet_r")
        bind(arc, ".Placement.Base.y", f"{s} * {P}fillet_y")
        bind(arc, ".Placement.Base.z", f"{P}floor_t")

    box(body, "FrontHole", True, f"{P}stop_x + 2", P + "hole_w", P + "hole_h",
        x="-1", y=f"-{P}hole_w / 2", z=P + "hole_z")

    # floor spring tab: two slits, a thinner free end, a window and a ramp under the tip
    for tag, s in (("L", 1), ("R", -1)):
        slit_y = P + "slit_y" if s > 0 else f"-{P}slit_y - {P}slit_w"
        box(body, f"Slit{tag}", True, f"{P}length - {P}slit_x + 1", P + "slit_w", f"{P}floor_t + 2",
            x=P + "slit_x", y=slit_y, z="-1")
    box(body, "TabThin", True, f"{P}length - {P}tab_x + 1", P + "tab_w", f"{P}floor_t - {P}tab_t + 1",
        x=P + "tab_x", y=f"-{P}tab_w / 2", z=P + "tab_t")
    box(body, "TabWindow", True, P + "win_l", P + "win_w", f"{P}floor_t + 2",
        x=P + "win_x", y=f"-{P}win_w / 2", z="-1")
    # tip beyond the window slopes down: a box tilted about y whose bottom face runs
    # from (win end, tab_t) down to (length, tab_t - ramp_drop)
    ramp = box(body, "TabRamp", True, f"{P}ramp_len / cos({P}ramp_angle) + 2", P + "win_w", "2",
               y=f"-{P}win_w / 2")
    ramp.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), 10))
    bind(ramp, ".Placement.Rotation.Angle", P + "ramp_angle")
    bind(ramp, ".Placement.Base.x", f"{P}win_x + {P}win_l")
    bind(ramp, ".Placement.Base.z", P + "tab_t")


def finish(doc, body, stem):
    """Recompute, report, save <stem>.FCStd and export out/<stem>.step + .stl."""
    doc.recompute()
    # A body shows only its Tip; without a GUI, PartDesign leaves every feature hidden, the Tip too.
    for feature in body.Group:
        if feature.TypeId.startswith("PartDesign::"):
            feature.Visibility = feature == body.Tip
    bad = [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
    shape = body.Shape
    print(f"features: {[o.Name for o in body.Group]}")
    print(f"errors: {bad or 'none'}; valid solid: {shape.isValid()}, solids: {len(shape.Solids)}")
    bb = shape.BoundBox
    print(f"volume {shape.Volume:.1f} mm3, bbox {bb.XLength:.2f} x {bb.YLength:.2f} x {bb.ZLength:.2f} mm")
    fcstd = os.path.join(HERE, f"{stem}.FCStd")
    doc.saveAs(fcstd)
    print(f"saved {fcstd}")

    out_dir = os.path.join(HERE, "out")
    os.makedirs(out_dir, exist_ok=True)
    shape.exportStep(os.path.join(out_dir, f"{stem}.step"))
    mesh = Mesh.Mesh(shape.tessellate(0.02))
    mesh.write(os.path.join(out_dir, f"{stem}.stl"))
    print(f"exported STEP + STL to {out_dir}")


def gui_resave(*stems):
    """Re-save <stem>.FCStd through the FreeCAD GUI (gui_finish.py) so the files open with the body
    visible and carry a thumbnail; files saved by freecadcmd open with everything hidden."""
    import subprocess

    if os.environ.get("NO_GUI_RESAVE"):  # quick headless check runs
        return
    paths =[os.path.join(HERE, f"{stem}.FCStd") for stem in stems]
    # one log per run, so several resaves can run side by side
    log = os.path.join(HERE, "out", f"gui_finish_{stems[0]}.log")
    env = dict(os.environ, GUI_RESAVE=";".join(paths), GUI_RESAVE_LOG=log)
    exe = os.path.join(App.getHomePath(), "bin", "freecad.exe")
    # files with several hundred features take a long time to open in the GUI
    subprocess.run([exe, os.path.join(HERE, "gui_finish.py")], env=env, timeout=5400,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(open(log, encoding="utf-8").read())


