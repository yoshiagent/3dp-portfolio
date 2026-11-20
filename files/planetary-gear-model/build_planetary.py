"""Planetary gear teaching model v2.1 as one parametric FreeCAD document (rebuild of
../planetary_gear_model_v2_1.scad, same parameters).

Run:  freecadcmd.exe build_planetary.py
Outputs planetary_gear_v21.FCStd here (every part a PartDesign body in its assembled place, the 2nd / 3rd planet
and pin as links), and each part on its own, in print-ready local coordinates, as out/<part>.step + .stl.

Parts: sun 24 T, planet 18 T (x3), ring 60 T, carrier, base, crank, pin (x3), D shaft; module 1.25, 20 deg.
The gears are FreeCAD's InvoluteGear profiles, padded. The scad thins every tooth by `backlash` at the pitch
circle with the same involute; here that is a profile shift x = -backlash / (2 m tan(pa)) (the ring's internal
teeth: +x) with the addendum / dedendum coefficients compensated so tip and root radii stay put, and no root
fillet, which matches the scad outline to 0.001 mm on the flanks.
Every size lives in the `Params` spreadsheet; each feature, gear and part placement is bound to it, so changing
the module, a tooth count or the backlash rebuilds the whole set (keep ring_teeth = sun + 2 x planet and
(sun + ring) / 3 a whole number for three planets at 120 deg).
"""
import os
import subprocess
import sys

import FreeCAD as App
import Mesh
from PartDesign import InvoluteGearFeature

HERE = os.path.dirname(os.path.abspath(__file__))
STEM = "planetary_gear_v21"
P = "Params."

PARAMS = [
    # gears
    ("module", "1.25", "模數 (mm)"),
    ("pressure_angle", "20", "壓力角 (deg)"),
    ("sun_teeth", "24", "太陽齒輪齒數"),
    ("planet_teeth", "18", "行星齒輪齒數"),
    ("ring_teeth", "=sun_teeth + 2 * planet_teeth", "內齒圈齒數（自動計算，60）"),
    ("add_c", "1", "齒頂高係數（× 模數）"),
    ("ded_c", "1.25", "齒根高係數（× 模數）"),
    ("backlash", "0.15", "每個齒輪在節圓上的齒厚減薄量；齒輪卡住時加大 (mm)"),
    ("gear_thickness", "6", "齒輪厚 (mm)"),
    ("axial_gap", "0.5", "齒輪面與底座／行星架之間的止推墊高 (mm)"),
    ("shift", "=-backlash / (2 * module * tan(pressure_angle))", "外齒輪移位係數：用來做出齒隙（自動計算）"),
    # shafts and bores
    ("shaft_d", "5", "D 形軸直徑 (mm)"),
    ("shaft_flat", "0.5", "D 形軸削平深度 (mm)"),
    ("key_clear", "0.15", "D 形孔（太陽齒輪、搖柄）每邊間隙 (mm)"),
    ("base_bearing_d", "=shaft_d + 0.4", "底座軸承孔直徑（自動計算）"),
    ("carrier_bore_d", "=shaft_d + 0.8", "行星架中心孔直徑（自動計算）"),
    ("pin_d", "4", "行星軸銷直徑 (mm)"),
    ("planet_bore_d", "=pin_d + 0.4", "行星齒輪軸孔直徑（自動計算）"),
    ("carrier_pin_hole_d", "=pin_d + 0.1", "行星架銷孔直徑（自動計算）"),
    ("pin_head_d", "7", "軸銷頭直徑 (mm)"),
    ("pin_head_h", "2", "軸銷頭厚 (mm)"),
    ("sun_pad_r", "4.5", "底座中心止推墊半徑（太陽齒輪下方）(mm)"),
    ("track_half_w", "2", "底座環形止推軌半寬（行星齒輪下方，軌道 ± 此值）(mm)"),
    ("carrier_pad_r", "4", "行星架銷孔周圍止推墊半徑 (mm)"),
    # frame
    ("carrier_thickness", "4", "行星架厚 (mm)"),
    ("base_thickness", "5", "底座厚 (mm)"),
    ("base_d", "120", "底座直徑 (mm)"),
    ("bearing_floor", "1", "底座軸承孔底厚 (mm)"),
    ("ring_outer_r", "45", "內齒圈外半徑 (mm)"),
    ("ring_h", "=gear_thickness + axial_gap", "內齒圈高（自動計算）"),
    ("mount_r", "42", "內齒圈／底座螺絲孔圓半徑 (mm)"),
    ("ring_screw_d", "2.8", "內齒圈螺絲孔直徑：M3 自攻 (mm)"),
    ("base_screw_d", "3.4", "底座螺絲孔直徑：M3 穿孔 (mm)"),
    ("rim_clear", "0.2", "底座定位環與內齒圈間隙 (mm)"),
    ("rim_w", "1.5", "底座定位環壁厚 (mm)"),
    ("rim_h", "3", "底座定位環高 (mm)"),
    ("crank_hub_d", "18", "搖柄輪轂直徑 (mm)"),
    ("crank_hub_h", "5", "搖柄輪轂高 (mm)"),
    ("crank_r", "28", "搖柄把手離中心 (mm)"),
    # derived positions
    ("orbit", "=module * (sun_teeth + planet_teeth) / 2", "行星軌道半徑＝中心距（自動計算）"),
    ("planet_phase", "=180 / planet_teeth", "行星齒輪轉半齒對齒（自動計算）(deg)"),
    ("ring_z", "=base_thickness", "內齒圈底面高（自動計算）"),
    ("gear_z", "=base_thickness + axial_gap", "齒輪底面高（自動計算）"),
    ("carrier_z", "=gear_z + gear_thickness + axial_gap", "行星架底面高（自動計算）"),
    ("crank_z", "=carrier_z + carrier_thickness + axial_gap", "搖柄底面高（自動計算）"),
    ("pin_shank", "=carrier_z + carrier_thickness - (gear_z + axial_gap)", "軸銷桿長（自動計算）"),
    ("shaft_len", "=crank_z + crank_hub_h - bearing_floor", "D 形軸長（自動計算）"),
]


def bind(obj, prop, expr):
    obj.setExpression(prop, expr)


def place(feat, x="0", y="0", z="0"):
    bind(feat, ".Placement.Base.x", x)
    bind(feat, ".Placement.Base.y", y)
    bind(feat, ".Placement.Base.z", z)
    return feat


def cylinder(body, name, cut, radius, height, **at):
    feat = body.newObject("PartDesign::SubtractiveCylinder" if cut else "PartDesign::AdditiveCylinder", name)
    bind(feat, "Radius", radius)
    bind(feat, "Height", height)
    return place(feat, **at)


def box(body, name, cut, length, width, height, **at):
    feat = body.newObject("PartDesign::SubtractiveBox" if cut else "PartDesign::AdditiveBox", name)
    bind(feat, "Length", length)
    bind(feat, "Width", width)
    bind(feat, "Height", height)
    return place(feat, **at)


def polar(body, name, original, count="3"):
    origin = {o.Role: o for o in body.Origin.OriginFeatures}
    feat = body.newObject("PartDesign::PolarPattern", name)
    feat.Originals = [original]
    feat.Axis = (origin["Z_Axis"], [""])
    feat.Angle = 360
    bind(feat, "Occurrences", count)
    body.Tip = feat
    return feat


def gear(body, name, teeth, external, height, cut=False):
    """InvoluteGear profile padded `height` up from z 0 (external), or pocketed through (internal)."""
    g = InvoluteGearFeature.makeInvoluteGear(f"{name}Profile")
    body.addObject(g)
    g.ExternalGear = external
    g.HighPrecision = True
    g.RootFilletCoefficient = 0
    bind(g, "NumberOfTeeth", teeth)
    bind(g, "Modules", P + "module")
    bind(g, "PressureAngle", P + "pressure_angle")
    s = P + "shift" if external else f"-{P}shift"
    bind(g, "ProfileShiftCoefficient", s)
    # keep tip and root radii where the unshifted gear has them
    if external:
        bind(g, "AddendumCoefficient", f"{P}add_c - {s}")
        bind(g, "DedendumCoefficient", f"{P}ded_c + {s}")
    else:
        bind(g, "AddendumCoefficient", f"{P}add_c + {s}")
        bind(g, "DedendumCoefficient", f"{P}ded_c - {s}")
    g.Visibility = False
    feat = body.newObject("PartDesign::Pocket" if cut else "PartDesign::Pad", name)
    feat.Profile = g
    if cut:
        feat.Type = "ThroughAll"
        feat.Midplane = True
    else:
        bind(feat, "Length", height)
    return feat


def d_bore(body, name, height):
    """D-shaped bore for the shaft (flat toward +x): a round bore with the flat's material added back."""
    r = f"({P}shaft_d / 2 + {P}key_clear)"
    flat_x = f"({P}shaft_d / 2 - {P}shaft_flat + {P}key_clear)"
    cylinder(body, f"{name}Bore", True, r, f"{height} + 2", z="-1")
    box(body, f"{name}Key", False, f"{r} - {flat_x} + 0.01", f"2 * {r} + 0.02", height,
        x=flat_x, y=f"-{r} - 0.01")


def sun(body):
    gear(body, "Gear", P + "sun_teeth", True, P + "gear_thickness")
    d_bore(body, "Shaft", P + "gear_thickness")


def planet(body):
    gear(body, "Gear", P + "planet_teeth", True, P + "gear_thickness")
    cylinder(body, "PinBore", True, f"{P}planet_bore_d / 2", f"{P}gear_thickness + 2", z="-1")


def ring(body):
    cylinder(body, "Ring", False, P + "ring_outer_r", P + "ring_h")
    gear(body, "Teeth", P + "ring_teeth", False, None, cut=True)
    hole = cylinder(body, "ScrewHole", True, f"{P}ring_screw_d / 2", f"{P}ring_h + 2", x=P + "mount_r", z="-1")
    polar(body, "ScrewHoles", hole)


def carrier(body):
    """Print orientation: thrust pads on top (the assembly turns it over so they face the planets)."""
    cylinder(body, "Plate", False, f"{P}orbit + 5", P + "carrier_thickness")
    pad = cylinder(body, "PinPad", False, P + "carrier_pad_r", P + "axial_gap", x=P + "orbit",
                   z=P + "carrier_thickness")
    polar(body, "PinPads", pad)
    cylinder(body, "CentreBore", True, f"{P}carrier_bore_d / 2", f"{P}carrier_thickness + 2", z="-1")
    hole = cylinder(body, "PinHole", True, f"{P}carrier_pin_hole_d / 2",
                    f"{P}carrier_thickness + {P}axial_gap + 2", x=P + "orbit", z="-1")
    polar(body, "PinHoles", hole)


def base(body):
    t = P + "base_thickness"
    rim_r = f"({P}ring_outer_r + {P}rim_clear)"
    cylinder(body, "Disc", False, f"{P}base_d / 2", t)
    cylinder(body, "Rim", False, f"{rim_r} + {P}rim_w", P + "rim_h", z=t)
    cylinder(body, "RimInside", True, rim_r, f"{P}rim_h + 1", z=t)
    # thrust track under the planets, then the pad under the sun (the track's inner cut would take the pad)
    cylinder(body, "Track", False, f"{P}orbit + {P}track_half_w", P + "axial_gap", z=t)
    cylinder(body, "TrackInside", True, f"{P}orbit - {P}track_half_w", f"{P}axial_gap + 1", z=t)
    cylinder(body, "SunPad", False, P + "sun_pad_r", P + "axial_gap", z=t)
    cylinder(body, "Bearing", True, f"{P}base_bearing_d / 2", f"{t} + {P}axial_gap", z=P + "bearing_floor")
    hole = cylinder(body, "ScrewHole", True, f"{P}base_screw_d / 2", f"{t} + 2", x=P + "mount_r", z="-1")
    polar(body, "ScrewHoles", hole)


def crank(body):
    cylinder(body, "Hub", False, f"{P}crank_hub_d / 2", P + "crank_hub_h")
    d_bore(body, "Shaft", P + "crank_hub_h")
    # arm: the hull of two D5 discs at x 7 and crank_r, z 2.5 to 7.5
    box(body, "Arm", False, f"{P}crank_r - 7", "5", "5", x="7", y="-2.5", z="2.5")
    cylinder(body, "ArmEnd", False, "2.5", "5", x="7", z="2.5")
    cylinder(body, "Boss", False, "4.5", "5", x=P + "crank_r", z="2.5")
    cylinder(body, "Handle", False, "2.5", "12", x=P + "crank_r", z="7.5")


def pin(body):
    """Print orientation: head down."""
    cylinder(body, "Head", False, f"{P}pin_head_d / 2", P + "pin_head_h")
    cylinder(body, "Shank", False, f"{P}pin_d / 2", P + "pin_shank", z=P + "pin_head_h")


def shaft(body):
    """D shaft along +z, flat toward +x."""
    cylinder(body, "Rod", False, f"{P}shaft_d / 2", P + "shaft_len")
    box(body, "Flat", True, P + "shaft_d", f"2 * {P}shaft_d", f"{P}shaft_len + 2",
        x=f"{P}shaft_d / 2 - {P}shaft_flat", y=f"-{P}shaft_d", z="-1")


# part: (builder, assembled placement: base x, y, z expressions, rotation axis, rotation angle expression)
PARTS = {
    "Base": (base, ("0", "0", "0"), None, None),
    "Ring": (ring, ("0", "0", P + "ring_z"), None, None),
    "Sun": (sun, ("0", "0", P + "gear_z"), None, None),
    "Planet": (planet, (P + "orbit", "0", P + "gear_z"), (0, 0, 1), P + "planet_phase"),
    "Carrier": (carrier, ("0", "0", f"{P}carrier_z + {P}carrier_thickness"), (1, 0, 0), "180"),
    "Pin": (pin, (P + "orbit", "0", f"{P}carrier_z + {P}carrier_thickness + {P}pin_head_h"), (1, 0, 0), "180"),
    "Shaft": (shaft, ("0", "0", P + "bearing_floor"), None, None),
    "Crank": (crank, ("0", "0", P + "crank_z"), None, None),
}


def main():
    doc = App.newDocument(STEM)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "參數說明")
    sheet.set("B1", "數值")
    for row, (alias, value, desc) in enumerate(PARAMS, start=2):
        sheet.set(f"A{row}", desc)
        sheet.set(f"B{row}", value)
        sheet.setAlias(f"B{row}", alias)
    doc.recompute()
    bodies = {}
    for name, (builder, base_xyz, axis, angle) in PARTS.items():
        body = doc.addObject("PartDesign::Body", name)
        builder(body)
        if axis:
            body.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(*axis), 0))
            bind(body, ".Placement.Rotation.Angle", angle)
        for k, expr in zip("xyz", base_xyz):
            bind(body, f".Placement.Base.{k}", expr)
        bodies[name] = body
    # 2nd and 3rd planet and pin: links turned 120 / 240 deg about the centre
    for name in ("Planet", "Pin"):
        for k in (1, 2):
            link = doc.addObject("App::Link", f"{name}{k + 1}")
            link.LinkedObject = bodies[name]
            link.LinkTransform = True
            link.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), 120 * k))
    doc.recompute()
    for body in bodies.values():
        for feature in body.Group:
            if feature.TypeId.startswith("PartDesign::"):
                feature.Visibility = feature == body.Tip
    bad = [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
    print(f"errors: {bad or 'none'}")
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    for name, body in bodies.items():
        shape = body.Shape.copy()
        shape.Placement = App.Placement()  # the part on its own, in its print-ready local frame
        bb = shape.BoundBox
        print(f"{name}: valid {shape.isValid()}, solids {len(shape.Solids)}, volume {shape.Volume:.2f} mm3, "
              f"bbox {bb.XLength:.3f} x {bb.YLength:.3f} x {bb.ZLength:.3f} mm")
        stem = f"planetary_{name.lower()}"
        shape.exportStep(os.path.join(out, f"{stem}.step"))
        Mesh.Mesh(shape.tessellate(0.01)).write(os.path.join(out, f"{stem}.stl"))
    path = os.path.join(HERE, f"{STEM}.FCStd")
    doc.saveAs(path)
    print(f"saved {path}")
    if not os.environ.get("NO_GUI_RESAVE"):
        log = os.path.join(out, "gui_finish.log")
        env = dict(os.environ, GUI_RESAVE=path, GUI_RESAVE_LOG=log)
        subprocess.run([os.path.join(App.getHomePath(), "bin", "freecad.exe"),
                        os.path.join(HERE, "..", "..", "tt_motor_adapter", "gui_finish.py")], env=env,
                       timeout=1800, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(open(log, encoding="utf-8").read())
    sys.stdout.flush()


main()
