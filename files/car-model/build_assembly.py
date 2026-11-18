"""Build car_assembly.FCStd in the FreeCAD GUI from STEP/car_assembly.step.

Every part keeps its name and colour from the STEP (ImportGui). The file is self-contained: no links to other
FCStd files, so it still opens after being downloaded on its own.

Run:  freecad.exe build_assembly.py   (GUI, so the imported parts keep their colours; quits when done)
Results go to out/assembly_build.log because GUI print output only reaches the report view.
"""
import os

import FreeCAD as App
import FreeCADGui as Gui
import ImportGui

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(os.path.dirname(HERE), "STEP", "car_assembly.step")
OUT = os.path.join(HERE, "car_assembly.FCStd")
LOG = os.path.join(HERE, "out", "assembly_build.log")

lines = []


def log(msg):
    lines.append(str(msg))
    App.Console.PrintMessage(f"{msg}\n")


def main():
    doc = App.newDocument("car_assembly")
    ImportGui.insert(STEP, doc.Name)
    doc.recompute()
    parts = [o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Shape.Solids]
    log(f"imported {len(parts)} parts from {os.path.basename(STEP)}")
    # Parts that share one shape in the STEP get the first one's name (wheel:left, wheel:left001).
    # Name each side from where it sits: x > 0 right, z > 0 top, y > -60 front.
    for o in parts:
        base, _, side = o.Label.rstrip("0123456789").partition(":")
        c = o.Shape.BoundBox.Center
        if side in ("left", "right"):
            o.Label = f"{base}:{'right' if c.x > 0 else 'left'}"
        elif side in ("top", "bottom"):
            o.Label = f"{base}:{'top' if c.z > 40 else 'bottom'}"
        elif side in ("front", "rear"):
            o.Label = f"{base}:{'front' if c.y > -60 else 'rear'}"
    names = [o.Label for o in parts]
    dup = sorted({n for n in names if names.count(n) > 1})
    log(f"duplicate labels after renaming: {dup or 'none'}")
    for o in sorted(parts, key=lambda o: o.Label):
        bb = o.Shape.BoundBox
        log(f"  {o.Label:34s} {bb.XLength:6.1f} x {bb.YLength:6.1f} x {bb.ZLength:6.1f}  valid={o.Shape.isValid()}")
    roots = [o for o in doc.Objects if o.TypeId == "App::Part" and not o.InList]
    for r in roots:
        r.Label = "car_model"
    bad = [o.Label for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
    log(f"errors: {bad or 'none'}")
    gdoc = Gui.getDocument(doc.Name)
    gdoc.ActiveView.viewIsometric()
    Gui.SendMsgToActiveView("ViewFit")
    Gui.updateGui()
    doc.saveAs(OUT)
    log(f"saved {OUT} ({os.path.getsize(OUT) / 1e6:.1f} MB)")


try:
    main()
except Exception as exc:  # record why, then quit anyway
    log(f"FAILED: {exc!r}")
finally:
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    with open(LOG, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    from PySide import QtCore

    for name in list(App.listDocuments()):
        App.closeDocument(name)
    QtCore.QTimer.singleShot(0, Gui.getMainWindow().close)
    QtCore.QTimer.singleShot(120000, lambda: os._exit(0))
