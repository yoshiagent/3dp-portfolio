"""Build ujoint_diff_assembly.FCStd in the FreeCAD GUI from STEP/ujoint_diff_assembly.step.

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
STEP = os.path.join(os.path.dirname(HERE), "STEP", "ujoint_diff_assembly.step")
OUT = os.path.join(HERE, "ujoint_diff_assembly.FCStd")
LOG = os.path.join(HERE, "out", "assembly_build.log")

lines = []


def log(msg):
    lines.append(str(msg))
    App.Console.PrintMessage(f"{msg}\n")


def main():
    doc = App.newDocument("ujoint_diff_assembly")
    ImportGui.insert(STEP, doc.Name)
    doc.recompute()
    parts = [o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Shape.Solids]
    log(f"imported {len(parts)} parts from {os.path.basename(STEP)}")
    # Parts that share one shape in the STEP get the first one's name; rename every part from its position
    # (STEP/ujoint_diff_assembly_names.json, written by src/uj_assembly.py: label -> bounding-box centre).
    import json

    with open(os.path.join(os.path.dirname(HERE), "STEP", "ujoint_diff_assembly_names.json"), encoding="utf-8") as fh:
        centres = json.load(fh)
    for o in parts:
        c = o.Shape.BoundBox.Center
        o.Label = min(centres, key=lambda k: (c.x - centres[k][0]) ** 2 + (c.y - centres[k][1]) ** 2
                      + (c.z - centres[k][2]) ** 2)
    names = [o.Label for o in parts]
    dup = sorted({n for n in names if names.count(n) > 1})
    log(f"duplicate labels after renaming: {dup or 'none'}")
    for o in sorted(parts, key=lambda o: o.Label):
        bb = o.Shape.BoundBox
        log(f"  {o.Label:34s} {bb.XLength:6.1f} x {bb.YLength:6.1f} x {bb.ZLength:6.1f}  valid={o.Shape.isValid()}")
    roots = [o for o in doc.Objects if o.TypeId == "App::Part" and not o.InList]
    for r in roots:
        r.Label = "ujoint_diff"
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
