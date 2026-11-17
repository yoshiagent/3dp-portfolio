"""Build steering_assembly.FCStd in the FreeCAD GUI from STEP/steering_assembly.step.

Every part keeps its name and colour from the STEP (ImportGui), so the file opens in FreeCAD as the whole
MG90S front steering at straight ahead: servo holder (base + side cover), stand-in MG90S and horn, crank,
tie rods, knuckles, stand-in wheels and the steering frame. The file is self-contained (no links to the
part FCStd files), so it still opens after being copied on its own.

Run:  freecad.exe build_assembly.py   (GUI, so the imported parts keep their colours; quits when done)
Results go to out/assembly_build.log because GUI print output only reaches the report view.
"""
import os

import FreeCAD as App
import FreeCADGui as Gui
import ImportGui

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STEP = os.path.join(ROOT, "STEP", "steering_assembly.step")
OUT = os.path.join(ROOT, "steering_assembly.FCStd")
LOG = os.path.join(ROOT, "out", "assembly_build.log")

lines = []


def log(msg):
    lines.append(str(msg))
    App.Console.PrintMessage(f"{msg}\n")


def main():
    doc = App.newDocument("steering_assembly")
    ImportGui.insert(STEP, doc.Name)
    doc.recompute()
    parts = [o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Shape.Solids]
    log(f"imported {len(parts)} parts from {os.path.basename(STEP)}")
    # parts that share one shape in the STEP (the two wheels) come in named after the first one
    # ("wheel left ...", "wheel left ...001"): name left / right from where they sit (x < 0 is left)
    for o in parts:
        base = o.Label.rstrip("0123456789")
        side = "right" if o.Shape.BoundBox.Center.x > 0 else "left"
        for word in (" left", " right"):
            if word in base:
                o.Label = base.replace(word, f" {side}", 1)
                break
    names = [o.Label for o in parts]
    dup = sorted({n for n in names if names.count(n) > 1})
    log(f"duplicate labels after renaming: {dup or 'none'}")
    for o in sorted(parts, key=lambda o: o.Label):
        bb = o.Shape.BoundBox
        log(f"  {o.Label:34s} {bb.XLength:6.1f} x {bb.YLength:6.1f} x {bb.ZLength:6.1f}  valid={o.Shape.isValid()}")
    roots = [o for o in doc.Objects if o.TypeId == "App::Part" and not o.InList]
    for r in roots:
        r.Label = "mg90s_steering"
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
    # Quit through the main window rather than os._exit, so this session's recovery folders are removed and the
    # next FreeCAD start shows no Document Recovery. The second timer is only a safety net if a dialog blocks.
    from PySide import QtCore

    for name in list(App.listDocuments()):
        App.closeDocument(name)
    QtCore.QTimer.singleShot(0, Gui.getMainWindow().close)
    QtCore.QTimer.singleShot(120000, lambda: os._exit(0))
