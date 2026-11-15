"""Build the TT motor adapter as a parametric FreeCAD model (tt_motor_adapter.FCStd).

Run:  freecadcmd.exe build_tt_adapter.py
The cradle alone; the geometry lives in tt_adapter_core.py.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tt_adapter_core as core  # noqa: E402

doc = core.make_doc("tt_motor_adapter", core.PARAMS)
body = doc.addObject("PartDesign::Body", "Adapter")
core.build_core(body)
core.finish(doc, body, "tt_motor_adapter")
core.gui_resave("tt_motor_adapter")
