"""Export fresh 3MF files from the FCStd models marked "export" in catalog.py into web/files/<id>/.

Run:  freecadcmd.exe export_3mf.py
Each file is the model's (only) PartDesign body, meshed with 0.02 mm deviation.
"""
import os
import sys

import FreeCAD as App
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from catalog import PROJECTS, ROOT  # noqa: E402

WEB = os.path.dirname(HERE)
for p in PROJECTS:
    for entry in p["files"]:
        if len(entry) < 3 or entry[2] != "export":
            continue
        src = os.path.join(ROOT, entry[0])
        doc = App.openDocument(src)
        body = doc.findObjects("PartDesign::Body")[0]
        shape = body.Shape
        dst_dir = os.path.join(WEB, "files", p["id"])
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, os.path.splitext(os.path.basename(src))[0] + ".3mf")
        Mesh.export([body], dst, 0.02)  # same mesher as File > Export in the GUI
        check = Mesh.Mesh(dst)
        print(f"{dst}: solid {shape.Volume:.1f} mm3, mesh {check.Volume:.1f} mm3, "
              f"closed {check.isSolid()}, non-manifold {check.hasNonManifolds()}")
        App.closeDocument(doc.Name)
