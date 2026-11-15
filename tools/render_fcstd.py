"""Run inside freecad.exe (GUI): render an isometric picture of every FCStd that catalog.py lists as a
"render" image, into web/img/<id>/<stem>.png. Log to web/tools/render.log.

Run:  freecad.exe render_fcstd.py      (RENDER_ONLY="a;b" limits it to FCStd stems a and b)
"""
import os
import sys
import time

import FreeCAD as App
import FreeCADGui as Gui

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from catalog import PROJECTS, ROOT  # noqa: E402

WEB = os.path.dirname(HERE)
only = [s for s in os.environ.get("RENDER_ONLY", "").split(";") if s]
log = open(os.path.join(HERE, "render.log"), "w", encoding="utf-8")
try:
    for p in PROJECTS:
        for img in p["images"]:
            if img[0] != "render":
                continue
            stem = os.path.splitext(os.path.basename(img[1]))[0]
            if only and stem not in only:
                continue
            t = time.time()
            doc = App.openDocument(os.path.join(ROOT, img[1]))
            name = doc.Name  # read now: closing other documents later can invalidate `doc`
            Gui.updateGui()
            view = Gui.getDocument(name).activeView()
            view.viewIsometric()
            view.fitAll()
            for _ in range(10):
                Gui.updateGui()
                time.sleep(0.1)
            out_dir = os.path.join(WEB, "img", p["id"])
            os.makedirs(out_dir, exist_ok=True)
            view.saveImage(os.path.join(out_dir, f"{stem}.png"), 1200, 900, "Transparent")
            log.write(f"{p['id']}/{stem}.png {time.time() - t:.0f}s\n")
            log.flush()
            for open_doc in list(App.listDocuments()):  # the file and any documents it pulled in
                App.closeDocument(open_doc)
except Exception as exc:  # report and still quit
    log.write(f"ERROR {exc!r}\n")
log.close()
os._exit(0)
