"""Run inside freecad.exe (GUI): open each FCStd in env GUI_RESAVE (';'-separated), fit an isometric view,
save (writes GuiDocument.xml + thumbnail), then quit normally (closing the main window). Log to env
GUI_RESAVE_LOG."""
import os
import time
import zipfile

import FreeCAD as App
import FreeCADGui as Gui

log = open(os.environ["GUI_RESAVE_LOG"], "w", encoding="utf-8")
pref = App.ParamGet("User parameter:BaseApp/Preferences/Document")
log.write(f"SaveThumbnail pref: {pref.GetBool('SaveThumbnail', True)}\n")
try:
    for path in os.environ["GUI_RESAVE"].split(";"):
        doc = App.openDocument(path)
        doc.recompute()
        Gui.updateGui()
        gdoc = Gui.getDocument(doc.Name)
        # files saved headless carry no view state, so FreeCAD opens them with everything hidden:
        # show each body and its Tip, hide the other features
        for body in doc.findObjects("PartDesign::Body"):
            body.ViewObject.Visibility = True
            for feat in body.Group:
                if feat.TypeId.startswith("PartDesign::"):
                    feat.ViewObject.Visibility = feat == body.Tip
        Gui.updateGui()
        view = gdoc.activeView()
        view.viewIsometric()
        view.fitAll()
        # let the view finish drawing before the thumbnail is grabbed (batches came out blank otherwise)
        for _ in range(10):
            Gui.updateGui()
            time.sleep(0.1)
        doc.save()
        names = zipfile.ZipFile(path).namelist()
        log.write(f"{path}: GuiDocument={'GuiDocument.xml' in names}, "
                  f"thumbnail={'thumbnails/Thumbnail.png' in names}\n")
        App.closeDocument(doc.Name)
except Exception as exc:  # report and still quit
    log.write(f"ERROR {exc!r}\n")
log.close()

# Quit through the main window rather than os._exit: a normal shutdown removes this session's recovery
# (transient) folders, which os._exit left behind, so the next FreeCAD start showed Document Recovery.
# The timers fire once the GUI event loop runs; the second is only a safety net if a dialog blocks closing.
from PySide import QtCore  # noqa: E402

for name in list(App.listDocuments()):
    App.closeDocument(name)
QtCore.QTimer.singleShot(0, Gui.getMainWindow().close)
QtCore.QTimer.singleShot(120000, lambda: os._exit(0))
