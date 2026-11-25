"""A small picture for every model file in the works' download tables: web/img/<id>/files/<file name>.jpg

3MF: the Bambu Studio plate picture inside the file when it has one (Metadata/plate_1.png), else a cadgen snapshot.
STEP / STL: a cadgen snapshot. FCStd rows use the FreeCAD render (render_fcstd.py) or a same-stem model's picture;
build_site.py picks those. Only missing or out-of-date pictures are made (the file in web/files is newer).

Run after build_site.py has copied the files (then run build_site.py again):  python tools/file_thumbs.py
Uses the system python (Pillow) and cadgen through uvx for the snapshots, all in one batch job.
"""
import io
import json
import os
import subprocess
import sys
import tempfile
import zipfile

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from catalog import PROJECTS  # noqa: E402

WEB = os.path.dirname(HERE)
SIZE = (480, 360)
BG = (238, 242, 247)
CADGEN = ["uvx", "--no-config", "--managed-python", "--python", "3.13", "--from", "cadgen==0.7.26", "cadgen"]


def save_thumb(img, dst):
    img = img.convert("RGBA")
    img.thumbnail(SIZE, Image.LANCZOS)
    out = Image.new("RGB", SIZE, BG)
    out.paste(img, ((SIZE[0] - img.width) // 2, (SIZE[1] - img.height) // 2), img)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    out.save(dst, quality=85)


def plate_png(path):
    try:
        with zipfile.ZipFile(path) as z:
            names = z.namelist()
            for n in ("Metadata/plate_1.png", "Metadata/thumbnail.png", "Thumbnails/thumbnail.png"):
                if n in names:
                    return Image.open(io.BytesIO(z.read(n)))
    except zipfile.BadZipFile:
        pass
    return None


todo, jobs, tmp = [], [], tempfile.mkdtemp(prefix="thumbs_")
for p in PROJECTS:
    for entry in p["files"]:
        name = os.path.basename(entry[0])
        stem, ext = os.path.splitext(name)
        if len(entry) > 2 and entry[2] == "export":
            name = stem + ".3mf"
            ext = ".3mf"
        if ext.lower() not in (".3mf", ".step", ".stl"):
            continue
        src = os.path.join(WEB, "files", p["id"], name)
        dst = os.path.join(WEB, "img", p["id"], "files", name + ".jpg")
        if not os.path.exists(src):
            print(f"missing {src}")
            continue
        if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
            continue
        img = plate_png(src) if ext.lower() == ".3mf" else None
        if img is not None:
            save_thumb(img, dst)
            print(f"plate picture  {p['id']}/{name}")
            continue
        png = os.path.join(tmp, f"{len(jobs):03d}.png")
        jobs.append({"input": src, "outputs": [{"path": png, "camera": "iso"}]})
        todo.append((png, dst, f"{p['id']}/{name}"))

if jobs:
    job = os.path.join(tmp, "job.json")
    with open(job, "w", encoding="utf-8") as f:
        json.dump({"jobs": jobs}, f)
    subprocess.run(CADGEN + ["snapshot", "--job", job, "--width", "960", "--height", "720"], check=True)
    for png, dst, label in todo:
        if os.path.exists(png):
            save_thumb(Image.open(png), dst)
            print(f"snapshot       {label}")
        else:
            print(f"FAILED         {label}")
print(f"{len(todo)} snapshots, done")
