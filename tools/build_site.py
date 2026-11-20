"""Build the 3D printing portfolio site in C:/CludeHome/3DP/web from catalog.py.

Run:  python build_site.py
Before it: freecadcmd.exe export_3mf.py (fresh 3MF files) and freecad.exe render_fcstd.py (FCStd previews).
Copies the listed files into files/<id>/, the pictures into img/<id>/, the demo page into demo/, and writes
index.html, works/<id>.html, assets/style.css and robots.txt. Nothing outside web/ is changed.
"""
import datetime
import html
import os
import re
import shutil
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from catalog import AUX_TOOLS, KINDS, PROJECTS, ROOT, SOFTWARE  # noqa: E402

WEB = os.path.dirname(HERE)
TODAY = datetime.date.today().isoformat()
FILE_TYPES = {".fcstd": ("FCStd", "fc"), ".3mf": ("3MF", "mf"), ".scad": ("SCAD", "src"), ".py": ("PY", "src"),
              ".txt": ("TXT", "src"), ".html": ("HTML", "src")}


def esc(text):
    """Escape for HTML and turn **x** into <strong>x</strong>, `x` into <code>x</code>."""
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", html.escape(text))
    return re.sub(r"`(.+?)`", r"<code>\1</code>", text)


# tool label on a work page -> anchor on the software page
TOOL_ANCHOR = {t: s["id"] for s in SOFTWARE for t in s["tools"]}


def tool_badges(tools, up):
    return "".join(f'<a class="badge tool" href="{up}software.html#{TOOL_ANCHOR.get(t, "aux")}">{esc(t)}</a>'
                   for t in tools)


def size_str(n):
    return f"{n / 1e6:.1f} MB" if n >= 1e6 else f"{max(n / 1e3, 0.1):.0f} KB"


def copy(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def collect(p):
    """Copy this project's files and pictures into web/; return (files, images) as site-relative paths."""
    pid = p["id"]
    files = []
    for entry in p["files"]:
        src, label = entry[0], entry[1]
        if len(entry) > 2 and entry[2] == "export":
            rel = f"files/{pid}/{os.path.splitext(os.path.basename(src))[0]}.3mf"
        else:
            rel = f"files/{pid}/{os.path.basename(src)}"
            copy(os.path.join(ROOT, src), os.path.join(WEB, rel))
        path = os.path.join(WEB, rel)
        if not os.path.exists(path):
            raise SystemExit(f"missing {path}: run export_3mf.py first")
        files.append((rel, label, os.path.getsize(path)))
    images = []
    for img in p["images"]:
        if img[0] == "file":
            rel = f"img/{pid}/{os.path.basename(img[1])}"
            copy(os.path.join(ROOT, img[1]), os.path.join(WEB, rel))
        elif img[0] == "zip":
            stem = os.path.splitext(os.path.basename(img[1]))[0]
            rel = f"img/{pid}/{stem}__{os.path.basename(img[2])}"
            os.makedirs(os.path.join(WEB, "img", pid), exist_ok=True)
            with open(os.path.join(WEB, rel), "wb") as fh:
                fh.write(zipfile.ZipFile(os.path.join(ROOT, img[1])).read(img[2]))
        else:  # render
            rel = f"img/{pid}/{os.path.splitext(os.path.basename(img[1]))[0]}.png"
            if not os.path.exists(os.path.join(WEB, rel)):
                print(f"warning: {rel} not rendered yet, skipped")
                continue
        images.append(rel)
    return files, images


def caption(rel):
    name = os.path.basename(rel)
    if "__" in name:
        stem, member = name.split("__", 1)
        return f"{stem}.3mf：Bambu Studio 列印盤 {member.split('_')[-1].split('.')[0]}" if "plate" in member \
            else f"{stem}.3mf 縮圖"
    return f"{os.path.splitext(name)[0]}" + ("（FreeCAD 等角視圖）" if rel.endswith(".png") and "preview" not in name
                                             and "thumb" not in name else "")


CSS = """:root {
  --bg: #080e18; --card: #131f2e; --deep: #07111e; --fg: #e2e8f0; --muted: #94a3b8; --line: #2d3f55;
  --blue: #3b82f6; --blue-soft: #93c5fd; --orange: #fb923c; --green: #4ade80;
}
* { box-sizing: border-box; }
html { overflow-x: hidden; }
body { margin: 0; background: var(--bg); color: var(--fg); line-height: 1.7;
  font-family: system-ui, -apple-system, "Segoe UI", "Microsoft JhengHei", "PingFang TC", sans-serif; }
a { color: var(--blue-soft); text-decoration: none; }
a:hover { text-decoration: underline; }
.wrap { max-width: 1180px; margin: 0 auto; padding: 0 16px; }
header.top { border-bottom: 1px solid var(--line); background: linear-gradient(180deg, #0d1828, var(--bg)); }
header.top .wrap { padding-top: 40px; padding-bottom: 32px; }
h1 { font-size: clamp(1.7rem, 4vw, 2.5rem); margin: 0 0 4px; letter-spacing: .02em; }
.en { color: var(--muted); font-size: .95rem; }
.lead { color: var(--muted); max-width: 760px; margin: 14px 0 0; }
.stats { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 22px; }
.stat { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 10px 16px; min-width: 120px; }
.stat b { display: block; font-size: 1.5rem; color: var(--fg); }
.stat span { color: var(--muted); font-size: .85rem; }
.filters { display: flex; flex-wrap: wrap; gap: 8px; margin: 28px 0 18px; }
.chip { background: var(--card); color: var(--muted); border: 1px solid var(--line); border-radius: 999px;
  padding: 6px 14px; font: inherit; font-size: .9rem; cursor: pointer; }
.chip[aria-pressed="true"] { background: #0f2040; color: var(--blue-soft); border-color: rgba(59,130,246,.55); }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(290px, 1fr)); gap: 20px; padding-bottom: 48px; }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 14px; overflow: hidden;
  display: flex; flex-direction: column; color: var(--fg); transition: border-color .15s, transform .15s; }
.card:hover { border-color: rgba(59,130,246,.6); transform: translateY(-2px); text-decoration: none; }
.thumb { aspect-ratio: 4 / 3; background: radial-gradient(circle at 50% 40%, #1d2f45, #0b1524 75%);
  display: flex; align-items: center; justify-content: center; overflow: hidden; }
.thumb img { width: 100%; height: 100%; object-fit: contain; }
.thumb .ph { color: var(--muted); font-size: .9rem; text-align: center; padding: 20px; }
.thumb .ph b { display: block; font-size: 2.4rem; color: #3b4f68; }
.card .body { padding: 14px 16px 16px; display: flex; flex-direction: column; gap: 6px; flex: 1; }
.card h2 { font-size: 1.12rem; margin: 0; line-height: 1.4; }
.card p { margin: 0; color: var(--muted); font-size: .92rem; }
.meta { display: flex; flex-wrap: wrap; gap: 6px; align-items: center; font-size: .8rem; color: var(--muted); }
.badge { border-radius: 6px; padding: 1px 8px; font-size: .78rem; border: 1px solid var(--line); white-space: nowrap; }
.k-own { color: var(--green); border-color: rgba(74,222,128,.4); }
.k-derived { color: var(--orange); border-color: rgba(251,146,60,.45); }
.k-third { color: var(--muted); }
.t-fc { color: var(--blue-soft); border-color: rgba(59,130,246,.45); }
.t-mf { color: var(--orange); border-color: rgba(251,146,60,.45); }
.t-src { color: var(--muted); }
.card .files { margin-top: auto; padding-top: 6px; display: flex; gap: 6px; flex-wrap: wrap; }
/* detail */
.crumb { padding: 18px 0 0; font-size: .9rem; }
.head { padding: 14px 0 6px; }
.head .meta { margin-top: 10px; }
.callout { background: #0f2040; border: 1px solid rgba(59,130,246,.3); color: #93c5fd; border-radius: 10px;
  padding: 12px 16px; margin: 18px 0; }
.layout { display: grid; grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr); gap: 28px; margin-top: 18px; }
@media (max-width: 860px) { .layout { grid-template-columns: 1fr; } }
.gallery .main { background: radial-gradient(circle at 50% 40%, #1d2f45, #0b1524 75%); border: 1px solid var(--line);
  border-radius: 14px; aspect-ratio: 4 / 3; display: flex; align-items: center; justify-content: center; overflow: hidden; }
.gallery .main img { width: 100%; height: 100%; object-fit: contain; }
.gallery .cap { color: var(--muted); font-size: .85rem; margin: 6px 2px 0; }
.strip { display: flex; gap: 8px; margin-top: 10px; flex-wrap: wrap; }
.strip button { width: 84px; height: 63px; padding: 0; border: 1px solid var(--line); border-radius: 8px; cursor: pointer;
  background: #0b1524; overflow: hidden; }
.strip button[aria-current="true"] { border-color: var(--blue); }
.strip img { width: 100%; height: 100%; object-fit: contain; }
.panel { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 16px 18px; }
.layout > .panel { align-self: start; }
.panel h2, .sec h2 { font-size: 1.1rem; margin: 0 0 10px; }
dl.specs { display: grid; grid-template-columns: auto 1fr; gap: 6px 16px; margin: 0; }
dl.specs dt { color: var(--muted); white-space: nowrap; }
dl.specs dd { margin: 0; }
.tools { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 14px; }
.demo { display: inline-block; margin-top: 16px; background: var(--blue); color: #fff; border-radius: 10px;
  padding: 10px 18px; font-weight: 600; }
.demo:hover { text-decoration: none; filter: brightness(1.1); }
.demo + .demo { margin-left: 8px; }
.secs { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 20px; margin: 28px 0; }
.sec { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 16px 18px; }
.sec ul { margin: 0; padding-left: 1.2em; }
.sec li { margin: 6px 0; }
table.files { width: 100%; border-collapse: collapse; }
table.files th, table.files td { text-align: left; padding: 10px 8px; border-bottom: 1px solid var(--line); vertical-align: top; }
table.files th { color: var(--muted); font-weight: 500; font-size: .85rem; }
table.files .fn { font-family: ui-monospace, Consolas, monospace; font-size: .85rem; word-break: break-all; }
table.files .sz { color: var(--muted); white-space: nowrap; font-size: .85rem; }
.tablewrap { overflow-x: auto; }
.none { color: var(--muted); }
.parts { margin: 28px 0; }
.parts > h2 { font-size: 1.1rem; margin: 0 0 4px; }
.parts > p { margin: 0 0 14px; }
.pgrid { display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 14px; }
.pgrid .card .thumb { aspect-ratio: 16 / 10; }
.pgrid .card .body { padding: 10px 12px 12px; gap: 4px; }
.pgrid .card h3 { font-size: .98rem; margin: 0; line-height: 1.4; }
.pgrid .card p { font-size: .86rem; }
.role { color: var(--blue-soft); border-color: rgba(59,130,246,.45); align-self: flex-start; }
.usedin { display: flex; flex-wrap: wrap; gap: 6px 10px; align-items: center; margin: 18px 0 0; }
.usedin a { background: #0f2040; border: 1px solid rgba(59,130,246,.4); border-radius: 8px; padding: 4px 10px; }
.pager { display: flex; justify-content: space-between; gap: 12px; margin: 36px 0 12px; flex-wrap: wrap; }
.pager a { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 10px 14px; max-width: 48%; }
.topnav { background: #0b1524; border-bottom: 1px solid var(--line); font-size: .9rem; }
.topnav .wrap { display: flex; flex-wrap: wrap; align-items: center; gap: 4px 6px; padding-top: 9px; padding-bottom: 9px; }
.topnav .sep { color: var(--line); }
code { background: #07111e; color: #7dd3fc; border: 1px solid rgba(59,130,246,.3); border-radius: 5px;
  padding: 0 5px; font-family: ui-monospace, Consolas, monospace; font-size: .88em; word-break: break-all; }
a.badge.tool { color: var(--fg); }
a.badge.tool:hover { border-color: rgba(59,130,246,.6); text-decoration: none; }
.statlink { color: var(--fg); }
.statlink:hover { border-color: rgba(59,130,246,.6); text-decoration: none; }
.statlink span { color: var(--blue-soft); }
ol.flow { list-style: none; padding: 0; margin: 22px 0 0; display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
ol.flow li { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 10px 14px; position: relative; }
ol.flow li b { display: block; }
ol.flow li span { color: var(--muted); font-size: .85rem; }
ol.flow li:not(:last-child)::after { content: "→"; position: absolute; right: -10px; top: 50%; transform: translateY(-50%);
  color: var(--blue); font-weight: 700; }
@media (max-width: 700px) { ol.flow { grid-template-columns: 1fr 1fr; } ol.flow li::after { display: none; } }
.toc { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 18px; }
.sw { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 20px 22px; margin: 24px 0;
  scroll-margin-top: 16px; }
.swhead { display: flex; justify-content: space-between; align-items: flex-start; gap: 12px; flex-wrap: wrap; }
.sw h2 { margin: 0; font-size: 1.35rem; }
.sw > p { color: var(--fg); margin: 12px 0; }
.subs { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 14px; margin-top: 14px; }
.sub { background: var(--deep); border: 1px solid var(--line); border-radius: 10px; padding: 12px 16px; margin-top: 14px; }
.subs .sub { margin-top: 0; }
.sub h3 { font-size: 1rem; margin: 0 0 6px; color: var(--blue-soft); }
.sub ul { margin: 0; padding-left: 1.2em; }
.sub li { margin: 5px 0; }
ul.works { list-style: none; padding: 0; display: flex; flex-wrap: wrap; gap: 8px; }
ul.works li { margin: 0; }
ul.works a { display: inline-block; border: 1px solid var(--line); border-radius: 8px; padding: 3px 10px; font-size: .9rem; }
ul.works a:hover { border-color: rgba(59,130,246,.6); text-decoration: none; }
footer { border-top: 1px solid var(--line); color: var(--muted); font-size: .85rem; margin-top: 24px; }
footer .wrap { padding-top: 18px; padding-bottom: 36px; }
footer ul { margin: 6px 0 0; padding-left: 1.2em; }
"""


def page(title, body, depth):
    up = "../" * depth
    return f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<meta name="robots" content="noindex, nofollow">
<title>{html.escape(title)}</title>
<link rel="stylesheet" href="{up}assets/style.css">
</head>
<body>
<nav class="topnav"><div class="wrap">
<a href="https://yoshiagent.github.io/MechanicalAI/">← MechanicalAI</a>
<span class="sep">｜</span><a href="{up}index.html">3D 列印作品集</a>
<span class="sep">｜</span><a href="{up}software.html">使用的設計軟體</a>
</div></nav>
{body}
</body>
</html>
"""


def footer_html(extra=""):
    return f"""<footer><div class="wrap">
<strong>資料來源</strong>
<ul>
<li>模型、參數與說明：本機專案資料夾 C:\\CludeHome\\3DP（FreeCAD、OpenSCAD、build123d 原始檔與 Bambu Studio 專案）</li>
<li>FreeCAD 預覽圖由 FreeCAD 1.1 繪製；3MF 縮圖取自檔案內建的 Bambu Studio 列印盤預覽</li>
{extra}<li>網站產生日期：{TODAY}</li>
</ul></div></footer>"""


def file_badges(files):
    counts = {}
    for rel, _, _ in files:
        name, cls = FILE_TYPES.get(os.path.splitext(rel)[1].lower(), ("檔案", "src"))
        if cls != "src":
            counts[(name, cls)] = counts.get((name, cls), 0) + 1
    return "".join(f'<span class="badge t-{c}">{n} × {k}</span>' for (n, c), k in counts.items())


def build_software(built):
    """software.html: the design software installed on this computer and the works each one was used for."""
    flow = [("設計建模", "FreeCAD、OpenSCAD、build123d、Python"), ("匯出", "FCStd → STEP／STL／3MF"),
            ("切片", "Bambu Studio"), ("列印", "Bambu Lab X2D")]
    flow_html = '<ol class="flow">' + "".join(
        f"<li><b>{esc(a)}</b><span>{esc(b)}</span></li>" for a, b in flow) + "</ol>"
    toc = "".join(f'<a class="badge tool" href="#{s["id"]}">{esc(s["name"])}</a>' for s in SOFTWARE)
    cards = ""
    for s in SOFTWARE:
        works = [p for p, _, _ in built if any(TOOL_ANCHOR.get(t) == s["id"] for t in p["tools"])]
        works_html = "".join(f'<li><a href="works/{p["id"]}.html">{esc(p["title"])}</a></li>' for p in works)
        secs = "".join(f'<div class="sub"><h3>{esc(h)}</h3><ul>' + "".join(f"<li>{esc(b)}</li>" for b in items)
                       + "</ul></div>" for h, items in s["sections"])
        cards += f"""<section class="sw" id="{s["id"]}">
<div class="swhead">
<div><h2>{esc(s["name"])}</h2><div class="en">{esc(s["role"])}</div></div>
<a class="badge tool" href="{s["url"]}" target="_blank" rel="noopener">官方網站 ↗</a>
</div>
<p>{esc(s["summary"])}</p>
<dl class="specs"><dt>本機版本</dt><dd><strong>{esc(s["version"])}</strong></dd>
<dt>安裝位置</dt><dd><code>{esc(s["where"])}</code></dd></dl>
<div class="subs">{secs}</div>
<div class="sub"><h3>用在這些作品（{len(works)} 件）</h3><ul class="works">{works_html}</ul></div>
</section>
"""
    aux_rows = "".join(f"<tr><td><strong>{esc(n)}</strong></td><td>{esc(v)}</td><td>{esc(u)}</td></tr>"
                       for n, v, u in AUX_TOOLS)
    body = f"""<header class="top"><div class="wrap">
<div class="crumb"><a href="index.html">← 全部作品</a></div>
<h1>使用的設計軟體</h1>
<div class="en">Design Software on This Computer</div>
<p class="lead">這台電腦上用來設計、檢查與列印這些作品的軟體：版本、安裝位置、在本站專案裡怎麼用，以及實際踩過的雷。</p>
{flow_html}
<div class="toc">{toc}<a class="badge tool" href="#aux">輔助工具</a></div>
</div></header>
<main class="wrap">
{cards}
<section class="sw" id="aux">
<div class="swhead"><div><h2>輔助工具</h2><div class="en">Supporting Tools</div></div></div>
<div class="tablewrap"><table class="files"><thead><tr><th>工具</th><th>版本</th><th>用途</th></tr></thead>
<tbody>{aux_rows}</tbody></table></div>
<p class="none">印表機：Bambu Lab X2D（雙噴頭，左直驅、右遠端送料）。</p>
</section>
</main>
{footer_html('<li>版本資訊：Windows「已安裝的程式」清單、pip list 與各軟體的 --version 輸出（查詢日期 ' + TODAY + '）</li>')}"""
    with open(os.path.join(WEB, "software.html"), "w", encoding="utf-8") as fh:
        fh.write(page("使用的設計軟體｜3D 列印作品集", body, 0))


def build():
    built = []
    for p in PROJECTS:
        files, images = collect(p)
        built.append((p, files, images))
    built.sort(key=lambda b: b[0]["date"], reverse=True)
    by_id = {b[0]["id"]: b for b in built}
    used_in = {}  # component id -> works that list it under "parts"
    for p, _, _ in built:
        for pid, _, _ in p.get("parts", []):
            if pid not in by_id:
                raise SystemExit(f"{p['id']}: unknown part id {pid!r}")
            used_in.setdefault(pid, []).append(p)

    # detail pages
    for n, (p, files, images) in enumerate(built):
        parts = ""
        if p.get("parts"):
            pcards = ""
            for pid, role, note in p["parts"]:
                q, _, qimgs = by_id[pid]
                img = (f'<img src="../{qimgs[0]}" alt="" loading="lazy">' if qimgs
                       else '<div class="ph"><b>3D</b></div>')
                pcards += (f'<a class="card" href="{pid}.html"><div class="thumb">{img}</div><div class="body">'
                           f'<span class="badge role">{esc(role)}</span><h3>{esc(q["title"])}</h3>'
                           f'<p>{esc(note)}</p></div></a>')
            parts = (f'<section class="parts"><h2>組成零件（{len(p["parts"])} 件作品）</h2>'
                     '<p class="none">以下零件直接沿用各自的作品；點進去可看設計說明、下載列印檔與原始碼。</p>'
                     f'<div class="pgrid">{pcards}</div></section>')
        usedin = ""
        if used_in.get(p["id"]):
            usedin = ('<div class="usedin"><span class="none">用在這些組合：</span>' + "".join(
                f'<a href="{u["id"]}.html">{esc(u["title"])} →</a>' for u in used_in[p["id"]]) + "</div>")
        kind = f'<span class="badge k-{p["kind"]}">{KINDS[p["kind"]]}</span>'
        tools = tool_badges(p["tools"], "../")
        credit = f'<div class="callout">{esc(p["credit"])}</div>' if p.get("credit") else ""
        if images:
            main = f'<div class="main"><img id="mainimg" src="../{images[0]}" alt="{esc(p["title"])}"></div>'
            cap = f'<p class="cap" id="cap">{esc(caption(images[0]))}</p>'
            strip = ""
            if len(images) > 1:
                strip = '<div class="strip">' + "".join(
                    f'<button type="button" data-src="../{im}" data-cap="{esc(caption(im))}" '
                    f'aria-current="{"true" if i == 0 else "false"}" aria-label="圖片 {i + 1}">'
                    f'<img src="../{im}" alt="" loading="lazy"></button>' for i, im in enumerate(images)) + "</div>"
            gallery = main + cap + strip
        else:
            gallery = ('<div class="main"><div class="thumb"><div class="ph"><b>3D</b>依原作授權不提供圖片與模型檔</div>'
                       '</div></div>')
        specs = "".join(f"<dt>{esc(k)}</dt><dd>{esc(v)}</dd>" for k, v in p["specs"])
        demo = ""
        if p.get("demo"):
            demo = f'<a class="demo" href="../demo/{os.path.basename(p["demo"][0])}">{esc(p["demo"][1])} ↗</a>'
        for url, label in p.get("videos", []):  # external videos (YouTube), opened in a new tab
            demo += f'<a class="demo" href="{html.escape(url)}" target="_blank" rel="noopener">{esc(label)} ↗</a>'
        secs = "".join(f'<section class="sec"><h2>{esc(h)}</h2><ul>' + "".join(f"<li>{esc(b)}</li>" for b in items)
                       + "</ul></section>" for h, items in p["sections"])
        if files:
            rows = ""
            for rel, label, size in files:
                name, cls = FILE_TYPES.get(os.path.splitext(rel)[1].lower(), ("檔案", "src"))
                rows += (f'<tr><td><span class="badge t-{cls}">{name}</span></td><td>{esc(label)}<br>'
                         f'<a class="fn" href="../{rel}" download>{html.escape(os.path.basename(rel))}</a></td>'
                         f'<td class="sz">{size_str(size)}</td></tr>')
            ftable = ('<div class="tablewrap"><table class="files"><thead><tr><th>類型</th><th>說明／下載</th>'
                      f'<th>大小</th></tr></thead><tbody>{rows}</tbody></table></div>'
                      '<p class="none">FCStd 用 FreeCAD 1.1 開啟（參數在 Params 試算表）；3MF 可直接拖進 Bambu Studio 或其他切片軟體。</p>')
        else:
            ftable = '<p class="none">此作品依原作授權不提供檔案下載。</p>'
        prev_l = built[n - 1] if n > 0 else None
        next_l = built[n + 1] if n + 1 < len(built) else None
        pager = '<nav class="pager">' + (
            f'<a href="{prev_l[0]["id"]}.html">← 較新：{esc(prev_l[0]["title"])}</a>' if prev_l else "<span></span>") + (
            f'<a href="{next_l[0]["id"]}.html">較舊：{esc(next_l[0]["title"])} →</a>' if next_l else "<span></span>") + "</nav>"
        body = f"""<div class="wrap">
<div class="crumb"><a href="../index.html">← 全部作品</a>　｜　<a href="../software.html">使用的設計軟體</a></div>
<div class="head">
<h1>{esc(p["title"])}</h1>
<div class="en">{esc(p["en"])}</div>
<div class="meta">{kind}<span>{p["date"]}</span></div>
</div>
{credit}
<p class="lead">{esc(p["summary"])}</p>
{usedin}
<div class="layout">
<div class="gallery">{gallery}</div>
<div class="panel"><h2>重點規格</h2><dl class="specs">{specs}</dl>
<div class="tools">{tools}</div>{demo}</div>
</div>
{parts}
<div class="secs">{secs}</div>
<section class="panel"><h2>檔案下載</h2>{ftable}</section>
{pager}
</div>
{footer_html()}
<script>
document.querySelectorAll('.strip button').forEach(function (b) {{
  b.addEventListener('click', function () {{
    document.getElementById('mainimg').src = b.dataset.src;
    document.getElementById('cap').textContent = b.dataset.cap;
    document.querySelectorAll('.strip button').forEach(function (o) {{ o.setAttribute('aria-current', o === b); }});
  }});
}});
</script>"""
        os.makedirs(os.path.join(WEB, "works"), exist_ok=True)
        with open(os.path.join(WEB, "works", f"{p['id']}.html"), "w", encoding="utf-8") as fh:
            fh.write(page(f"{p['title']}｜3D 列印作品集", body, 1))
        if p.get("demo"):
            copy(os.path.join(ROOT, p["demo"][0]), os.path.join(WEB, "demo", os.path.basename(p["demo"][0])))

    # index
    cards = ""
    for p, files, images in built:
        img = (f'<img src="{images[0]}" alt="" loading="lazy">' if images
               else '<div class="ph"><b>3D</b>依原作授權不提供圖片</div>')
        cards += f"""<a class="card" href="works/{p["id"]}.html" data-kind="{p["kind"]}">
<div class="thumb">{img}</div>
<div class="body">
<div class="meta"><span class="badge k-{p["kind"]}">{KINDS[p["kind"]]}</span><span>{p["date"]}</span></div>
<h2>{esc(p["title"])}</h2>
<p>{esc(p["summary"])}</p>
<div class="files">{file_badges(files)}</div>
</div></a>
"""
    n_fc = sum(1 for _, fs, _ in built for rel, _, _ in fs if rel.lower().endswith(".fcstd"))
    n_mf = sum(1 for _, fs, _ in built for rel, _, _ in fs if rel.lower().endswith(".3mf"))
    n_own = sum(1 for p, _, _ in built if p["kind"] != "third")
    chips = '<button class="chip" type="button" data-f="all" aria-pressed="true">全部</button>' + "".join(
        f'<button class="chip" type="button" data-f="{k}" aria-pressed="false">{v}</button>' for k, v in KINDS.items())
    body = f"""<header class="top"><div class="wrap">
<h1>3D 列印作品集</h1>
<div class="en">3D Printing Portfolio — LEGO / DUPLO / Technic 相容零件、齒輪教學模型</div>
<p class="lead">自己設計與改作的 3D 列印作品：每件附設計說明、重點尺寸與可下載的 FreeCAD 參數化模型（FCStd）和列印檔（3MF）。</p>
<div class="stats">
<div class="stat"><b>{len(built)}</b><span>件作品（自製／改作 {n_own}）</span></div>
<div class="stat"><b>{n_fc}</b><span>個 FCStd 參數化模型</span></div>
<div class="stat"><b>{n_mf}</b><span>個 3MF 列印檔</span></div>
<a class="stat statlink" href="software.html"><b>{len(SOFTWARE)}</b><span>套設計軟體介紹 →</span></a>
</div>
</div></header>
<main class="wrap">
<div class="filters" role="group" aria-label="篩選">{chips}</div>
<div class="grid">
{cards}</div>
</main>
{footer_html('<li>他人作品只做文字記錄並標註原作者；依原作授權（Standard Digital File License）不提供其模型檔</li>')}
<script>
document.querySelectorAll('.chip').forEach(function (c) {{
  c.addEventListener('click', function () {{
    document.querySelectorAll('.chip').forEach(function (o) {{ o.setAttribute('aria-pressed', o === c); }});
    document.querySelectorAll('.card').forEach(function (card) {{
      card.style.display = (c.dataset.f === 'all' || card.dataset.kind === c.dataset.f) ? '' : 'none';
    }});
  }});
}});
</script>"""
    with open(os.path.join(WEB, "index.html"), "w", encoding="utf-8") as fh:
        fh.write(page("3D 列印作品集", body, 0))
    build_software(built)
    os.makedirs(os.path.join(WEB, "assets"), exist_ok=True)
    with open(os.path.join(WEB, "assets", "style.css"), "w", encoding="utf-8") as fh:
        fh.write(CSS)
    with open(os.path.join(WEB, "robots.txt"), "w", encoding="utf-8") as fh:
        fh.write("User-agent: *\nDisallow: /\n")
    print(f"built {len(built)} works, {n_fc} FCStd, {n_mf} 3MF")


build()
