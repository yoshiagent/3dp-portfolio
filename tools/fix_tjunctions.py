"""Close T-junction open edges in 3MF and binary STL files, in place.

Run:  python tools/fix_tjunctions.py file.3mf [more.3mf | file.stl ...]

A T-junction is a long triangle edge on one side of a seam and two or more shorter edges, through
vertices lying on it, on the other side. Slicers count those edges as open. The fix splits the
triangle with the long edge at those vertices (a fan towards its third corner), so both sides share
the same edges. No vertex is added or moved; only <triangles> change.

Works on plain 3MF (cadgen) and Bambu Studio projects (3D/Objects/*.model): every other entry of the
zip is copied unchanged, and in a Bambu project the face counts in Metadata/model_settings.config are
updated for the parts that changed. Extra triangle attributes (paint data) are kept on the split pieces.
"""
from __future__ import annotations

import re
import sys
import zipfile
from collections import Counter
from pathlib import Path

import numpy as np

TOL = 1e-4  # mm: distance from the edge for a vertex to count as lying on it

VERT = re.compile(r'<vertex\s+x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"\s*/>')
TRI = re.compile(r'<triangle\s+v1="(\d+)"\s+v2="(\d+)"\s+v3="(\d+)"([^/]*)/>')
OBJ = re.compile(r'(<object\s+id="(\d+)"[^>]*>)(.*?)(</object>)', re.S)
TRIS_BLOCK = re.compile(r'(<triangles>)(.*?)(</triangles>)', re.S)


def boundary(tris: list[tuple[int, int, int]]) -> set[tuple[int, int]]:
    count = Counter()
    for a, b, c in tris:
        for e in ((a, b), (b, c), (c, a)):
            count[(min(e), max(e))] += 1
    return {e for e, k in count.items() if k == 1}


def repair(verts: np.ndarray, tris: list[tuple[int, int, int]], extra: list[str]):
    """Returns (tris, extra, open edges before, open edges after)."""
    before = len(boundary(tris))
    for _ in range(50):
        bnd = boundary(tris)
        if not bnd:
            break
        bverts = np.array(sorted({v for e in bnd for v in e}))
        new_t, new_x, split = [], [], False
        for (a, b, c), x in zip(tris, extra):
            done = False
            for p, q, r in ((a, b, c), (b, c, a), (c, a, b)):
                if (min(p, q), max(p, q)) not in bnd:
                    continue
                d = verts[q] - verts[p]
                L2 = float(d @ d)
                if L2 == 0:
                    continue
                w = verts[bverts] - verts[p]
                t = w @ d / L2
                off = np.linalg.norm(w - np.outer(t, d), axis=1)
                on = (off < TOL) & (t > 1e-6) & (t < 1 - 1e-6) & (bverts != p) & (bverts != q)
                if not on.any():
                    continue
                mids = [int(v) for v in bverts[on][np.argsort(t[on])]]
                chain = [p] + mids + [q]
                for u, v in zip(chain, chain[1:]):
                    new_t.append((u, v, r))
                    new_x.append(x)
                done = split = True
                break
            if not done:
                new_t.append((a, b, c))
                new_x.append(x)
        tris, extra = new_t, new_x
        if not split:
            break
    return tris, extra, before, len(boundary(tris))


def fix_model(xml: str, label: str):
    """Repair every mesh object in one .model document. Returns (xml, {object id: triangle delta})."""
    deltas = {}

    def fix_obj(m):
        head, oid, body, tail = m.groups()
        tb = TRIS_BLOCK.search(body)
        if not tb or "<vertices>" not in body:
            return m.group(0)
        verts = np.array(VERT.findall(body), float)
        parsed = TRI.findall(tb.group(2))
        tris = [(int(a), int(b), int(c)) for a, b, c, _ in parsed]
        extra = [x for *_, x in parsed]
        new_t, new_x, before, after = repair(verts, tris, extra)
        print(f"  {label} object {oid}: {len(tris)} triangles, open edges {before} -> {after}"
              + (f", +{len(new_t) - len(tris)} triangles" if len(new_t) != len(tris) else ""))
        if len(new_t) == len(tris):
            return m.group(0)
        deltas[oid] = len(new_t) - len(tris)
        sep = "\n" if "\n" in tb.group(2) else ""
        rows = sep.join(f'<triangle v1="{a}" v2="{b}" v3="{c}"{x}/>' for (a, b, c), x in zip(new_t, new_x))
        body = body[:tb.start()] + tb.group(1) + sep + rows + sep + tb.group(3) + body[tb.end():]
        return head + body + tail

    return OBJ.sub(fix_obj, xml), deltas


def fix_3mf(path: Path) -> None:
    print(path)
    with zipfile.ZipFile(path) as z:
        infos = z.infolist()
        data = {i.filename: z.read(i.filename) for i in infos}
    changed = {}
    for name in data:
        if name.endswith(".model") and b"<vertices>" in data[name]:
            xml, deltas = fix_model(data[name].decode("utf-8"), name)
            if deltas:
                data[name] = xml.encode("utf-8")
                changed[name] = deltas
    if not changed:
        print("  nothing to fix")
        return
    cfg = "Metadata/model_settings.config"
    if cfg in data and "3D/3dmodel.model" in data:
        # Bambu project: component (path, objectid) -> top-level object id; the part id is the objectid.
        top = data["3D/3dmodel.model"].decode("utf-8")
        text = data[cfg].decode("utf-8")
        for om in OBJ.finditer(top):
            for cpath, cid in re.findall(r'path="/?([^"]+)"\s+objectid="(\d+)"', om.group(3)):
                d = changed.get(cpath, {}).get(cid)
                if not d:
                    continue
                obj = re.search(rf'<object id="{om.group(2)}">.*?</object>', text, re.S)
                block = obj.group(0)
                block = re.sub(r'(<metadata face_count=")(\d+)', lambda m: m.group(1) + str(int(m.group(2)) + d),
                               block, count=1)
                part = re.search(rf'<part id="{cid}".*?</part>', block, re.S)
                pblock = re.sub(r'(<mesh_stat face_count=")(\d+)',
                                lambda m: m.group(1) + str(int(m.group(2)) + d), part.group(0), count=1)
                block = block.replace(part.group(0), pblock)
                text = text.replace(obj.group(0), block)
                print(f"  model_settings: object {om.group(2)} part {cid} face_count +{d}")
        data[cfg] = text.encode("utf-8")
    tmp = path.with_suffix(".tmp3mf")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for i in infos:
            z.writestr(i, data[i.filename], compress_type=zipfile.ZIP_DEFLATED)
    tmp.replace(path)


def fix_stl(path: Path) -> None:
    """Binary STL: weld the triangle soup by exact coordinates, repair, write it back (normals recomputed)."""
    print(path)
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    rec = np.frombuffer(raw[84:84 + n * 50],
                        dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    verts, inv = np.unique(rec["v"].reshape(-1, 3), axis=0, return_inverse=True)
    tris = [tuple(int(i) for i in t) for t in inv.reshape(-1, 3)]
    new_t, _, before, after = repair(verts.astype(float), tris, [""] * len(tris))
    print(f"  {len(tris)} triangles, open edges {before} -> {after}, +{len(new_t) - len(tris)} triangles")
    if len(new_t) == len(tris):
        return
    p = verts[np.array(new_t)]  # float32 coordinates exactly as in the source
    nrm = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    out = np.zeros(len(new_t), dtype=rec.dtype)
    out["n"], out["v"] = nrm, p
    path.write_bytes(raw[:80] + len(new_t).to_bytes(4, "little") + out.tobytes())


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        (fix_stl if arg.lower().endswith(".stl") else fix_3mf)(Path(arg))
