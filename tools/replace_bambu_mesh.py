"""Swap meshes inside a Bambu Studio project 3MF, keeping its plates, positions and settings.

Use from Python:  replace(project_path, {"3D/Objects/object_24.model": (verts, tris), ...})

For each sub-model the new mesh is turned (one of the 24 axis-aligned rotations) and moved so its bounding box
matches the old mesh, then accepted only if it is the same part: the old vertices must lie on the new mesh
(median distance from the old vertices to the new surface) and the volumes must agree within `vol_tol`. Meant for small edits such as a
looser hole; a part whose outside changed is refused. Face counts in Metadata/model_settings.config are updated;
every other entry of the zip is copied unchanged. The file is rewritten through a temporary file.
"""
from __future__ import annotations

import itertools
import re
import zipfile
from pathlib import Path

import numpy as np

VERT = re.compile(r'<vertex\s+x="([^"]+)"\s+y="([^"]+)"\s+z="([^"]+)"\s*/>')
TRI = re.compile(r'<triangle\s+v1="(\d+)"\s+v2="(\d+)"\s+v3="(\d+)"[^/]*/>')


def rotations() -> list[np.ndarray]:
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                m[i, j] = s
            if np.linalg.det(m) > 0:
                out.append(m)
    return out


def volume(v: np.ndarray, t: np.ndarray) -> float:
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6)


def to_surface(p: np.ndarray, v: np.ndarray, t: np.ndarray, chunk: int = 64) -> np.ndarray:
    """Distance from each point p to the triangle mesh (v, t): closest point on each triangle (Ericson)."""
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    ab, ac = b - a, c - a
    out = []
    for i in range(0, len(p), chunk):
        q = p[i:i + chunk, None, :]
        ap = q - a
        d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
        bp = q - b
        d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
        cp = q - c
        d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        with np.errstate(divide="ignore", invalid="ignore"):
            den = va + vb + vc
            sv, sw = vb / den, vc / den
            closest = a + ab * sv[..., None] + ac * sw[..., None]  # inside the face
            # outside: nearest point on each edge, take the closest of the three
            def seg(p0, d):
                s = np.clip(((q - p0) * d).sum(-1) / np.maximum((d * d).sum(-1), 1e-18), 0, 1)
                return p0 + d * s[..., None]
            e = [seg(a, ab), seg(a, ac), seg(b, c - b)]
        inside = (va >= 0) & (vb >= 0) & (vc >= 0) & np.isfinite(sv) & np.isfinite(sw)
        dist = np.where(inside, np.linalg.norm(q - closest, axis=-1), np.inf)
        for x in e:
            dist = np.minimum(dist, np.linalg.norm(q - x, axis=-1))
        out.append(dist.min(axis=1))
    return np.concatenate(out)


def align(old_v, new_v, new_t, size_tol=0.15):
    """Best (moved new vertices, median, max distance of the old vertices to the new surface) over the 24
    rotations, or None."""
    lo, hi = old_v.min(0), old_v.max(0)
    best = None
    for r in rotations():
        nv = new_v @ r.T
        if np.abs(np.ptp(nv, 0) - (hi - lo)).max() > size_tol:
            continue
        nv = nv + ((lo + hi) / 2 - (nv.min(0) + nv.max(0)) / 2)
        d = to_surface(old_v, nv, new_t)
        if best is None or np.median(d) < best[1]:
            best = (nv, float(np.median(d)), float(d.max()))
    return best


def replace(path: Path, meshes: dict, vol_tol: float = 0.03, med_tol: float = 0.02, same_part: bool = True) -> None:
    """same_part=False swaps in a reshaped part on purpose: the bounding boxes may differ by up to 1 mm and the
    same-part test is only reported; the rotation is still the one that best fits the old vertices (keep the
    holes unchanged so they pick the orientation)."""
    path = Path(path)
    with zipfile.ZipFile(path) as z:
        infos = z.infolist()
        data = {i.filename: z.read(i.filename) for i in infos}
    counts = {}
    for sub, (new_v, new_t) in meshes.items():
        xml = data[sub].decode("utf-8")
        old_v = np.array(VERT.findall(xml), float)
        old_t = np.array(TRI.findall(xml), int)
        new_t = np.asarray(new_t)
        got = align(old_v, np.asarray(new_v, float), new_t, 0.15 if same_part else 1.0)
        if got is None:
            raise SystemExit(f"{path.name} {sub}: no rotation matches the old bounding box {np.ptp(old_v, 0)}")
        nv, med, mx = got
        v_old, v_new = volume(old_v, old_t), volume(nv, np.asarray(new_t))
        print(f"  {sub}: {len(old_t)} -> {len(new_t)} triangles, old vertices off new surface median {med:.3f} "
              f"max {mx:.3f} mm, volume {v_old:.1f} -> {v_new:.1f} mm3")
        if same_part and (med > med_tol or abs(v_new - v_old) > vol_tol * abs(v_old)):
            raise SystemExit(f"{path.name} {sub}: not the same part (refused)")
        vx = "\n".join(f'     <vertex x="{a:.6g}" y="{b:.6g}" z="{c:.6g}"/>' for a, b, c in nv)
        tx = "\n".join(f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in new_t)
        xml = re.sub(r"<vertices>.*?</vertices>", lambda m: f"<vertices>\n{vx}\n    </vertices>", xml, count=1, flags=re.S)
        xml = re.sub(r"<triangles>.*?</triangles>", lambda m: f"<triangles>\n{tx}\n    </triangles>", xml, count=1,
                     flags=re.S)
        data[sub] = xml.encode("utf-8")
        counts[sub] = len(new_t)
    cfg = "Metadata/model_settings.config"
    if cfg in data:
        top = data["3D/3dmodel.model"].decode("utf-8")
        text = data[cfg].decode("utf-8")
        for oid, body in re.findall(r'<object id="(\d+)"[^>]*>(.*?)</object>', top, re.S):
            for cpath, cid in re.findall(r'path="/?([^"]+)"\s+objectid="(\d+)"', body):
                if cpath not in counts:
                    continue
                n = counts[cpath]
                obj = re.search(rf'<object id="{oid}">.*?</object>', text, re.S)
                block = re.sub(r'(<metadata face_count=")\d+', rf"\g<1>{n}", obj.group(0), count=1)
                block = re.sub(rf'(<part id="{cid}".*?<mesh_stat face_count=")\d+', rf"\g<1>{n}", block, count=1,
                               flags=re.S)
                text = text.replace(obj.group(0), block)
        data[cfg] = text.encode("utf-8")
    tmp = path.with_suffix(".tmp3mf")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for i in infos:
            z.writestr(i, data[i.filename], compress_type=zipfile.ZIP_DEFLATED)
    tmp.replace(path)
