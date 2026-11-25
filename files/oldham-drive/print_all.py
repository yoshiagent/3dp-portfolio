"""Every printed part of the differential + Oldham coupling + TT motor module, one Bambu Studio project:
3MF/oldham_drive_plates.3mf (bed 256 x 256, Bambu Lab X2D settings copied from ../../differential_model).

  1 base + motor frame   drive base, TT motor frame (SideTop, the closed STL from ../../tt_motor_adapter)
  2 drive train          ring 44T, pinion 11T, carrier, cross pin, side gears x2, spider gears x2, inner sleeves x2,
                         Oldham coupling (motor hub, slider, axle hub)
Not printed: TT motor, Technic 2L pins, LEGO 2L and 10L x2 cross axles, half bushes x4, bushes x2, LEGO motorcycle
wheels (tyre 2902 on rim 2903) x2.

Adapted from ../../car_model/src/print_all.py (same mesher, T-junction repair and 3MF writer).
Run with cadgen's python from oldham_drive/:  python src/print_all.py
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
import uuid
import zipfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "web" / "tools"))

import drive_parts as c  # noqa: E402
from fix_tjunctions import boundary, repair  # noqa: E402

OUT = HERE.parent / "3MF" / "oldham_drive_plates.3mf"
TEMPLATE = ROOT / "differential_model" / "differential_model.3mf"  # project settings (printer, bed)
TT = ROOT / "tt_motor_adapter" / "out"
BED, GAP = 256.0, 15.0  # gap: room for both neighbours' 5 mm auto brims
STRIDE = BED * 1.2  # Bambu Studio lays plates out on a grid this far apart
# usable area: the X2D's second nozzle only reaches x >= 20.5 (extruder_printable_area), plus 5 mm auto brim
AREA = (27.0, 250.0, 8.0, 248.0)  # x0, x1, y0, y1


def _module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = path.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    v, inv = np.unique(rec["v"].reshape(-1, 3), axis=0, return_inverse=True)
    return v.astype(float), inv.reshape(-1, 3)


def mesh(part, tol: float = 0.02, ang: float = 0.15) -> tuple[np.ndarray, np.ndarray]:
    """build123d part -> welded, T-junction-free triangle mesh."""
    verts, tris = part.tessellate(tol, ang)
    v = np.array([(p.X, p.Y, p.Z) for p in verts])
    key = np.round(v, 5)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    t = [tuple(int(inv[i]) for i in tri) for tri in tris]
    t = [x for x in t if len(set(x)) == 3]
    t, _, _, after = repair(uniq, t, [""] * len(t))
    if after:
        raise SystemExit(f"{part.label}: {after} open edges left")
    return uniq, np.array(t)


def flat(v: np.ndarray) -> np.ndarray:
    return v - v.min(axis=0)


def plate_parts() -> list[tuple[str, list[tuple[str, np.ndarray, np.ndarray]]]]:
    X, Z = (1, 0, 0), (0, 0, 1)
    pose = c.d.print_pose
    ep = _module("electric_print_parts", ROOT / "electric_differential" / "src" / "print_parts.py")

    def b3d(label, part):
        v, t = mesh(part)
        print(f"  {label:22s} {len(t):7d} triangles")
        return label, flat(v), t

    gears = dict(ep.gear_plate_parts())
    up_front = (0, 1, 0)  # vehicle +y up: the motor hub's slot face on the bed, the axle hub's slot face up
    frame_v, frame_t = read_stl(TT / "tt_motor_technic_SideTop.stl")
    return [
        ("1 底板與馬達外框 base + motor frame", [b3d("drive_base", pose(c.drive_base(), x_dir=X, z_dir=Z)),
                                              ("tt_motor_frame_SideTop", flat(frame_v), frame_t)]),
        ("2 差速器與聯軸器 drive train", [b3d(k, gears[k]) for k in (
            "ring_half", "pinion", "carrier_left", "cross_pin", "side_gear_1", "side_gear_2", "spider_1", "spider_2",
            "sleeve_right", "sleeve_left")] + [
            b3d("oldham_hub_motor", pose(c.oldham_hub_motor(), x_dir=X, z_dir=up_front)),
            b3d("oldham_hub_axle", pose(c.oldham_hub_axle(), x_dir=X, z_dir=up_front)),
            b3d("oldham_slider", pose(c.oldham_slider(), x_dir=X, z_dir=Z))]),  # slider standing on an edge
    ]


def pack(parts, width: float = AREA[1] - AREA[0]):
    """Shelf packing, deepest first; returns [(name, v moved, t)] centred in AREA on the origin plate."""
    parts = sorted(parts, key=lambda p: -np.ptp(p[1][:, 1]))
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    for name, v, t in parts:
        sx, sy = np.ptp(v[:, 0]), np.ptp(v[:, 1])
        if x and x + sx > width:
            x, y, row_h = 0.0, y + row_h + GAP, 0.0
        placed.append((name, v + (x, y, 0), t))
        x += sx + GAP
        row_h = max(row_h, sy)
    allv = np.vstack([v for _, v, _ in placed])
    size = np.ptp(allv[:, :2], axis=0)
    if size[0] > width + 1e-6 or size[1] > AREA[3] - AREA[2] + 1e-6:
        raise SystemExit(f"plate too full: {size}")
    centre = np.array([(AREA[0] + AREA[1]) / 2, (AREA[2] + AREA[3]) / 2, 0])
    shift = centre - np.array([*(allv.min(0)[:2] + allv.max(0)[:2]) / 2, 0])
    return [(name, v + shift, t) for name, v, t in placed], size


def write(plates, path: Path) -> None:
    ns = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
          'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
          'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"')
    cols = math.ceil(math.sqrt(len(plates)))
    files, top_objs, items, rels, cfg_objs, cfg_plates = {}, [], [], [], [], []
    oid = 0
    for k, (pname, parts) in enumerate(plates):
        px, py = (k % cols) * STRIDE, -(k // cols) * STRIDE
        instances = []
        for name, v, t in parts:
            oid += 2  # top-level object id; its mesh (sub-model object, part) is oid - 1, unique as Bambu expects
            centre = (v.min(0) + v.max(0)) / 2
            local = v - centre
            sub = f"3D/Objects/object_{oid}.model"
            vx = "".join(f'<vertex x="{a:.5f}" y="{b:.5f}" z="{c_:.5f}"/>' for a, b, c_ in local)
            tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c_}"/>' for a, b, c_ in t)
            files[sub] = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>'
                          '<metadata name="BambuStudio:3mfVersion">1</metadata><resources>'
                          f'<object id="{oid - 1}" p:UUID="{uuid.uuid4()}" type="model"><mesh><vertices>{vx}</vertices>'
                          f'<triangles>{tx}</triangles></mesh></object></resources><build/></model>')
            top_objs.append(f'<object id="{oid}" p:UUID="{uuid.uuid4()}" type="model"><components>'
                            f'<component p:path="/{sub}" objectid="{oid - 1}" p:UUID="{uuid.uuid4()}" '
                            'transform="1 0 0 0 1 0 0 0 1 0 0 0"/></components></object>')
            items.append(f'<item objectid="{oid}" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 '
                         f'{centre[0] + px:.5f} {centre[1] + py:.5f} {centre[2]:.5f}" printable="1"/>')
            rels.append(f'<Relationship Target="/{sub}" Id="rel-{len(rels) + 1}" '
                        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>')
            cfg_objs.append(
                f'  <object id="{oid}">\n    <metadata key="name" value="{name}"/>\n'
                f'    <metadata key="extruder" value="1"/>\n    <metadata face_count="{len(t)}"/>\n'
                f'    <part id="{oid - 1}" subtype="normal_part">\n      <metadata key="name" value="{name}"/>\n'
                '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                f'      <mesh_stat face_count="{len(t)}" edges_fixed="0" degenerate_facets="0" facets_removed="0" '
                'facets_reversed="0" backwards_edges="0"/>\n    </part>\n  </object>\n')
            instances.append(f'    <model_instance>\n      <metadata key="object_id" value="{oid}"/>\n'
                             '      <metadata key="instance_id" value="0"/>\n'
                             f'      <metadata key="identify_id" value="{100 + oid}"/>\n    </model_instance>\n')
        cfg_plates.append(f'  <plate>\n    <metadata key="plater_id" value="{k + 1}"/>\n'
                          f'    <metadata key="plater_name" value="{pname}"/>\n'
                          '    <metadata key="locked" value="false"/>\n'
                          '    <metadata key="filament_map_mode" value="Auto For Flush"/>\n'
                          '    <metadata key="filament_maps" value="1 1 1 1 1"/>\n'
                          '    <metadata key="filament_volume_maps" value="0 0 0 0 0"/>\n'
                          + "".join(instances) + '  </plate>\n')
    files["3D/3dmodel.model"] = (
        f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>\n'
        ' <metadata name="Application">BambuStudio-02.08.02.61</metadata>\n'
        ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
        ' <metadata name="Title">Differential + Oldham coupling + TT motor: all printed parts</metadata>\n'
        f' <resources>{"".join(top_objs)}</resources>\n <build>{"".join(items)}</build>\n</model>\n')
    files["3D/_rels/3dmodel.model.rels"] = (
        '<?xml version="1.0" encoding="UTF-8"?>\n<Relationships '
        f'xmlns="http://schemas.openxmlformats.org/package/2006/relationships">{"".join(rels)}</Relationships>\n')
    files["Metadata/model_settings.config"] = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n'
                                               + "".join(cfg_objs) + "".join(cfg_plates) + '</config>\n')
    files["[Content_Types].xml"] = (
        '<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
        '<Default Extension="png" ContentType="image/png"/></Types>\n')
    files["_rels/.rels"] = (
        '<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel-1" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>\n')
    with zipfile.ZipFile(TEMPLATE) as z:
        settings = json.loads(z.read("Metadata/project_settings.config"))
    # single-colour plates: no prime tower (the template's tower, placed for its own 2 plates, collides here)
    settings["enable_prime_tower"] = "0"
    settings["wipe_tower_x"] = ["15"] * len(plates)
    settings["wipe_tower_y"] = ["220"] * len(plates)
    files["Metadata/project_settings.config"] = json.dumps(settings, indent=4, ensure_ascii=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp3mf")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for name in ("[Content_Types].xml", "_rels/.rels", "3D/3dmodel.model", "3D/_rels/3dmodel.model.rels"):
            z.writestr(name, files.pop(name))
        for name, data in files.items():
            z.writestr(name, data)
    tmp.replace(path)


if __name__ == "__main__":
    plates = []
    for pname, parts in plate_parts():
        for name, v, t in parts:
            n_open = len(boundary([tuple(x) for x in t.tolist()]))
            if n_open:
                raise SystemExit(f"{name}: {n_open} open edges")
        placed, size = pack(parts)
        plates.append((pname, placed))
        print(f"{pname}: {len(placed)} parts, footprint {size[0]:.1f} x {size[1]:.1f} mm, "
              f"tallest {max(v[:, 2].max() for _, v, _ in placed):.1f} mm")
    write(plates, OUT)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1e6:.1f} MB), {sum(len(p) for _, p in plates)} parts on {len(plates)} plates")
