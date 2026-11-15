"""Add a keychain loop on top of Hello Kitty Mini's head as a second part of the same object.

Writes a new 3MF; the source file is left untouched. Needs: pip install manifold3d numpy

Units: the 3MF object is stored at 2x and scaled by 0.5 in the build item, so all loop sizes
below are given in printed millimetres and converted to the object's local frame.
"""
import re
import sys
import uuid
import zipfile

import numpy as np
from manifold3d import CrossSection, Manifold

SRC = "Hello_kitty_mini.3mf"
DST = "Hello_kitty_mini_keyring.3mf"

SCALE = 0.499999504  # build item scale of the figure (local -> printed mm)

# loop, printed mm
LOOP_WIDTH = 8.0  # front-to-back (along Y)
LOOP_HOLE = 3.5  # hole diameter, fits a jump ring or key-chain link
LOOP_THICK = 2.5  # left-to-right (along X); hole axis is along X
HOLE_CLEAR = 0.5  # hole bottom above the head surface
EMBED = 0.75  # tab bottom below the lowest head point under it

# placement, local frame: centred in the gap between the left ear (x < -8.5) and the bow (x > -2)
LOOP_X = -5.25
LOOP_Y = 0.0
HEAD_TOP_UNDER_HOLE = 20.6  # max head z under the hole footprint (measured)
HEAD_LOW_UNDER_TAB = 17.6  # lowest head z under the tab footprint (measured)


def loop_mesh():
    R = LOOP_WIDTH / 2 / SCALE
    r = LOOP_HOLE / 2 / SCALE
    th = LOOP_THICK / SCALE
    zc = HEAD_TOP_UNDER_HOLE + HOLE_CLEAR / SCALE + r
    zb = HEAD_LOW_UNDER_TAB - EMBED / SCALE
    # profile in (y, z): round top + straight sides down into the head, minus the hole
    outline = CrossSection.circle(R, 96).translate((0, zc)) + CrossSection.square((2 * R, zc - zb)).translate((-R, zb))
    profile = outline - CrossSection.circle(r, 64).translate((0, zc))
    solid = profile.extrude(th)  # (u, v, w) = (y, z, x)
    mesh = solid.to_mesh()
    uvw = np.asarray(mesh.vert_properties)[:, :3]
    xyz = np.column_stack([uvw[:, 2] + LOOP_X - th / 2, uvw[:, 0] + LOOP_Y, uvw[:, 1]])  # cyclic: keeps orientation
    return xyz, np.asarray(mesh.tri_verts), solid, (zb, zc, R, r, th)


def mesh_xml(obj_id, verts, tris):
    vx = "".join(f'     <vertex x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>\n' for x, y, z in verts)
    tx = "".join(f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>\n' for a, b, c in tris)
    return (
        f'  <object id="{obj_id}" p:UUID="{uuid.uuid4()}" type="model">\n   <mesh>\n    <vertices>\n{vx}    </vertices>\n'
        f"    <triangles>\n{tx}    </triangles>\n   </mesh>\n  </object>\n"
    )


def main():
    verts, tris, solid, (zb, zc, R, r, th) = loop_mesh()
    zin = zipfile.ZipFile(SRC)
    with zipfile.ZipFile(DST, "w", zipfile.ZIP_DEFLATED) as zout:
        for info in zin.infolist():
            data = zin.read(info.filename)
            if info.filename == "3D/Objects/object_1.model":
                s = data.decode("utf-8")
                assert s.count(" </resources>") == 1
                s = s.replace(" </resources>", mesh_xml(3, verts, tris) + " </resources>")
                data = s.encode("utf-8")
            elif info.filename == "3D/3dmodel.model":
                s = data.decode("utf-8")
                comp = '<component p:path="/3D/Objects/object_1.model" objectid="1"'
                assert s.count(comp) == 1
                new = f'<component p:path="/3D/Objects/object_1.model" objectid="3" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n    '
                s = s.replace("    " + comp, "    " + new + comp)
                data = s.encode("utf-8")
            elif info.filename == "Metadata/model_settings.config":
                s = data.decode("utf-8")
                part = (
                    f'    <part id="3" subtype="normal_part" uuid="{uuid.uuid4()}">\n'
                    '      <metadata key="name" value="keyring_loop"/>\n'
                    '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                    '      <metadata key="extruder" value="1"/>\n'
                    f'      <mesh_stat face_count="{len(tris)}" edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
                    "    </part>\n"
                )
                assert s.count("  </object>") == 1
                s = s.replace("  </object>", part + "  </object>")
                s = re.sub(r'(<assemble_item object_id="2" volume_id="0"[^>]*/>)', r'\1\n   <assemble_item object_id="2" volume_id="1" transform="1 0 0 0 1 0 0 0 1 0 0 0" />', s)
                data = s.encode("utf-8")
            zout.writestr(info, data, compress_type=zipfile.ZIP_DEFLATED)
    print(f"wrote {DST}: loop {len(tris)} triangles, manifold status {solid.status().name}")
    print(f"local: x {LOOP_X - th / 2:.2f}..{LOOP_X + th / 2:.2f}, y +/-{R:.2f}, z {zb:.2f}..{zc + R:.2f}, hole centre z {zc:.2f}")
    top = (zc + R) * SCALE + 12.5
    print(f"printed: loop {LOOP_WIDTH} x {LOOP_THICK} mm, hole {LOOP_HOLE} mm, figure height 25.0 -> {top:.2f} mm")


if __name__ == "__main__":
    sys.exit(main())
