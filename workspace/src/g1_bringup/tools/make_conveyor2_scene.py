#!/usr/bin/env python3
"""Generate g1_conveyor2_scene.xml: the conveyor world with a stand-in part.

world:=conveyor2 is g1_conveyor_scene.xml exactly -- tray, conveyor, pallet, robot, everything --
except that every BSH panel (part_tray_1..4 and part_drop_test) is replaced by a plain flat panel
standing on four legs. It exists to build and exercise the pick-and-place system (tracking, Pick,
Place into a nest) with a part the hand can certainly hold, while the real panel's grasp waits for
the real hand. The bodies keep their names, so everything that looks them up still finds them.

The flat panel, in its body frame (world-aligned at rest, no rotation):
  plate  PLATE_HALF_X x PLATE_HALF_Y x PLATE_HALF_T half-extents: the BSH panel's own footprint
         (0.0683 along the line, 0.298 across), 6 mm thick
  legs   four 16 x 16 mm posts LEG_H tall under it, at y = +-LEG_Y and x = LEG_X_BACK / LEG_X_FRONT:
         inside the pallet's end pads (|y| 0.255..0.298, x -0.066..+0.037 about the nest centre),
         so the part stands on the pads exactly like the BSH panel's feet do, and well in from the
         near edge, so the middle of that edge is free to be taken from below
  origin FEET_BELOW above the feet, as for the BSH panel, so a seated part's origin lands in the
         pallet's nest sites unchanged (pad top + 12.25 mm)

LEG_H puts the plate's underside LEG_H above whatever it stands on: 10 cm, so on the tray (floor
z 0.71, rim top 0.77) the plate is 4 cm above the rim and a hand coming in over the rim, fingers
below the plate and thumb on top, clears it.

Also writes the tracker's model of the same part (g1_perception/models/flat_panel.stl, in this
body frame, metres) so the depth tracker and the simulator agree on the shape exactly.

    make_conveyor2_scene.py                 # default paths
"""

import argparse
import pathlib
import re
import struct

import numpy as np

PLATE_HALF_X = 0.0683
PLATE_HALF_Y = 0.298
PLATE_HALF_T = 0.003
LEG_HALF = 0.008
LEG_H = 0.100
LEG_Y = 0.280
LEG_X_BACK = -0.058
LEG_X_FRONT = 0.029
FEET_BELOW = 0.01225
MASS = 0.3
LEG_MASS = 0.01

# Where the four tray parts stand (x as in g1_conveyor_scene.xml; feet on the tray floor, top
# z 0.71) and where the drop-test part starts (above the load station's nest, feet 10 mm above
# the tallest cradle wall, z 0.842, so it falls in when the simulation starts).
TRAY_FLOOR = 0.71
TRAY_X = {1: 1.605, 2: 1.755, 3: 1.905, 4: 2.055}
TRAY_Y = -0.80
DROP_POS = (0.40, 0.60, 0.842 + 0.010 + FEET_BELOW)


def boxes():
    """(centre, half-extents) of every box, body frame"""
    plate_z = -FEET_BELOW + LEG_H + PLATE_HALF_T
    out = [((0.0, 0.0, plate_z), (PLATE_HALF_X, PLATE_HALF_Y, PLATE_HALF_T))]
    for lx in (LEG_X_BACK, LEG_X_FRONT):
        for ly in (-LEG_Y, LEG_Y):
            out.append(((lx, ly, -FEET_BELOW + LEG_H / 2), (LEG_HALF, LEG_HALF, LEG_H / 2)))
    return out


def _fmt(v):
    return " ".join(f"{x:.5g}" for x in v)


def body_xml(name, pos):
    f = _fmt
    geoms = []
    for i, (c, h) in enumerate(boxes()):
        mass = MASS - 4 * LEG_MASS if i == 0 else LEG_MASS
        geoms.append(
            f'      <geom name="{name}_{"plate" if i == 0 else f"leg_{i}"}" group="2" type="box" '
            f'size="{f(h)}" pos="{f(c)}" material="mat_part" mass="{mass:.4g}" '
            f'contype="1" conaffinity="1" friction="1 0.01 0.0001"/>'
        )
    return (
        f'    <body name="{name}" pos="{f(pos)}">\n'
        f'      <joint type="free" name="{name}_joint" damping="0.2"/>\n'
        + "\n".join(geoms)
        + "\n    </body>"
    )


def write_stl(path):
    tris = []
    for c, h in boxes():
        c, h = np.array(c), np.array(h)
        v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
        # 6 faces, two triangles each, outward winding
        for a, b, cc, d in [
            (0, 1, 3, 2),
            (4, 6, 7, 5),
            (0, 4, 5, 1),
            (2, 3, 7, 6),
            (0, 2, 6, 4),
            (1, 5, 7, 3),
        ]:
            tris += [(v[a], v[b], v[cc]), (v[a], v[cc], v[d])]
    with open(path, "wb") as fh:
        fh.write(b"flat_panel (make_conveyor2_scene.py)".ljust(80, b" "))
        fh.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / np.linalg.norm(n)
            fh.write(struct.pack("<12fH", *n, *t[0], *t[1], *t[2], 0))


def main():
    here = pathlib.Path(__file__).resolve().parent
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--src", type=pathlib.Path, default=here.parent / "mjcf" / "g1_conveyor_scene.xml"
    )
    p.add_argument(
        "--dst", type=pathlib.Path, default=here.parent / "mjcf" / "g1_conveyor2_scene.xml"
    )
    p.add_argument(
        "--stl",
        type=pathlib.Path,
        default=here.parents[1] / "g1_perception" / "models" / "flat_panel.stl",
    )
    a = p.parse_args()
    s = a.src.read_text()
    bodies = {f"part_tray_{k}": (x, TRAY_Y, TRAY_FLOOR + FEET_BELOW) for k, x in TRAY_X.items()}
    bodies["part_drop_test"] = DROP_POS
    for name, pos in bodies.items():
        m = re.search(rf'    <body name="{name}"[^>]*>.*?\n    </body>', s, re.S)
        if not m:
            raise SystemExit(f"{a.src}: no body {name}")
        s = s[: m.start()] + body_xml(name, pos) + s[m.end() :]
    note = (
        "<!-- GENERATED by g1_bringup/tools/make_conveyor2_scene.py from g1_conveyor_scene.xml: the same\n"
        "     world with every BSH panel replaced by a flat panel on four legs (see that tool). Do not\n"
        "     edit by hand; edit g1_conveyor_scene.xml or the tool and regenerate. -->\n"
    )
    s = s.replace('<mujoco model="g1_conveyor_scene">', '<mujoco model="g1_conveyor2_scene">', 1)
    s = re.sub(r"(<mujoco[^>]*>\n)", r"\1" + note, s, count=1)
    a.dst.write_text(s)
    write_stl(a.stl)
    print(f"wrote {a.dst} and {a.stl}")


if __name__ == "__main__":
    main()
