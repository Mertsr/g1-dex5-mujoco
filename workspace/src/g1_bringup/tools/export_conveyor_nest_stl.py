#!/usr/bin/env python3
"""Export one pallet nest -- its two end cradles -- as STL.

Built from make_conveyor_pallet.py's own constants, so the part and the simulated nest cannot
drift apart: change a dimension there, rerun both. Each cradle is one solid (pad, end wall, two
stops and the 3 mm base slab, unioned), so it opens cleanly in CAD and slices for printing.

Writes two files:
  BSH_nest.stl          metres, in BSH_panel.stl's OWN coordinate frame: open both together and
                        the panel sits seated in the nest. For checking fit in CAD.
  BSH_nest_zup_mm.stl   millimetres, z up, the pallet plate's top surface at z = 0, nest centre
                        at x = y = 0, x along the conveyor's travel. For making the part.

    export_conveyor_nest_stl.py --clearance-mm 2.0 --out-dir .

Needs numpy, trimesh and manifold3d (pip install trimesh manifold3d).
"""

import argparse
import importlib.util
import pathlib
import sys

import numpy as np
import trimesh

# BSH_panel.stl frame <-> seated nest frame. The scene turns the panel with
# euler="-1.5708 1.57 0" and shifts the mesh by refpos; in exact 90-degree form that is
# W = R (stl - refpos), W measured from the panel origin with world axes.
R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
REFPOS = np.array([0.0, 0.001, -0.046])


def load_generator():
    path = pathlib.Path(__file__).resolve().parent / "make_conveyor_pallet.py"
    spec = importlib.util.spec_from_file_location("make_conveyor_pallet", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def box(half, centre):
    mesh = trimesh.creation.box(extents=2 * np.asarray(half, dtype=float))
    mesh.apply_translation(centre)
    return mesh


def build(g, clearance):
    """One nest's two cradles, in the nest frame: x along travel, y across, z up from the plate."""
    pitch = 2 * (g.PART_HALF_ALONG + clearance) + 2 * g.STOP_HALF_T
    wall_y = g.PART_HALF_ACROSS + clearance + g.WALL_HALF_T
    wall_h = g.PAD_HEIGHT + g.WALL_ABOVE
    stop_h = g.PAD_HEIGHT + g.STOP_ABOVE
    pad_half = ((g.PAD_FRONT_X - g.PAD_BACK_X) / 2, (g.PART_HALF_ACROSS - g.PAD_INNER) / 2, g.PAD_HEIGHT / 2)
    pad_cx = (g.PAD_FRONT_X + g.PAD_BACK_X) / 2
    pad_cy = (g.PART_HALF_ACROSS + g.PAD_INNER) / 2
    stop_half_y = (g.PART_HALF_ACROSS + clearance - g.PAD_INNER) / 2
    stop_cy = (g.PART_HALF_ACROSS + clearance + g.PAD_INNER) / 2
    base_half = (pitch / 2 + g.STOP_HALF_T, (wall_y + g.WALL_HALF_T - g.PAD_INNER) / 2, g.BASE_T / 2)
    base_cy = (wall_y + g.WALL_HALF_T + g.PAD_INNER) / 2

    cradles = []
    for s in (-1, 1):
        parts = [
            box(base_half, (0, s * base_cy, g.BASE_T / 2)),
            box(pad_half, (pad_cx, s * pad_cy, g.PAD_HEIGHT / 2)),
            box((pitch / 2, g.WALL_HALF_T, wall_h / 2), (0, s * wall_y, wall_h / 2)),
            box((g.STOP_HALF_T, stop_half_y, stop_h / 2), (pitch / 2, s * stop_cy, stop_h / 2)),
            box((g.STOP_HALF_T, stop_half_y, stop_h / 2), (-pitch / 2, s * stop_cy, stop_h / 2)),
        ]
        cradle = trimesh.boolean.union(parts, engine="manifold")
        if not cradle.is_watertight or len(cradle.split()) != 1:
            raise SystemExit("a cradle did not union into one watertight solid")
        cradles.append(cradle)
    return trimesh.util.concatenate(cradles), pitch


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--clearance-mm", type=float, default=2.0)
    parser.add_argument("--out-dir", type=pathlib.Path, default=pathlib.Path("."))
    args = parser.parse_args(argv)
    g = load_generator()
    nest, pitch = build(g, args.clearance_mm / 1000.0)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    seated_origin = np.array([0.0, 0.0, g.PAD_HEIGHT + g.FEET_BELOW])  # panel origin, nest frame
    to_panel = np.eye(4)
    to_panel[:3, :3] = R.T
    to_panel[:3, 3] = REFPOS - R.T @ seated_origin
    in_panel_frame = nest.copy()
    in_panel_frame.apply_transform(to_panel)
    in_panel_frame.export(args.out_dir / "BSH_nest.stl")

    zup = nest.copy()
    zup.apply_scale(1000.0)
    zup.export(args.out_dir / "BSH_nest_zup_mm.stl")

    ext = zup.extents
    print(f"wrote {args.out_dir}/BSH_nest.stl and BSH_nest_zup_mm.stl: two cradles, clearance "
          f"{args.clearance_mm:g} mm, nest pitch {pitch * 1000:.1f} mm, overall "
          f"{ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
