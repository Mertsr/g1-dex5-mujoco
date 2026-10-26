#!/usr/bin/env python3
"""Occupancy grid of a conveyor world for Nav2's localization, straight from the scene geometry.

    make_conveyor_map.py --model-dir <unitree_mujoco>/unitree_robots/g1 \\
        --scene g1_bringup/mjcf/g1_conveyor_fixed_scene.xml --out g1_navigation/maps/conveyor

The scene is staged into the robot model's directory (it <include>s g1_29dof.xml from there)
and loaded with the robot at its spawn pose; the robot itself is left out of the map.

A cell is occupied where any geom the simulated LiDAR can see (group 2, sim_sensors.yaml's
scene_geom_group) reaches into the height band the scan pipeline keeps (g1_navigation's
scan.yaml: 0.30-1.50 m above the floor) -- the outline, seen from above, of the part of the
object in that band (its convex hull: boxes, rollers and the panels' meshes are what the world
is made of). Everything else inside the extent is free. That is what slam_toolbox would build
from the same scans, without driving the robot round to build it; the world spawns the robot
at the origin, which is where the map's own origin puts the AMCL initial pose (0, 0, 0).

Re-run whenever the conveyor scenes' fixed geometry changes. The conveyor variants (conveyor,
conveyor_flat, conveyor_fixed, conveyor2) share it: the table, the belt and the pallet are the
same; only the parts on the table differ, and those are small next to them.
"""

import argparse
import os
import shutil

import mujoco
import numpy as np
from matplotlib.path import Path
from scipy.spatial import ConvexHull

BAND = (0.30, 1.50)  # g1_navigation/config/scan.yaml min_height / max_height
LIDAR_GROUP = 2  # vendor sensor_publisher.cc scene_geom_group
RES = 0.05  # m per cell, as maps/facility
MARGIN = 3.0  # m of free floor round the objects


def geom_points(m, d, g):
    """World-frame points outlining geom g (corners, rims or mesh vertices)."""
    t = m.geom_type[g]
    size = m.geom_size[g]
    pos = d.geom_xpos[g]
    rot = d.geom_xmat[g].reshape(3, 3)
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * size
    elif t in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        a = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        r, h = size[0], size[1] + (size[0] if t == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0)
        ring = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        local = np.vstack([np.hstack([ring, np.full((24, 1), z)]) for z in (-h, h)])
    elif t in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        a = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        rx, ry, rz = (size[0],) * 3 if t == mujoco.mjtGeom.mjGEOM_SPHERE else size
        local = np.vstack([np.stack([rx * np.cos(a), ry * np.sin(a), np.full(24, z)], axis=1) for z in (-rz, rz)])
    elif t == mujoco.mjtGeom.mjGEOM_MESH:
        k = m.geom_dataid[g]
        local = m.mesh_vert[m.mesh_vertadr[k]: m.mesh_vertadr[k] + m.mesh_vertnum[k]]
    else:
        return None
    return local @ rot.T + pos


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model-dir", required=True, help="unitree_mujoco's unitree_robots/g1")
    p.add_argument("--scene", required=True, help="a conveyor scene from g1_bringup/mjcf")
    p.add_argument("--out", required=True, help="output path without extension (.pgm and .yaml)")
    a = p.parse_args()

    staged = os.path.join(a.model_dir, "conveyor_map.staged.xml")
    shutil.copyfile(a.scene, staged)
    try:
        m = mujoco.MjModel.from_xml_path(staged)
    finally:
        os.remove(staged)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    robot = m.body("pelvis").id

    shapes = []
    for g in range(m.ngeom):
        if m.geom_group[g] != LIDAR_GROUP or m.body_rootid[m.geom_bodyid[g]] == robot:
            continue
        pts = geom_points(m, d, g)
        if pts is None:
            continue  # the floor plane
        if pts[:, 2].max() < BAND[0] or pts[:, 2].min() > BAND[1]:
            continue
        shapes.append(pts[:, :2])
    allpts = np.vstack(shapes + [np.zeros((1, 2))])
    lo = np.floor((allpts.min(axis=0) - MARGIN) / RES) * RES
    hi = np.ceil((allpts.max(axis=0) + MARGIN) / RES) * RES
    w, h = int(round((hi[0] - lo[0]) / RES)), int(round((hi[1] - lo[1]) / RES))
    xs = lo[0] + (np.arange(w) + 0.5) * RES
    ys = lo[1] + (np.arange(h) + 0.5) * RES
    cx, cy = np.meshgrid(xs, ys)
    centers = np.stack([cx.ravel(), cy.ravel()], axis=1)
    occ = np.zeros(w * h, dtype=bool)
    for pts in shapes:
        try:
            hull = pts[ConvexHull(pts).vertices]
            occ |= Path(hull).contains_points(centers, radius=RES * 0.5)
        except Exception:  # degenerate (flat) outline: its points' cells
            pass
        ij = np.floor((pts - lo) / RES).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
        occ[ij[ok, 1] * w + ij[ok, 0]] = True
    grid = np.where(occ.reshape(h, w), 0, 254).astype(np.uint8)[::-1]  # pgm row 0 is the top

    with open(a.out + ".pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(grid.tobytes())
    with open(a.out + ".yaml", "w") as f:
        f.write(
            f"image: {os.path.basename(a.out)}.pgm\nmode: trinary\nresolution: {RES}\n"
            f"origin: [{lo[0]:.2f}, {lo[1]:.2f}, 0]\nnegate: 0\noccupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )
    print(f"{w} x {h} cells at {RES} m, origin ({lo[0]:.2f}, {lo[1]:.2f}), "
          f"{int(occ.sum())} occupied, from {len(shapes)} geoms")


if __name__ == "__main__":
    main()
