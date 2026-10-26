# maps

`conveyor` is the conveyor line's room (`world:=conveyor`, `conveyor_flat`, `conveyor_fixed`,
`conveyor2`), built from the scene geometry rather than by driving the robot round:
`g1_bringup/tools/make_conveyor_map.py` marks every cell where something the simulated LiDAR sees
reaches into the scan band (0.30-1.50 m) -- the walls, the belt, the table with the panels. 186 by
174 cells at 5 cm, origin `[-4.05, -4.35]`; the robot spawns at the origin, which is AMCL's
initial pose. `bringup.launch.py` picks it for the conveyor worlds, and `facility` for
`world:=navigation`. The room is tight -- the robot spawns 0.26 m from the belt -- so in the
conveyor worlds Nav2 also loads `config/nav2_world_conveyor.yaml` (robot radius 0.22 m,
inflation 0.40 m, instead of 0.45 and 0.85).

```bash
python3 workspace/src/g1_bringup/tools/make_conveyor_map.py \
    --model-dir <unitree_mujoco>/unitree_robots/g1 \
    --scene workspace/src/g1_bringup/mjcf/g1_conveyor_fixed_scene.xml \
    --out workspace/src/g1_navigation/maps/conveyor
```

`facility` is the `g1_navigation_scene` world, mapped with `mode:=mapping` while the robot was
driven around the four rooms. 361 by 359 cells at 5 cm, origin `[-9.05, -8.96]`, covering 18.1 by
18.0 m against a facility that is 18 by 18 m. All four rooms, the corridor doorways, the shelving,
the workbench and the pillars are resolved.

## Only the occupancy grid is committed

`map_saver_cli` produces `facility.pgm` and `facility.yaml`, which is what is here. slam_toolbox can
also serialize its pose graph, and that is what its `localization` mode loads, but for this map
those come out at 33 MB and 1.6 MB against 130 KB for the grid. A 33 MB binary that has to be
regenerated whenever the scene changes does not belong in git.

The consequence is that localization runs on `nav2_map_server` and AMCL rather than slam_toolbox's
`localization` mode, which needs the pose graph. The cost is worth knowing: AMCL's motion model is
parameterised on odometry noise, and this track's odometry is exact ground truth, so the model is
degenerate and its `alpha1` through `alpha5` cannot be tuned to anything that transfers to hardware.

## Regenerating

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping
# drive the robot through all four rooms, then:
ros2 run nav2_map_server map_saver_cli -f facility
```

Re-map whenever `g1_bringup`'s `g1_navigation_scene.xml` changes. Nothing checks that the committed
map still matches the scene, and a stale map shows up as Nav2 planning through walls that moved.
