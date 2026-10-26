# G1 basic: simulation, walking, Nav2 and the conveyor world

A ROS 2 Humble stack for the [Unitree G1](https://www.unitree.com/g1) humanoid (29 DoF, RH56E2
hands), developed simulation-first against `unitree_mujoco`. This branch is the basic subset:

- **Simulation:** MuJoCo with the simulated Mid360 LiDAR and head depth camera, speaking the same
  DDS topics as the real robot.
- **Walking:** a learned walking policy under the onboard-controller stand-in, driven through the
  LocoClient bridge with a gait shaper and a locomotion-authority bracket.
- **Navigation:** SLAM Toolbox mapping, AMCL localization against a committed map, and Nav2.
- **Worlds:** the multi-room facility (`world:=navigation`, map `facility`) and the conveyor line
  (`world:=conveyor`, `conveyor_flat`, `conveyor_fixed`, `conveyor2`, map `conveyor`): a room
  with a table of BSH panels, a belt and a pallet that rides it.

Object perception and manipulation are not part of this branch.

Based on [grove-g1](https://github.com/Adyansh04/grove-g1) (BSD-3-Clause, see `LICENSE`).

On hardware the simulation card becomes the vendor's onboard motion service and the LiDAR front
end becomes `livox_ros_driver2`. Everything above the DDS rail is unchanged.

Two rules shape the design, and both apply in simulation so the habits transfer:

- Only one publisher ever commands a low-level channel. Control-mode ownership is explicit.
- Arm motion goes through `rt/arm_sdk`, which blends against the onboard balance controller.
  Commanding raw `/lowcmd` means owning balance yourself.

## Packages

| Package | What it does |
|---|---|
| [`g1_bringup`](workspace/src/g1_bringup) | The entry point. Launch files, scenes and config that compose everything below. |
| [`g1_description`](workspace/src/g1_description) | Vendored G1 URDF plus the `ros2_control` xacro wrapper. |
| [`g1_hand_interface`](workspace/src/g1_hand_interface) | `ros2_control` plugin for one hand, over the hand's own DDS topics. |
| [`g1_hardware_interface`](workspace/src/g1_hardware_interface) | `ros2_control` plugin bridging the 14 arm joints onto `rt/arm_sdk`. |
| [`g1_locomotion`](workspace/src/g1_locomotion) | LocoClient bridge, gait shaper and the locomotion-authority bracket. |
| [`g1_motion_service_sim`](workspace/src/g1_motion_service_sim) | Simulation stand-in for the robot's onboard motion service. |
| [`g1_msgs`](workspace/src/g1_msgs) | The `SetLocoMode`, `ApproachObject` and `Retreat` actions and the `LocoStatus` message. |
| [`g1_navigation`](workspace/src/g1_navigation) | SLAM Toolbox mapping, AMCL localization and Nav2. |
| [`g1_sensor_relay`](workspace/src/g1_sensor_relay) | Publishes LiDAR and depth frames sampled inside the simulator. |
| [`g1_state_estimation`](workspace/src/g1_state_estimation) | Publishes `odom` to `base_footprint` and the TF chain Nav2 needs. |

## Quick start

The ROS build and runtime commands run inside the dev container. The host is Ubuntu 24.04; the
container provides the Ubuntu 22.04 and ROS 2 Humble combination the Unitree SDK needs.

### Prerequisites

Install Docker Engine with Docker Compose v2, the NVIDIA driver and NVIDIA Container Toolkit on
the host. The simulator uses the GPU exposed by `docker-compose.yml`. For the GUI modes, run from
an X11 desktop session; `manage.sh start` grants the container local X11 access.

Install [`vcstool`](https://github.com/dirk-thomas/vcstool) on the host as well. The import script
uses its `vcs` command before the development container exists:

```bash
python3 -m pip install --user vcstool
```

### Start the development container

Run these commands from the repository root on the host:

```bash
cp .env.example .env
./scripts/import-externals.sh
./scripts/manage.sh start
./scripts/manage.sh exec
```

`import-externals.sh` pulls the third-party packages listed in `workspace.repos` into
`workspace/src` and puts the two that ship a non-standard layout into a buildable one. Run it
again whenever `workspace.repos` changes.

### Build and run the stack

Inside the container:

```bash
cd /root/workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Then bring up the robot. One command covers every mode:

```bash
# Simulator only, with the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py headless:=false

# Build a map, with RViz but without the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true headless:=true

# Localize against the committed map and navigate, with RViz but without the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true headless:=true

# The conveyor world: localize against maps/conveyor and navigate, with the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py \
  world:=conveyor_fixed mode:=localization nav:=true rviz:=true headless:=false
```

The map follows the world (`facility` for `navigation`, `conveyor` for the conveyor worlds);
`map:=<file.yaml>` overrides it. The conveyor map is built from the scene geometry by
`g1_bringup/tools/make_conveyor_map.py`, see `g1_navigation/maps/README.md`. The first time a
conveyor world starts, the launch copies the panel mesh (`g1_bringup/cad/BSH_panel.stl`) into the
simulator's model directory.

`headless:=true` is the default and is appropriate for non-GUI runs. Set `headless:=false` only
when a local display is available to open the MuJoCo viewer; do not use the viewer's Reload button
while sensors are running.

After launching a mode, confirm that the ROS graph is available from a second container shell:

```bash
ros2 topic list -t
```

Send it somewhere:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

In the conveyor world, for example in front of the table (`x` toward the table, `y` to its side):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: -0.9}, orientation: {w: 1.0}}}}"
```

## Development environment

Docker Compose is the runtime source of truth; `.devcontainer/devcontainer.json` is the VS Code
layer on top. In VS Code, use `Dev Containers: Reopen in Container`.

| Setting | Value |
|---|---|
| ROS distro | Humble, pinned. Unitree tests only Foxy and Humble. |
| Middleware | CycloneDDS, pinned to loopback |
| `ROS_DOMAIN_ID` | 1 |
| Robot override | `GROVE_G1_ROS_DOMAIN_ID`, `GROVE_G1_CYCLONEDDS_URI`, `GROVE_G1_ROBOT_NIC` |
| C++ standard | C++20 on GCC 11.4 |
| Workspace | `/root/workspace` |
| Shared data | `/root/data` |

The container runs `privileged` with `network_mode: host` and a `/dev` bind mount. That is
deliberate for local robotics development: DDS discovery between the bare-DDS simulator and the
ROS graph happens over loopback, and device access has to work.

Pointing the container at a real G1 is three environment variables, not an image rebuild:
`GROVE_G1_CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.hardware.xml` (baked in beside the
loopback one, differing only in the interface), `GROVE_G1_ROBOT_NIC` for the NIC that reaches
the robot, and `GROVE_G1_ROS_DOMAIN_ID` for its domain. They are prefixed because the base
image's own `/etc/profile.d/10-ros-env.sh` rewrites the unprefixed names. `sim.launch.py`
refuses to start unless `CYCLONEDDS_URI` names a profile that pins `lo`, so the simulator
cannot be brought up pointing at a robot.

Lifecycle:

```bash
./scripts/manage.sh start | stop | restart | recreate | logs | exec
```

Use `exec-as-me` instead of `exec` for anything that rewrites source files in place, such as
`clang-tidy --fix` or `clang-format -i`. It runs as your host user, so the files do not come back
owned by root.

Project dependencies belong in `.devcontainer/Dockerfile`, followed by
`./scripts/manage.sh recreate`. Do not install into a running container and forget about it.

## Tests

```bash
colcon test --packages-select g1_description g1_locomotion g1_navigation
colcon test-result --all
```

Leftover nodes from a previous run are the most productive source of phantom failures here:
several copies of the stack on one DDS graph look like bugs everywhere except where they are.
Clear them first, and note it verifies the graph rather than the process table:

```bash
./scripts/clean-stack.sh
```

Suites that launch a simulator are timing-sensitive and serialize on a shared ctest resource lock.
Run them **one package at a time**, and check nothing is left over from a previous run
(`pgrep -x unitree_mujoco`) before trusting a result: a stray simulator is the usual explanation
for a batch of failures that all pass on a clean rerun. Each package README says which of its
tests need a simulator.

Those suites carry the ctest label `simulator`, so the rest can be run on their own:

```bash
colcon test --ctest-args -LE simulator   # everything that needs no simulator
colcon test --ctest-args -L  simulator   # only the simulator suites
```

## Continuous integration

Every pull request, and every push to `main`, builds the workspace and runs the tests that need
no simulator, in the image built by `.github/ci.Dockerfile`. Lint runs as part of `colcon test`,
not separately.

The simulator suites are excluded: they are CPU-time-sensitive and measure a shared runner
rather than the stack. Run them locally before merging anything that touches locomotion,
navigation or the sensor path.

A per-package C++ coverage table is printed to each run's summary. It covers only the tests CI
runs, so the node and launch layer reads low there by construction — it is a signal on the pure
logic, not a figure for the repository, which is why there is no badge for it.

## Repository layout

```
.devcontainer/     derived dev image
workspace/src/     ROS 2 packages
workspace/patches/ patches applied to vendored sources at image build
workspace/vendor/  our source compiled into the vendored simulator
scripts/           container lifecycle, and stack teardown
```
