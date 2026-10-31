![Grove-G1](docs/media/banner.svg)

# Grove-G1

[![CI](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml/badge.svg)](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml)
[![License: BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)

> **This is the Humble line.** Locomotion below the waist stays with Unitree's own internal leg
> policy: the stack sends walking commands to the onboard controller and never touches a leg
> joint directly. Arm and hand manipulation is done through MoveIt on top of that.
>
> For the ROS 2 Jazzy line, where the stack owns all 29 body motors over `rt/lowcmd` and runs its
> own balance policy with no onboard controller underneath, see
> [`main`](https://github.com/Adyansh04/grove-g1/tree/main).

An autonomy stack for the [Unitree G1](https://www.unitree.com/g1) humanoid, built on ROS 2 Humble
and developed simulation-first against `unitree_mujoco`.

The simulator speaks the same DDS topics as the real robot, so the bridge layer, the navigation
stack and the control-authority logic all carry over to hardware without code changes. Moving to
the physical G1 is a domain-ID and interface change, not a rewrite.

## Status of this branch (`pallet-nest-end`): panels into the pallet's nests

**Goal.** In `world:=conveyor_fixed`, the G1 (Inspire RH56E2 hands) takes a BSH panel off the tray
with its left hand, carries it to the start of the belt and sets it into the pallet's nest_4, then
a second panel into nest_3, without swinging the panel into its head, body or the cell.

**Result: it does not work as a real grasp, and the main reason is the arm, not the planner.**
What runs end to end in MuJoCo does so because the simulator welds the panel to the palm once the
fingers close; the hand alone does not hold it the way the arm needs it held.

### Why: the 7-DOF arm cannot carry and place palm up

The only grasp this hand can slide under a panel lying on the tray is the palm-up scoop (the four
straight fingers wedge into the 2-3 cm gap under the panel's lip, the thumb closes on top). The
G1's arm runs out of joint range holding a hand palm up in front of the body:

- **At the table only the elbow-up arm configuration avoids the torso.** Every elbow-down IK
  answer passes through the body. With the hand held still the elbow can move only about 5 cm
  before the wrist roll and the shoulder yaw reach their limits, so the arm cannot be re-shaped
  once the panel is in hand: the configuration the pick ends in is the one it places with.
- **Palm up puts the wrist roll at the end of its range** (limit ±1.97 rad, about ±113°).
  Lowering the panel into nest_4 from `pallet_end_left` the wrist roll reaches 1.87 and the
  shoulder yaw -2.48 (limit -2.62): 0.10 and 0.14 rad to spare.
- **So the place has almost no room.** With the robot 8 cm to the right of the stance the lower
  stopped 1 cm short of the nest, both joints at their limits. Carrying in a different arm posture
  (the "work" posture, the elbow further out) changes nothing: the nest, not the carry, sets the
  joints at the bottom. nest_3, 15 cm further along the belt, is out of reach from the same stance
  (shoulder yaw at its limit) and needs its own stance (`pallet_nest3_left`).
- Our own straight-line planner (`moveOwnLine`: damped least squares, joint-limit handling in the
  null space, collision-checked waypoints) and the stance search (`stanceFor`, the place says
  where to stand) get the most out of that range, but cannot add range the arm does not have.

Turned over (palm down) the same arm is comfortable: in the nests the wrist roll sits at -0.3 to
0.8 and the shoulder yaw at -0.7 to -1.5, and nest_4 was reached from the stance and 8 cm off it
in all four directions, nest_3 from its stance and 8 cm to the side (4-16 mm off the nest).

### Why palm down does not save it: the hand

Palm down means the thumb has to go under the panel and the fingers over it
(`scoop_thumb_left_near`). The RH56E2 has one thumb, thicker and softer than the four fingers
together, and the gap under the panel's lip on the tray is only 2-3 cm:

- With the panel free in MuJoCo and a stiff thumb, the thumb goes under the lip only within about
  5 mm of one height. 1 cm lower it hits the table or pushes the panel 15-30 mm along it; 1 cm
  higher it rides over the lip onto the top.
- With the simulator's own (soft) finger gains the thumb ended on top of the panel at every height
  tried (0, +1, +2 cm). The placements above still succeeded only because the weld held the panel.
- One hand cannot hold the 0.6 m panel level either way: in MuJoCo neither scoop holds it without
  the weld, palm up or down, even with the finger torque raised to 3 Nm. The panel sags and turns
  in the hand (it settles 2-4 cm as the fingers close; the pick now re-measures the held pose after
  the lift for that).
- The RH56E2's fingertip force (up to 30 N per the vendor) would be enough on paper for a 0.3 kg
  panel held near its middle; the problem is the geometry of getting the thumb under it, not the
  force.

### What would make it work

- **A fixture or tray that lifts the panel's edge** (4 cm or more of clearance), so the thumb can
  go under with room for arm sag; or panels presented on edge.
- **A different end effector** for this part (a flat two-jaw or suction gripper needs no gap and
  no wrist roll).
- **Two hands**, the second taking the panel's other half (the palm-up two-hand scoop is in the
  code, `scoop_two_hand`), or a regrasp from palm up to palm down between the hands.
- **A first finger move that lifts the edge** before the thumb goes in, as a person does; untried,
  and the simulator's finger model is too weak to judge it.
- **Nests turned toward the robot** (or the robot placing from the pallet's side), so the
  palm-up place lands mid-range of the wrist instead of at its end.

### What is in the code

- `scoop_thumb_left_near` is listed first in `g1_perception/config/object_tracker_conveyor_flat.yaml`,
  so the left pick tries palm down first; delete that line (or move it below `scoop_left_near`)
  to go back to the palm-up scoop.
- Commands, parameters and every measurement: [`g1_manipulation/README.md`](workspace/src/g1_manipulation/README.md),
  section "Into the pallet's nests".

## What it does today

The robot maps a facility with SLAM Toolbox, localizes against the saved map, and drives itself to
a goal pose under Nav2. Arm trajectories run through `ros2_control` onto Unitree's weight-blended
`rt/arm_sdk` interface, so the vendor's onboard controller keeps the legs balanced throughout.

MoveIt plans for either arm or both together, collision-checked against a live octomap built from
the LiDAR, and each Dex3-1 hand is its own planning group with `open` and `closed` postures.

On top of that, pick and place are served as actions, and a BehaviorTree.CPP behaviour tree
sequences them with navigation into a mission that runs end to end in the facility world: drive
to a workbench, walk the last half metre under closed-loop control, pick a cube up, carry it
across the building, and put it down on a bench. Object poses come from a source that refuses to
run on hardware, because there is no object-detection pipeline yet; a real one replaces it
without the skills changing.

Nav2 parks within 0.5 m of a goal and the arm's usable window is about 0.2 m wide, so a base
approach skill closes the gap against the measured object rather than against the map. The tree
is editable in Groot2 against a generated node palette.

Learned manipulation for unstructured scenes is the next milestone and is not built yet.

## Nav2 Demo

![Nav2 Demo](docs/media/grove_nav2_demo.gif)

## MoveIt Demo

![MoveIt Demo](docs/media/grove_moveit_demo.gif)

## Pick & Place Demo

![Pick & Place Demo](docs/media/grove_pick_place_demo.gif)

## Architecture

![Grove-G1 architecture](docs/media/architecture.svg)

On hardware the simulation card becomes the vendor's onboard motion service and the LiDAR front
end becomes `livox_ros_driver2`. Everything above the DDS rail is unchanged.

Two rules shape the design, and both apply in simulation so the habits transfer:

- Only one publisher ever commands a low-level channel. Control-mode ownership is explicit.
- Arm and locomotion motion goes through `rt/arm_sdk`, which blends against the onboard balance
  controller. Commanding raw `/lowcmd` means owning balance yourself.

## Packages

| Package | What it does |
|---|---|
| [`g1_bringup`](workspace/src/g1_bringup) | The entry point. Launch files, scenes and config that compose everything below. |
| [`g1_description`](workspace/src/g1_description) | Vendored G1 URDF plus the `ros2_control` xacro wrapper. |
| [`g1_hand_interface`](workspace/src/g1_hand_interface) | `ros2_control` plugin for one Dex3-1 hand, over the hand's own DDS topics. |
| [`g1_hardware_interface`](workspace/src/g1_hardware_interface) | `ros2_control` plugin bridging the 14 arm joints onto `rt/arm_sdk`. |
| [`g1_locomotion`](workspace/src/g1_locomotion) | LocoClient bridge, gait shaper and the locomotion-authority bracket. |
| [`g1_motion_service_sim`](workspace/src/g1_motion_service_sim) | Simulation stand-in for the robot's onboard motion service. |
| [`g1_manipulation`](workspace/src/g1_manipulation) | Pick and place as actions, and the object-pose source behind them. |
| [`g1_moveit_config`](workspace/src/g1_moveit_config) | MoveIt config: arm and hand planning groups, kinematics, the octomap. |
| [`g1_msgs`](workspace/src/g1_msgs) | The `SetLocoMode` action and `LocoStatus` message. |
| [`g1_navigation`](workspace/src/g1_navigation) | SLAM Toolbox mapping, AMCL localization and Nav2. |
| [`g1_orchestration`](workspace/src/g1_orchestration) | The behaviour tree that sequences navigation and manipulation into a mission. |
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

# Plan for the arms and hands, with the LiDAR octomap in the planning scene
ros2 launch g1_bringup bringup.launch.py moveit:=true sensors:=true rviz:=true headless:=true

# Pick and place skills, on the small test world where the object is already within reach
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true pin_pelvis:=true world:=manipulation \
  activate_arm:=true activate_arm_delay_s:=40.0 headless:=true

# Everything at once, including the MuJoCo viewer and RViz
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true rviz:=true \
  activate_arm:=true activate_arm_delay_s:=40.0 headless:=false
```

`headless:=true` is the default and is appropriate for non-GUI runs. Set `headless:=false` only
when a local display is available to open the MuJoCo viewer; do not use the viewer's Reload button
while sensors are running.

After launching a mode, confirm that the ROS graph is available from a second container shell:

```bash
ros2 topic list -t
```

Run a mission. The tree drives navigation and manipulation; nothing else needs starting, and
Groot2 on the host can watch it tick at `localhost:1667`:

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place_in_place.xml
```

The full navigate-pick-carry-place mission needs the facility world and a map:

```bash
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true world:=navigation \
  rviz:=true activate_arm:=true activate_arm_delay_s:=55.0
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place.xml
```

With `rviz:=true` and both MoveIt and Nav2 running, this opens two RViz windows: the MoveIt one
for the arm and a second on the navigation config for the map and costmaps.

Send it somewhere:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

Moving the arms or hands needs an explicit acquire step first, and a matching release. It takes
the arm and both hands together; a hand that is absent or unpowered logs and leaves the arm
usable.

```bash
ros2 launch g1_bringup activate_arm.launch.py
# plan from RViz's MotionPlanning panel, or send FollowJointTrajectory goals to
# arm_trajectory_controller, left_hand_controller or right_hand_controller
ros2 launch g1_bringup deactivate_arm.launch.py
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
