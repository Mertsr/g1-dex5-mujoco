"""The operator entry point: bare sim, mapping, localization, Nav2, or a combination.

    ros2 launch g1_bringup bringup.launch.py                                  # bare sim
    ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true         # + SLAM
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
    ros2 launch g1_bringup bringup.launch.py world:=conveyor_fixed mode:=localization nav:=true rviz:=true

sim.launch.py and control.launch.py are unchanged and still work standalone; this file sits
above them and adds nothing of its own except the composition.

================================================================================
g1_navigation is referenced WITHOUT being declared as a dependency. On purpose.
================================================================================

g1_navigation already declares <exec_depend>g1_bringup</exec_depend>, because it composes
bring-up and not the other way round. Adding a reciprocal dependency here does not build --
colcon refuses outright:

    ERROR:colcon:colcon list: Unable to order packages topologically:
    g1_bringup: ['g1_navigation']
    g1_navigation: ['g1_bringup']

So the reference below is a launch-time path lookup and nothing else. g1_bringup builds,
installs and runs with g1_navigation absent; only mode:=mapping / mode:=localization name it,
and _navigation_share() turns an absence into an actionable message.

Two pieces, each conditional and each unaware of the other:

  * the simulator -- `_simulator()`, the only place it is named anywhere in this file, and
    exactly once per launch. Two would put two motion_service_sim processes on /lowcmd.
  * navigation -- g1_navigation's nav_stack.launch.py, which stages no simulator of its own.

The map follows the world: world:=navigation localizes against maps/facility, the conveyor
worlds against maps/conveyor. map:= overrides either.

A note on what is NOT forwarded: a child's DeclareLaunchArgument default never fires for a name
the PARENT also declares, so a name declared in both places must be forwarded explicitly or the
parent's value silently wins. Names this file never declares -- nav_stack's use_composition and
container_name -- resolve to the child's own default, which is the intended value.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 'none' keeps g1_navigation entirely out of the picture; the other two bring it in.
MODES = ("none", "mapping", "localization")

# A bare simulator wants 2.0; anything that starts a stack alongside it wants 4.0, because more
# nodes come up before the first physics tick and the robot topples at spawn if discovery is
# still in progress. Declaring either as this file's default would silently override whichever
# branch was right, because an included launch file inherits the parent's configurations.
# The empty sentinel means "let the branch decide", and a concrete value is always forwarded.
DEFAULT_SIM_START_DELAY_S = {"bare": "2.0", "loaded": "4.0"}


def _navigation_share():
    """g1_navigation's share directory, or a message an operator can act on."""
    try:
        return get_package_share_directory("g1_navigation")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "bringup.launch.py: mode:=mapping and mode:=localization need the g1_navigation "
            "package, which is not on the ament prefix path.\n"
            "  - Build it:  colcon build --packages-select g1_navigation\n"
            "  - Then source install/setup.bash again in this shell.\n"
            "g1_bringup deliberately does not declare g1_navigation as a dependency (the two "
            "would form a colcon dependency cycle -- see this file's docstring), so nothing "
            "builds it for you.\n"
            "mode:=none needs none of this and runs the simulator on its own."
        ) from exc


def _simulator(sim_args):
    """The one simulator this file stages, and the only place it is named.

    Isolated deliberately. Everything above composes around it, so swapping in a hardware
    bring-up later replaces this function's body and nothing else: the branches only decide
    what goes in sim_args. There is no platform:= argument because there is no second
    implementation yet, and one would be a knob with a single position.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments=sim_args.items(),
    )


def _map_for(world):
    """The committed occupancy grid for a world, or None to keep the localization default."""
    name = "conveyor" if world.startswith("conveyor") else "facility" if world == "navigation" else None
    if name is None:
        return None
    path = os.path.join(_navigation_share(), "maps", f"{name}.yaml")
    return path if os.path.exists(path) else None


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'none' is the simulator on its own; 'mapping' "
            f"builds a map with slam_toolbox; 'localization' runs map_server + AMCL against "
            f"the committed one, and is what nav:=true requires."
        )

    navigating = mode != "none"
    want_nav = LaunchConfiguration("nav").perform(context).lower() == "true"
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    pin_pelvis = LaunchConfiguration("pin_pelvis").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)

    if want_nav and mode != "localization":
        raise RuntimeError(
            f"nav:=true needs mode:=localization, not mode:={mode!r}. Navigating against a "
            "map slam_toolbox is still building means the goal pose moves under the planner."
        )
    if pin_pelvis and navigating:
        raise RuntimeError(
            "pin_pelvis:=true welds the pelvis to the world and disables the walking policy, "
            "so the robot cannot drive anywhere. It is a bare-sim debugging aid; use it with "
            "mode:=none."
        )

    delay = LaunchConfiguration("sim_start_delay_s").perform(context)
    if not delay:
        delay = DEFAULT_SIM_START_DELAY_S["loaded" if navigating else "bare"]

    # Every argument below is forwarded EXPLICITLY, including the ones whose values match
    # this file's own defaults. Included launch files inherit the parent's configurations,
    # so a child's DeclareLaunchArgument default never fires for anything declared here.
    sim_args = {
        # Not optional on the navigation modes, whatever the operator asked for: sensors gates
        # the LiDAR sweep, the relay, the odom -> base_footprint -> pelvis chain and the waist
        # joint states, and navigation is dead without all four.
        "sensors": "true" if navigating else LaunchConfiguration("sensors"),
        "world": LaunchConfiguration("world"),
        "odometry": LaunchConfiguration("odometry"),
        "headless": LaunchConfiguration("headless"),
        "pin_pelvis": "true" if pin_pelvis else "false",
        "walk_policy": LaunchConfiguration("walk_policy"),
        "waist_hold_rad": LaunchConfiguration("waist_hold_rad"),
        "sim_start_delay_s": delay,
        # We own RViz, below. Without this the simulator opens its own on the wrong config.
        "rviz": "false",
    }

    actions = [_simulator(sim_args)]

    if navigating:
        # nav_stack.launch.py stages no simulator of its own, which is what makes including it
        # next to _simulator() safe; two would be two writers on /lowcmd.
        nav_args = {
            "mode": mode,
            "nav": "true" if want_nav else "false",
            # Same value sim.launch.py gets: the gait shaper's profile has to match the
            # policy actually driving the legs.
            "walk_policy": LaunchConfiguration("walk_policy"),
        }
        map_yaml = LaunchConfiguration("map").perform(context) or _map_for(world)
        if map_yaml and mode == "localization":
            nav_args["map"] = map_yaml
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_navigation_share(), "launch", "nav_stack.launch.py")
                ),
                launch_arguments=nav_args.items(),
            )
        )

    if want_rviz:
        actions.extend(_rviz(navigating))

    return actions


def _rviz(navigating):
    """The RViz window that matches what is running: g1_navigation's config (map, costmaps,
    plan) on the navigation modes, g1_bringup's sensor config otherwise."""
    bringup_share = get_package_share_directory("g1_bringup")
    generic_rviz  = os.path.join(bringup_share, "launch", "rviz.launch.py")
    if navigating:
        config = os.path.join(_navigation_share(), "config", "g1_navigation.rviz")
    else:
        config = os.path.join(bringup_share, "config", "g1_sensors.rviz")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(generic_rviz),
            launch_arguments={"rviz_config": config}.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="none",
            description="'none' runs the simulator alone and never touches g1_navigation. "
            "'mapping' adds the scan pipeline and slam_toolbox. 'localization' adds the scan "
            "pipeline, map_server and AMCL, and is the mode nav:=true requires.",
        ),
        DeclareLaunchArgument(
            "nav",
            default_value="false",
            description="Start Nav2, the gait shaper and the locomotion authority bracket. "
            "Requires mode:=localization.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RViz. mode:=none uses g1_bringup's sensor config (fixed frame "
            "odom); the navigation modes use g1_navigation's, which adds the Nav2 display "
            "group and is fixed on map.",
        ),
        DeclareLaunchArgument(
            "walk_policy",
            default_value="walker",
            description="Which walking policy drives the legs: 'walker' (walker.onnx), "
            "'isaaclab' (the unitree_sim_isaaclab-trained policy.onnx) or 'none'. Forwarded to "
            "sim.launch.py, and to the navigation stack so the gait shaper loads the matching "
            "profile.",
        ),
        DeclareLaunchArgument(
            "waist_hold_rad",
            default_value="",
            description="SIM-ONLY: three comma-separated radians (yaw,roll,pitch) to stand the "
            "waist at. Requires pin_pelvis:=true, which sim.launch.py enforces. Use it to "
            "check arm planning against a torso that is not square to the pelvis.",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="LiDAR sweep, the relay and the odom -> base_footprint -> pelvis "
            "chain. Only meaningful with mode:=none -- the navigation modes need it and turn "
            "it on themselves.",
        ),
        DeclareLaunchArgument(
            "odometry",
            default_value="sportmodestate",
            description="Which source publishes odom -> base_footprint. 'sportmodestate' is "
            "exact MuJoCo state and is what the mission is tuned against. 'fast_lio' runs the "
            "LiDAR-inertial pipeline the real robot uses, over the simulated Mid360.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which scene to stage. 'navigation' is the multi-room facility; "
            "'conveyor', 'conveyor_flat', 'conveyor_fixed' and 'conveyor2' are the conveyor "
            "line (a table with the BSH panels, a belt, a pallet). Each has its committed map "
            "(maps/facility, maps/conveyor); localization against a world without one will "
            "not converge.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Occupancy grid to localize against. Empty: the world's own "
            "(maps/facility for navigation, maps/conveyor for the conveyor worlds).",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer. Its Reload button is fatal with "
            "sensors on; see the README.",
        ),
        DeclareLaunchArgument(
            "pin_pelvis",
            default_value="false",
            description="SIM-ONLY debugging aid: weld the pelvis and disable the walking "
            "policy, to exercise the arm bridge with nothing driving the legs. mode:=none only.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="",
            description="Seconds to delay the simulator's start. Empty means the branch's own "
            "default: 2.0 for mode:=none, 4.0 for the navigation modes, which start more nodes "
            "before the first physics tick.",
        ),
        OpaqueFunction(function=_setup),
    ])
