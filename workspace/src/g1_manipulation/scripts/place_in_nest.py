#!/usr/bin/env python3
"""Walk to the pallet and put the held part into a nest, stepping to where the arm reaches it.

    ros2 run g1_manipulation place_in_nest.py nest_4
    ros2 run g1_manipulation place_in_nest.py nest_3 --stance pallet_end_left --tries 3

Run it after the pick and raise_<cm> (the part held up, clear of the belt). It

1. walks to --stance with go_to_stance.py (unless --no-walk);
2. sends Place for pallet/slot/<nest>;
3. if the place says the nest is out of the arm's reach from where the robot stopped -- the
   walk ends only within a few centimetres of a stance, and the arm, as it holds the part, has
   little room -- the result names the stance it is in reach from
   ("[stance x=.. y=.. yaw_deg=..]"); this walks there and places again, up to --tries times.

Nothing is planned round the robot: the place either goes along its straight lines or stops
with the part where it was.
"""
import argparse
import os
import re
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from g1_msgs.action import Place

STANCE = re.compile(r"\[stance x=([-0-9.]+) y=([-0-9.]+) yaw_deg=([-0-9.]+)\]")


def walk(args):
    print("walking:", " ".join(args), flush=True)
    script = os.path.join(get_package_prefix("g1_manipulation"), "lib", "g1_manipulation", "go_to_stance.py")
    return subprocess.run([sys.executable, script] + args).returncode


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("nest", help="nest name, e.g. nest_4 (or a full id like pallet/slot/nest_4)")
    p.add_argument("--stance", default="pallet_end_left", help="stance to walk to first")
    p.add_argument("--no-walk", action="store_true", help="place from where the robot stands")
    p.add_argument("--arm", default="left")
    p.add_argument("--tries", type=int, default=3, help="places at most (each after a step)")
    p.add_argument("--accept", type=float, default=0.04, help="go_to_stance --accept for the steps, m")
    a = p.parse_args()

    slot = a.nest if "/" in a.nest else "pallet/slot/" + a.nest
    if not a.no_walk:
        walk([a.stance])

    rclpy.init()
    node   = Node("place_in_nest")
    client = ActionClient(node, Place, "/g1_manipulation_server/place")
    if not client.wait_for_server(timeout_sec=30.0):
        print("no /g1_manipulation_server/place", flush=True)
        return 1
    for k in range(1, a.tries + 1):
        goal                   = Place.Goal()
        goal.surface_object_id = slot
        goal.arm               = a.arm
        sent = client.send_goal_async(
            goal, feedback_callback=lambda fb: print("  phase:", fb.feedback.phase, flush=True))
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            print("the place was not accepted", flush=True)
            return 1
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done)
        result = done.result().result
        print(f"place {k}: {'OK' if result.success else 'failed'}: {result.message}", flush=True)
        if result.success:
            return 0
        m = STANCE.search(result.message)
        if m is None or k == a.tries:
            return 1
        walk(["--x", m.group(1), "--y", m.group(2), "--yaw-deg", m.group(3),
              "--accept", str(a.accept), "--accept-yaw-deg", "4"])
    return 1


if __name__ == "__main__":
    sys.exit(main())
