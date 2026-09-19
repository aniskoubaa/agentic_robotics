#!/usr/bin/env python3
"""
AGR Arm — the showcase: clear the bench, one block at a time.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_demos demo_pick_and_place
    ros2 run agr_arm_demos demo_pick_and_place --ros-args -p blocks:=1

WHAT  Reset the bench, look at it from the overview pose, then pick each of
      the three blocks and place it in the drop tray. Narrated as it goes,
      and it reports at the end whether the blocks are actually in the tray
      according to Gazebo — not according to whether the code thinks it went
      well.

WHY A DEMO AND NOT JUST THE EXAMPLE
    06_pick_and_place shows ONE pick with every step explained. This runs the
    whole task end to end and measures it, which is a different thing to
    watch: three picks in a row is where you see that the second and third
    are not free — the tray fills up, the drop point has to move, and a block
    that bounces changes the problem for the one after it.

    It is also the honest test of the platform. If this finishes 3/3, the
    kinematics, the controllers, the gripper and the grasp constraint are all
    working together, which no single check proves on its own.

Run `ros2 run agr_arm_demos diagnose` first if anything here surprises you.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from agr_arm_tools import gz_utils, kinematics as K
from agr_arm_tools.arm import Arm
from agr_arm_tools.move_to_pose_server import POSES

ORDER = ('block_red', 'block_green', 'block_blue')
TRAY = (0.66, 0.30)
# Blocks are dropped at three different spots inside the tray, spread in BOTH
# axes. Spacing them 0.05 m apart along x alone is not enough: a 50 mm cube
# dropped 50 mm from another one rolls into it, and the second block ends up
# sitting on top of the first. The tray's inner floor is 0.19 x 0.19, which
# is just enough for three cubes if they are staggered.
TRAY_SLOTS = ((-0.055, -0.045), (0.0, +0.050), (+0.055, -0.045))
HOVER = 0.12
CARRY_Z = 0.95
DROP_Z = 0.86


def say(text: str) -> None:
    print(text, flush=True)


def in_tray(p) -> bool:
    """Is this block in the drop tray?

    Note the generous height band. A block that lands ON TOP of one already
    in the tray is in the tray — the task was "clear the bench", not "arrange
    them in a single layer" — and an earlier version of this test called that
    a miss, which made a perfectly good run report 2/3.
    """
    return (abs(p[0] - TRAY[0]) < 0.11 and abs(p[1] - TRAY[1]) < 0.11
            and 0.74 < p[2] < 0.95)


def pick_one(arm, name, slot):
    pose = gz_utils.get_model_world_pose(name)
    if pose is None:
        say(f'    {name} is not in the world — skipping')
        return False, 0.0
    bx, by, bz = pose[0]
    grasp_z = K.lowest_tcp_over(K.BENCH_Z)

    ok, why = arm.can_reach(bx, by, grasp_z)
    if not ok:
        say(f'    {name} at ({bx:+.2f}, {by:+.2f}) cannot be reached: {why}')
        return False, 0.0

    t0 = time.time()
    arm.open_gripper()
    arm.move_to(bx, by, bz + HOVER)
    arm.move_to(bx, by, grasp_z)
    arm.close_gripper()
    arm.sleep(0.4)
    held = arm.held_object
    say(f'    gripper closed at {arm.tip_separation_mm:.0f} mm; '
        f'holding {held or "nothing"}')
    arm.move_to(bx, by, CARRY_Z)
    arm.move_to(TRAY[0] + slot[0], TRAY[1] + slot[1], DROP_Z)
    arm.open_gripper()
    arm.sleep(1.5)
    arm.move_to(TRAY[0] + slot[0], TRAY[1] + slot[1], CARRY_Z)

    end = gz_utils.get_model_world_pose(name)
    landed = end[0] if end else (0, 0, 0)
    ok = in_tray(landed)
    stacked = ok and landed[2] > 0.82
    say(f'    {name}: moved {math.dist(landed, (bx, by, bz)) * 100:.0f} cm, '
        f'landed ({landed[0]:+.3f}, {landed[1]:+.3f}, {landed[2]:.3f}) — '
        f'{"IN THE TRAY" + (" (stacked)" if stacked else "") if ok else "MISSED"}'
        f'   [{time.time() - t0:.0f} s]')
    return ok, time.time() - t0


def main() -> int:
    rclpy.init()
    node = Node('demo_pick_and_place')
    node.declare_parameter('blocks', 3)
    count = max(1, min(3, int(node.get_parameter('blocks').value)))

    arm = Arm(node)
    reset = node.create_publisher(String, '/grasp/reset', 10)

    say('\nAGR arm — pick and place\n' + '=' * 62)
    if not arm.wait_for_state():
        say('no /joint_states — start the simulator first:  agr-sim arm tools:=true\n')
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    say('\n  Putting the bench back the way it started.')
    # Wait for grasp_server to have MATCHED this publisher before sending.
    # create_publisher() returns instantly but DDS discovery does not, and a
    # reset published into the void is lost silently: the demo then picks the
    # blocks up from wherever the previous run happened to leave them, which
    # still "works" and is not the demo anyone meant to run.
    waited = 0.0
    while waited < 10.0 and reset.get_subscription_count() == 0:
        arm.sleep(0.1)
        waited += 0.1
    if reset.get_subscription_count() == 0:
        say('    nobody is listening on /grasp/reset — grasp_server is not '
            'running.\n    The bench will not be reset.')
    else:
        reset.publish(String(data=''))
        arm.sleep(6.0)
        placed = gz_utils.get_world_poses()
        marks = [f'{n} ({p[0][0]:+.2f}, {p[0][1]:+.2f})'
                 for n, p in sorted(placed.items()) if n.startswith('block_')]
        say('    ' + ';  '.join(marks) if marks else '    no blocks found')
    if not arm.heard_from_grasp_server:
        say('\n  WARNING: grasp_server is not running, so the gripper will close\n'
            '  on nothing and every pick will fail. Start it with:\n'
            '      agr-sim arm tools:=true\n'
            '  Carrying on so you can watch the motion.\n')

    say('\n  Going to the overview pose — from here the wrist camera sees the\n'
        '  whole bench. (ros2 run agr_arm_teleop camera_view to watch.)')
    arm.move_joints(POSES['watch'], timeout=20.0)
    arm.open_gripper()
    p = arm.tcp
    say(f'    grasp point at ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m')

    say(f'\n  Clearing {count} block(s) into the tray at '
        f'({TRAY[0]:.2f}, {TRAY[1]:.2f}).')
    results = []
    t_all = time.time()
    for name, slot in zip(ORDER[:count], TRAY_SLOTS[:count]):
        say(f'\n  --- {name} ---')
        results.append(pick_one(arm, name, slot))

    say('\n  Back to home.')
    arm.home(timeout=20.0)

    good = sum(1 for ok, _ in results if ok)
    say('\n' + '=' * 62)
    say(f'{good}/{len(results)} block(s) in the tray, '
        f'{time.time() - t_all:.0f} s total.')
    if good < len(results):
        say('A miss is usually one of three things, in this order of likelihood:\n'
            '  - grasp_server is not running (run diagnose, check 8)\n'
            '  - the block was knocked out of reach by an earlier pick\n'
            '  - the tray is full and the block bounced out')
    print()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if good == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
