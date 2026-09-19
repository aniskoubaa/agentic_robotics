#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
AGR Arm — print the reachable workspace, or test one point.

    ros2 run agr_arm_tools workspace
    ros2 run agr_arm_tools workspace 0.45 0.18 0.775
    ros2 run agr_arm_tools workspace --height 0.95

WHY THIS IS THE FIRST TOOL YOU SHOULD RUN
    The UAV platform has `list_airframes` — the thing you check before
    wondering why the drone will not do what you asked. This is the arm's
    equivalent. Almost every "the arm ignored me" on a fixed-base robot is
    the target being outside the workspace, and the workspace is not a shape
    anyone can picture from a datasheet number: "850 mm reach" is measured to
    the TOOL, but a straight-down grasp puts the WRIST 0.22 m above the point
    you asked for, and it is the wrist that has to fit inside the arm's span.

    So the map below is not a circle of radius 0.85. It is smaller, and it is
    not even centred where you would guess, because the UR5e's wrist is
    offset 0.133 m sideways from its shoulder.

Nothing here talks to ROS or needs the simulator running — it is pure
kinematics, so it works while you are still writing the script.
"""

import sys

from agr_arm_tools import kinematics as K

BENCH_Z = 0.775          # a block's centre when it is sitting on the bench


def print_point(x: float, y: float, z: float) -> int:
    ok, result = K.reachable(x, y, z)
    print(f'\n  ({x:+.3f}, {y:+.3f}, {z:.3f}) with the tool pointing down:\n')
    if ok:
        print(f'    REACHABLE\n    {K.describe(result)}')
        return 0
    print(f'    NOT REACHABLE\n    {result}')
    return 1


def print_map(z: float) -> int:
    xs = [0.15 + 0.05 * i for i in range(17)]      # 0.15 .. 0.95
    ys = [0.40 - 0.05 * i for i in range(17)]      # +0.40 .. -0.40
    print(f'\n  Top-down reachability at z = {z:.3f} m '
          f'(a block on the bench sits at {BENCH_Z:.3f})\n')
    print('        ' + ''.join(f'{x:5.2f}' for x in xs) + '   <- x (m)')
    for y in ys:
        row = ''.join('    #' if K.reachable(x, y, z)[0] else '    .' for x in xs)
        print(f'  {y:+5.2f}' + row)
    print('   ^ y (m)          # = a straight-down grasp is possible, . = not')
    print('\n  Nothing here checks for COLLISION. The marks close to x = 0.15 are')
    print('  directly over the pedestal, and the arm would have to fold through')
    print('  its own base to get there. Reachable is a statement about the six')
    print('  joint angles, not a promise that the path is clear.')
    print(f'\n  shoulder at ({K.SHOULDER[0]:.2f}, {K.SHOULDER[1]:.2f}, '
          f'{K.SHOULDER[2]:.3f});  the links span {K.MAX_WRIST_REACH:.3f} m '
          f'to the wrist centre.')
    print(f'  The tool reaches further than the wrist does — {K.NOMINAL_REACH:.2f} m — '
          f'but only\n  when it is NOT pointing straight down. That is the whole '
          f'reason this map\n  is smaller than the number on the datasheet.\n')
    return 0


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    height = BENCH_Z
    for i, a in enumerate(sys.argv):
        if a == '--height' and i + 1 < len(sys.argv):
            height = float(sys.argv[i + 1])
            args = [v for v in args if v != sys.argv[i + 1]]
    if len(args) >= 3:
        return print_point(float(args[0]), float(args[1]), float(args[2]))
    return print_map(height)


if __name__ == '__main__':
    sys.exit(main())
