#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 01 — keyboard teleoperation of the UR5e (+ Husky).

WHAT  Drive the robot BY HAND from the keyboard. You nudge one arm joint at a
      time, open/close the gripper, and (for Task B) drive the Husky base. This
      is how you will perform the demonstrations that Script 03 records.

WHY   A VLA learns by COPYING demonstrations. Before we can record any, we need
      a way to move the robot ourselves. This is the lowest level of control in
      the whole school: we publish raw joint-position targets, not named poses.

LEARN - The arm is 6 JointPositionController topics, one per joint:
            /ur5e_<joint>/cmd   (std_msgs/Float64, radians, ABSOLUTE target)
        We keep a target vector and publish it whenever you press a key.
      - The gripper is the Robotiq driving knuckle PLUS five mimic joints that
        DART won't move for us — so we publish all six (see vla_client.base).
      - The Husky base is the same /cmd_vel Twist you used on Day 1.

      The joint names + gripper mimic map come from vla_client.base, so teleop,
      recording, and the VLA executor all agree on ONE command space.

Run while `raise-sim` is up:

    ros2 run raise2026_labs 01_teleop.py            # arm only
    ros2 run raise2026_labs 01_teleop.py --task B   # also enable base driving
    # or:  01_d3
"""

import argparse
import sys
import termios
import tty
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Make the shared vla_client package importable from this starter script.
# (api_clients/ lives two levels up, under day2/.)
try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=False)
DAY2 = _d2paths.DAY2
from vla_client.base import (   # noqa: E402
    UR5E_JOINTS, JOINT_LIMITS,
    GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_OPEN, GRIPPER_CLOSED,
)

STEP = 0.05          # radians moved per key press
BASE_SPEED = 0.3     # m/s and rad/s for Husky driving (Task B)

# Keys 1..6 select+increase a joint; q,w,e,r,t,y decrease the same joints.
INC_KEYS = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5}
DEC_KEYS = {'q': 0, 'w': 1, 'e': 2, 'r': 3, 't': 4, 'y': 5}

HELP = """
┌──────────────── RAISE 2026 teleop ────────────────┐
│ Arm joints:  1..6 = +{step}rad   q w e r t y = -{step}rad
│ Gripper:     o = open      c = close
│ Base:        {base_row}
│ Misc:        h = help        x = quit
└────────────────────────────────────────────────────┘
"""
BASE_ROW_ON = "i/k = fwd/back   j/l = turn   (space = stop)"
BASE_ROW_OFF = "OFF in this mode — restart with --task B to drive the Husky"


def help_text(enable_base: bool) -> str:
    return HELP.format(step=STEP, base_row=BASE_ROW_ON if enable_base else BASE_ROW_OFF)


class Teleop(Node):
    def __init__(self, enable_base: bool):
        super().__init__('raise_teleop')
        self.enable_base = enable_base

        # One publisher per arm joint command topic.
        self.arm_pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10) for j in UR5E_JOINTS}
        # Gripper: driving knuckle + every mimic sibling (DART needs all of them).
        self.grip_pubs = {GRIPPER_KNUCKLE: self.create_publisher(Float64, f'/{GRIPPER_KNUCKLE}/cmd', 10)}
        for j in GRIPPER_MIMIC_SIGNS:
            self.grip_pubs[j] = self.create_publisher(Float64, f'/{j}/cmd', 10)
        if enable_base:
            self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # The target we hold and re-publish. Start near a neutral pose.
        self.target = [0.0, -1.0, 1.0, -1.4, -1.57, 0.0]
        self.gripper = GRIPPER_OPEN
        self.publish_arm()
        self.publish_gripper()

    def publish_arm(self):
        for j, v in zip(UR5E_JOINTS, self.target):
            self.arm_pubs[j].publish(Float64(data=v))

    def publish_gripper(self):
        # Driving knuckle at the target, each mimic at sign * target.
        self.grip_pubs[GRIPPER_KNUCKLE].publish(Float64(data=self.gripper))
        for j, sign in GRIPPER_MIMIC_SIGNS.items():
            self.grip_pubs[j].publish(Float64(data=sign * self.gripper))

    def publish_base(self, lin: float, ang: float):
        if self.enable_base:
            t = Twist()
            t.linear.x, t.angular.z = lin, ang
            self.base_pub.publish(t)
        else:
            # Student report 2026-07-14: keys looked dead. Say WHY, loudly.
            print('  (base driving is OFF — restart with:  01_d3 --task B)')

    def handle_key(self, k: str) -> bool:
        """Apply one keypress. Return False to quit."""
        if k in INC_KEYS or k in DEC_KEYS:
            idx = INC_KEYS.get(k, DEC_KEYS.get(k))
            delta = STEP if k in INC_KEYS else -STEP
            lo, hi = JOINT_LIMITS[UR5E_JOINTS[idx]]
            self.target[idx] = max(lo, min(hi, self.target[idx] + delta))
            self.publish_arm()
            print(f'  joint {idx} ({UR5E_JOINTS[idx]}) -> {self.target[idx]:+.3f} rad')
        elif k == 'o':
            self.gripper = GRIPPER_OPEN; self.publish_gripper(); print('  gripper OPEN')
        elif k == 'c':
            self.gripper = GRIPPER_CLOSED; self.publish_gripper(); print('  gripper CLOSE')
        elif k == 'i':
            self.publish_base(BASE_SPEED, 0.0)
        elif k == 'k':
            self.publish_base(-BASE_SPEED, 0.0)
        elif k == 'j':
            self.publish_base(0.0, BASE_SPEED)
        elif k == 'l':
            self.publish_base(0.0, -BASE_SPEED)
        elif k == ' ':
            self.publish_base(0.0, 0.0)
        elif k == 'h':
            print(help_text(self.enable_base))
        elif k == 'x':
            return False
        return True


def read_key() -> str:
    """Read ONE keypress without waiting for Enter (raw terminal mode)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 keyboard teleop')
    ap.add_argument('--task', default='A', help='A | C | B  (B enables base driving)')
    args = ap.parse_args()

    rclpy.init()
    node = Teleop(enable_base=args.task.upper() == 'B')
    print(help_text(node.enable_base))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not node.handle_key(read_key()):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
