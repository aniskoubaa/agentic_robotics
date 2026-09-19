#!/usr/bin/env python3
"""
TurtleBot example 02 — make it move.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 02_drive
    ros2 run agr_tb_examples 02_drive --ros-args -p speed:=0.15 -p seconds:=3.0

WHAT  Publish a velocity, hold it, stop. Then report what the wheels actually
      did, which is not always what you asked for.

LEARN - THE MESSAGE IS geometry_msgs/TwistStamped, NOT Twist. This changed in
        Jazzy and it is the single most common reason a TurtleBot script does
        nothing at all: publish a Twist to /cmd_vel and there is no error, no
        warning, and no movement. Check before you debug anything else:
            ros2 topic info /cmd_vel
      - VELOCITY COMMANDS DO NOT LATCH. Unlike a joint position controller,
        the base stops when the messages stop — there is a watchdog. So
        driving means publishing repeatedly, which is what Base.drive() does
        at 20 Hz, and stopping means publishing zeros rather than falling
        silent.
      - ON A TURTLEBOT 4 THE ROBOT MAY REFUSE. It spawns docked, and a docked
        Create 3 ignores velocity entirely. Worse, once near the dock its
        REFLEX layer can override you and drive backwards at 0.14 m/s.
        ensure_ready() undocks first; this script reports it if a reflex
        fires anyway.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base


def main():
    rclpy.init()
    node = Node('drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.declare_parameter('speed', 0.2)
    node.declare_parameter('turn', 0.0)
    node.declare_parameter('seconds', 4.0)
    speed = float(node.get_parameter('speed').value)
    turn = float(node.get_parameter('turn').value)
    seconds = float(node.get_parameter('seconds').value)

    bot = Base(node)
    if not bot.wait_for_state():
        print(f'nothing on {bot.robot.odom} — is `agr-sim turtlebot` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    print(f'\n{bot.robot.key}: publishing {bot.robot.cmd_vel_type} '
          f'on {bot.robot.cmd_vel} at 20 Hz')
    bot.ensure_ready()

    print(f'  commanding {speed:+.2f} m/s, {turn:+.2f} rad/s for {seconds:.1f} s ...')
    r = bot.drive(speed, turn, seconds)

    print(f'\n  asked for      {r["commanded"]:+.3f} m/s')
    print(f'  wheels reached {r["measured_max"]:+.3f} m/s (max), '
          f'{r["measured_min"]:+.3f} m/s (min)')
    average = r['moved'] / r['seconds']
    print(f'  travelled      {r["moved"]:.3f} m in {r["seconds"]:.1f} s '
          f'= {average:.3f} m/s average')
    # Say what the numbers show, not what they usually show. Over a short run
    # the acceleration ramp costs a visible fraction; over a longer one it
    # washes out and the average lands on the command.
    shortfall = abs(r['commanded']) - abs(average)
    if shortfall > 0.02:
        print(f'  that is {shortfall:.3f} m/s under the command — the ramp at each\n'
              f'  end costs more the shorter the run. Try seconds:=10 and watch\n'
              f'  the average climb towards {abs(r["commanded"]):.2f}.')
    else:
        print('  the average has essentially reached the command: over a run this\n'
              '  long the acceleration ramp at each end no longer shows.')
    if r['reflex_reversed']:
        print('\n  A REFLEX FIRED. The wheels went backwards while you were asking\n'
              '  them to go forwards. On a Create 3 that is REFLEX_DOCK_AVOID,\n'
              '  REFLEX_CLIFF or REFLEX_BUMP deciding it knows better. Drive\n'
              '  further from the dock, or use the built-in actions:\n'
              '      ros2 action list')
    print(f'\n  {bot.describe()}\n')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
