#!/usr/bin/env python3
"""
TurtleBot — the showcase: a narrated tour of what this robot can do.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_demos demo_tour
    ros2 run agr_tb_demos demo_tour --ros-args -p seconds:=30.0

WHAT  Six numbered acts, each printing what it is about to try and then what
      actually happened. It reports measurements, not opinions: distances
      from odometry, clearances from the lidar, and — on a TurtleBot 4 —
      whether the Create 3's reflexes overrode the command.

The acts:
    1. read the robot's own state
    2. undock, if it is a TurtleBot 4 sitting on its charger
    3. drive a measured metre, and compare with what was asked
    4. turn on the spot, closed on odometry
    5. look around with the lidar and report the clearances
    6. wander with obstacle avoidance for a while, then stop

Run `ros2 run agr_tb_demos diagnose` first if anything here surprises you.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base


def say(text=''):
    print(text, flush=True)


def act(number, title):
    say(f'\n  --- {number}. {title} ---')


def main() -> int:
    rclpy.init()
    node = Node('demo_tour', parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.declare_parameter('seconds', 25.0)
    wander_for = float(node.get_parameter('seconds').value)

    bot = Base(node)
    say(f'\nTurtleBot tour — {bot.robot.key}\n' + '=' * 62)
    say(f'  {bot.robot.description}')

    if not bot.wait_for_state():
        say(f'\n  nothing on {bot.robot.odom}. Start the simulator first:')
        say(f'      agr-sim turtlebot model:={bot.robot.key}\n')
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    bot.sleep(2.0)

    act(1, 'what the robot knows about itself')
    say(f'    {bot.describe()}')
    say(f'    drive topic {bot.robot.cmd_vel} ({bot.robot.cmd_vel_type}), '
        f'lidar {bot.robot.scan}')
    say(f'    camera: {bot.robot.camera or "none — this model has no camera"}')

    act(2, 'getting permission to move')
    if bot.robot.is_tb4:
        say('    A TurtleBot 4 spawns on its charger, and a docked Create 3')
        say('    ignores velocity commands entirely. Undocking is not optional.')
        t0 = time.time()
        bot.ensure_ready()
        say(f'    took {time.time() - t0:.0f} s; docked={bot.is_docked}')
    else:
        say('    A TurtleBot 3 has no dock and no permission to ask for.')
        bot.ensure_ready()

    act(3, 'driving a measured metre')
    speed = min(0.15, bot.robot.max_speed * 0.6)
    r = bot.drive(speed, seconds=1.0 / speed)
    say(f'    asked for 1.000 m at {speed:.2f} m/s; odometry says '
        f'{r["moved"]:.3f} m')
    say(f'    wheels peaked at {r["measured_max"]:.3f} m/s')
    if r['reflex_reversed']:
        say('    A REFLEX FIRED — the Create 3 drove backwards mid-command.')
    say('    The shortfall is the acceleration ramp at each end, not drift.')

    act(4, 'turning on the spot')
    err = bot.turn(math.pi / 2)
    say(f'    asked for 90 deg, overshot by {math.degrees(err):+.1f} deg')
    say('    closed on odometry, not on a timer — a timer is wrong by')
    say('    whatever the acceleration ramp costs, and that compounds.')

    act(5, 'looking around')
    bot.sleep(1.0)
    for label, bearing in (('ahead', 0.0), ('left', math.pi / 2),
                           ('right', -math.pi / 2)):
        if label == 'ahead':
            d = bot.range_ahead()
        else:
            bot.turn(bearing)
            d = bot.range_ahead()
            bot.turn(-bearing)
        shown = f'{d:.2f} m' if math.isfinite(d) else 'nothing in range'
        say(f'    {label:<6} {shown}')

    act(6, f'wandering for {wander_for:.0f} s with obstacle avoidance')
    say('    Sense, decide, act — forty times a second, with no map and no')
    say('    memory. Watch it fail to notice it has been here before.')
    turns = _wander(node, bot, wander_for)
    say(f'    {turns} avoidance turns; ended at {bot.describe()}')

    say('\n' + '=' * 62)
    say('Tour complete. Next:')
    say('    ros2 run agr_tb_teleop teleop_keyboard      drive it yourself')
    say('    ros2 run agr_tb_examples 05_drive_a_square  watch odometry drift')
    say('')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def _wander(node, bot, duration):
    """The same loop as 06_avoid_obstacle, inlined so the demo stands alone."""
    stop_at, clear_at = 0.55, 0.85
    speed = min(0.18, bot.robot.max_speed * 0.7)
    turn = min(0.7, bot.robot.max_turn * 0.4)
    turning, turns = 0.0, 0
    end = time.time() + duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        ahead = bot.range_ahead()
        if turning == 0.0 and math.isfinite(ahead) and ahead < stop_at:
            turning = 1.0 if _space(bot.scan, +1) >= _space(bot.scan, -1) else -1.0
            turns += 1
        elif turning != 0.0 and (not math.isfinite(ahead) or ahead > clear_at):
            turning = 0.0
        bot.send(0.0 if turning else speed, turning * turn)
    bot.stop()
    return turns


def _space(scan, sign):
    if scan is None:
        return 0.0
    total, count = 0.0, 0
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if 0.2 < sign * angle < math.radians(50) + 0.2:
            total += r if math.isfinite(r) else scan.range_max
            count += 1
    return total / count if count else 0.0


if __name__ == '__main__':
    sys.exit(main())
