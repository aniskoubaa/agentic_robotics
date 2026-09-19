#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
Which TurtleBot are we talking to, and what does it publish?

The same idea as agr_uav_description's airframe registry, for the same
reason: the lessons should be written once and run on any of the robots, so
the per-robot differences live in one YAML file instead of being scattered
through every script as an if-statement.

    from agr_tb_tools.registry import get, names
    bot = get('tb4_standard')
    bot.camera          -> '/oakd/rgb/preview/image_raw'
    bot.docked_at_start -> True
"""
from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT = 'tb3_waffle'


class Robot:
    """One row of robots.yaml, with attribute access and no surprises."""

    def __init__(self, key: str, data: dict):
        self.key = key
        self.family = data['family']
        self.model = data['model']
        self.description = data.get('description', '')
        self.launch_package = data['launch_package']
        self.launch_file = data['launch_file']
        self.worlds = list(data.get('worlds') or [])
        self.cmd_vel = data['cmd_vel']
        self.cmd_vel_type = data.get('cmd_vel_type', 'TwistStamped')
        self.odom = data['odom']
        self.scan = data['scan']
        self.imu = data.get('imu')
        self.camera = data.get('camera')
        self.max_speed = float(data.get('max_speed', 0.2))
        self.max_turn = float(data.get('max_turn', 1.0))
        self.docked_at_start = bool(data.get('docked_at_start', False))

    @property
    def is_tb4(self) -> bool:
        return self.family == 'turtlebot4'

    @property
    def has_camera(self) -> bool:
        return bool(self.camera)

    @property
    def default_world(self) -> str:
        return self.worlds[0] if self.worlds else ''

    def __repr__(self) -> str:
        return f'<Robot {self.key} ({self.family}/{self.model})>'


def _load() -> dict:
    path = os.path.join(
        get_package_share_directory('agr_tb_tools'), 'config', 'robots.yaml')
    with open(path) as handle:
        return yaml.safe_load(handle) or {}


def names() -> list:
    return sorted(_load())


def get(key: str = DEFAULT) -> Robot:
    data = _load()
    if key not in data:
        raise KeyError(
            f'unknown robot {key!r}. Known: {", ".join(sorted(data))}')
    return Robot(key, data[key])


def all_robots() -> list:
    data = _load()
    return [Robot(k, v) for k, v in sorted(data.items())]


def main() -> int:
    """`ros2 run agr_tb_tools list_robots` — what can I start?"""
    print(f'\n{"NAME":<15}{"FAMILY":<13}{"CAMERA":<32}{"DOCKED":<8}{"MAX m/s":>8}')
    print('-' * 78)
    for bot in all_robots():
        print(f'{bot.key:<15}{bot.family:<13}'
              f'{(bot.camera or "-- none --"):<32}'
              f'{("yes" if bot.docked_at_start else "no"):<8}{bot.max_speed:>8.2f}')
    print('\nStart one with:   agr-sim turtlebot model:=<NAME>')
    print('Worlds per robot:')
    for bot in all_robots():
        print(f'  {bot.key:<15}{", ".join(bot.worlds)}')
    print('\nDOCKED=yes means the robot spawns on its charger and will NOT drive')
    print('until it is undocked. The examples call Base.ensure_ready(), which')
    print('does it for you and says so.\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
