#!/usr/bin/env python3
"""
UAV example 06 — what can this airframe actually do?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_examples 06_list_airframes

WHAT  Print every registered airframe and its capabilities, then explain what
      each one makes impossible.

LEARN - A MISSION IS NOT FEASIBLE IN THE ABSTRACT — it is feasible FOR A
        PLATFORM. "Loiter here" is trivial for a multirotor and impossible for
        a fixed-wing, which cannot stop. Any code that reasons about missions
        has to read capabilities, not guess from the name.
      - This is the whole reason airframes.yaml exists. Capability facts live
        in exactly one place, so adding a platform is a YAML edit rather than
        a search-and-replace through every planner.
      - No ROS needed. This reads a config file — a reminder that "robotics
        code" is mostly not middleware.
"""
from agr_uav_description import list_airframes, load_airframe


def main():
    names = list_airframes()
    print(f'\n{"NAME":<16}{"CLASS":<12}{"HOVER":<7}{"CAMERA":<8}'
          f'{"MIN AIRSPEED":<14}{"TURN RADIUS"}')
    print('-' * 72)
    for n in names:
        a = load_airframe(n)
        print(f'{n:<16}{a.airframe_class:<12}'
              f'{"yes" if a.can_hover else "NO":<7}'
              f'{"yes" if a.has_camera else "-":<8}'
              f'{a.capabilities.get("min_airspeed_ms", 0):<14.1f}'
              f'{a.min_turn_radius_m:.0f} m')

    print('\nWhat each one makes IMPOSSIBLE:')
    for n in names:
        a = load_airframe(n)
        if not a.can_hover:
            print(f'  {n:<16} cannot hover — every "loiter here" becomes an '
                  f'orbit of at least {a.min_turn_radius_m:.0f} m')
        elif a.capabilities.get('min_airspeed_ms', 0) > 0:
            print(f'  {n:<16} needs {a.capabilities["min_airspeed_ms"]:.0f} m/s '
                  f'in forward flight — a geometrically perfect mission can '
                  f'still be infeasible')
        else:
            print(f'  {n:<16} hovers — the baseline; nothing is ruled out')
    print()


if __name__ == '__main__':
    main()
