# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Print the airframe registry.  ros2 run agr_uav_tools list_airframes"""
from agr_uav_description import list_airframes, load_airframe


def main() -> None:
    print(f"{'NAME':<16}{'PX4 MODEL':<22}{'AUTOSTART':<11}{'CLASS':<12}HOVER")
    for name in list_airframes():
        a = load_airframe(name)
        print(f'{a.name:<16}{a.px4_model:<22}{a.sys_autostart:<11}'
              f'{a.airframe_class:<12}{"yes" if a.can_hover else "NO"}')
        if a.description:
            print(f'    {a.description}')
