# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Read-only access to config/airframes.yaml.

Everything that needs to know what an airframe can do goes through here, so
capability facts live in exactly one place. Adding a platform is a YAML edit,
not a code change.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory

_PACKAGE = 'agr_uav_description'


def registry_path() -> str:
    """Absolute path to the installed airframes.yaml."""
    return os.path.join(get_package_share_directory(_PACKAGE), 'config', 'airframes.yaml')


def _load_all() -> Dict[str, Any]:
    with open(registry_path(), 'r', encoding='utf-8') as fh:
        return yaml.safe_load(fh).get('airframes', {}) or {}


@dataclass(frozen=True)
class Airframe:
    """One airframe, as declared in airframes.yaml."""

    name: str
    px4_model: str
    sys_autostart: int
    description: str = ''
    camera_link: str = ''
    capabilities: Dict[str, Any] = field(default_factory=dict)

    @property
    def has_camera(self) -> bool:
        return bool(self.camera_link)

    def gz_model_name(self, instance: int = 0) -> str:
        """What Gazebo calls this vehicle.

        PX4 strips its own 'gz_' prefix and appends the instance number, so
        gz_x500_mono_cam instance 0 is the Gazebo model 'x500_mono_cam_0'.
        """
        base = self.px4_model[3:] if self.px4_model.startswith('gz_') else self.px4_model
        return f'{base}_{instance}'

    def gz_camera_topic(self, world: str, instance: int = 0) -> str:
        """Full Gazebo transport topic for this airframe's camera image."""
        return (f'/world/{world}/model/{self.gz_model_name(instance)}'
                f'/link/{self.camera_link}/sensor/camera/image')

    # -- capability shortcuts, so callers don't index dicts by hand ----------
    @property
    def airframe_class(self) -> str:
        return self.capabilities.get('airframe_class', 'unknown')

    @property
    def can_hover(self) -> bool:
        return bool(self.capabilities.get('can_hover', False))

    @property
    def cruise_speed_ms(self) -> float:
        return float(self.capabilities.get('cruise_speed_ms', 0.0))

    @property
    def min_turn_radius_m(self) -> float:
        return float(self.capabilities.get('min_turn_radius_m', 0.0))


def list_airframes() -> List[str]:
    """Names of every registered airframe, sorted."""
    return sorted(_load_all().keys())


def load_airframe(name: str) -> Airframe:
    """Look up one airframe by registry name.

    Raises KeyError naming the valid options — a typo in a launch argument
    should say what was expected, not fail somewhere deep in PX4.
    """
    entries = _load_all()
    if name not in entries:
        raise KeyError(
            f"unknown airframe {name!r}; registered: {', '.join(sorted(entries))}"
        )
    entry = entries[name]
    return Airframe(
        name=name,
        px4_model=entry['px4_model'],
        sys_autostart=int(entry['sys_autostart']),
        description=entry.get('description', ''),
        camera_link=entry.get('camera_link', '') or '',
        capabilities=entry.get('capabilities', {}) or {},
    )
