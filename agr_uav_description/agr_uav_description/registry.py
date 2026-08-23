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
    capabilities: Dict[str, Any] = field(default_factory=dict)

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
        capabilities=entry.get('capabilities', {}) or {},
    )
