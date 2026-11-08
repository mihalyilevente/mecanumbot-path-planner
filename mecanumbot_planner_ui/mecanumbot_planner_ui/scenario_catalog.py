"""Load start/goal scenario YAML files (no rclpy)."""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml


def list_scenarios(scenarios_dir: str) -> List[str]:
    """Return scenario basenames (without .yaml), sorted."""
    if not scenarios_dir or not os.path.isdir(scenarios_dir):
        return []
    names = []
    for entry in os.listdir(scenarios_dir):
        if entry.endswith('.yaml'):
            names.append(entry[:-5])
    return sorted(names)


def scenario_path(scenarios_dir: str, name: str) -> str:
    if name.endswith('.yaml'):
        name = name[:-5]
    if os.path.sep in name or (os.path.altsep and os.path.altsep in name):
        raise ValueError('Scenario name must be a bare basename')
    path = os.path.join(scenarios_dir, name + '.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Scenario not found: {name}')
    return path


def load_scenario(scenarios_dir: str, name: str) -> Dict[str, Any]:
    path = scenario_path(scenarios_dir, name)
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or 'pairs' not in data:
        raise ValueError(f'Invalid scenario file: {name}')
    return data


def find_pair(scenario: Dict[str, Any], pair_name: str) -> Optional[Dict[str, Any]]:
    for pair in scenario.get('pairs', []):
        if pair.get('name') == pair_name:
            return pair
    return None
