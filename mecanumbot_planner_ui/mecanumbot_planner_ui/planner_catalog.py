"""Planner IDs, harness colours, and behavior-tree filename mapping."""

from __future__ import annotations

import os
from typing import Dict, List

# Colours match mecanumbot_sim_env harness / result pictures.
PLANNERS: List[Dict[str, str]] = [
    {'id': 'Shortest', 'colour': '#1f77b4'},
    {'id': 'Safest', 'colour': '#2ca02c'},
    {'id': 'Smoothest', 'colour': '#d62728'},
    {'id': 'Combined', 'colour': '#9467bd'},
    {'id': 'NavFn', 'colour': '#ff7f0e'},
]

PLANNER_IDS = [entry['id'] for entry in PLANNERS]
COLOURS = {entry['id']: entry['colour'] for entry in PLANNERS}


def behavior_tree_path(trees_dir: str, planner_id: str) -> str:
    """Absolute path to navigate_to_pose_<planner>.xml for a planner id."""
    if planner_id not in PLANNER_IDS:
        raise ValueError(f'Unknown planner_id: {planner_id}')
    return os.path.join(trees_dir, f'navigate_to_pose_{planner_id.lower()}.xml')


def planner_list() -> List[Dict[str, str]]:
    return [dict(entry) for entry in PLANNERS]
