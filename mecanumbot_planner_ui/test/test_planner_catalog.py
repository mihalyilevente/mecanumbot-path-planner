import os

from mecanumbot_planner_ui.planner_catalog import (
    PLANNER_IDS,
    behavior_tree_path,
    planner_list,
)


def test_planner_list_has_five():
    entries = planner_list()
    assert [e['id'] for e in entries] == PLANNER_IDS
    assert len(entries) == 5


def test_behavior_tree_path():
    path = behavior_tree_path('/trees', 'Safest')
    assert path == os.path.join('/trees', 'navigate_to_pose_safest.xml')
