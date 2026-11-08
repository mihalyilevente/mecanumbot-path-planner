import os
import textwrap

from mecanumbot_planner_ui.scenario_catalog import (
    find_pair,
    list_scenarios,
    load_scenario,
)


def test_list_and_load(tmp_path):
    content = textwrap.dedent(
        """\
        map: demo
        pairs:
          - name: a_to_b
            start: [0.0, 0.0, 0.0]
            goal: [1.0, 0.0, 0.0]
        """
    )
    (tmp_path / 'demo.yaml').write_text(content, encoding='utf-8')
    assert list_scenarios(str(tmp_path)) == ['demo']
    data = load_scenario(str(tmp_path), 'demo')
    assert data['map'] == 'demo'
    pair = find_pair(data, 'a_to_b')
    assert pair['goal'] == [1.0, 0.0, 0.0]


def test_repo_gap_scenario_if_present():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    scenarios = os.path.join(root, 'mecanumbot_sim_env', 'scenarios')
    if not os.path.isdir(scenarios):
        return
    names = list_scenarios(scenarios)
    assert 'test_gap_vs_detour' in names
    data = load_scenario(scenarios, 'test_gap_vs_detour')
    assert find_pair(data, 'through_or_around') is not None
