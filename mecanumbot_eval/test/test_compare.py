import csv
import os

from mecanumbot_eval.compare_runs import evaluate, recommend, score_pairs
from mecanumbot_eval.kinematics import RUN_FIELDS


def _write_run(folder, pair, planner, twists):
    path = os.path.join(folder, 'runs', f'{pair}_{planner}.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(RUN_FIELDS)
        for twist in twists:
            t, vx, vy, wz = twist
            writer.writerow([t, 0, 0, 0, vx, vy, wz, vx, vy, wz])


def _summary(folder, rows):
    with open(os.path.join(folder, 'summary.csv'), 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            'pair', 'planner', 'success', 'duration_s', 'planned_length_m',
            'driven_length_m', 'final_error_m', 'replans'])
        writer.writeheader()
        writer.writerows(rows)


def test_energy_winner_and_time_winner(tmp_path):
    # Shortest finishes sooner and spins the whole way. Safest takes longer and barely rolls.
    _write_run(tmp_path, 'gap', 'Shortest', [(0.0, 0.0, 0.0, 1.0), (4.0, 0.0, 0.0, 1.0)])
    _write_run(tmp_path, 'gap', 'Safest', [(0.0, 0.05, 0.0, 0.0), (10.0, 0.05, 0.0, 0.0)])
    _summary(tmp_path, [
        {'pair': 'gap', 'planner': 'Shortest', 'success': True, 'duration_s': 4.0,
         'planned_length_m': 1.0, 'driven_length_m': 0.0, 'final_error_m': 0.1, 'replans': 0},
        {'pair': 'gap', 'planner': 'Safest', 'success': True, 'duration_s': 10.0,
         'planned_length_m': 3.0, 'driven_length_m': 2.0, 'final_error_m': 0.1, 'replans': 1},
    ])
    rows, _, text, destination = evaluate(str(tmp_path), alpha=0.5)
    by_planner = {row['planner']: row['energy_j'] for row in rows}
    assert by_planner['Safest'] < by_planner['Shortest']

    energy_table = score_pairs(rows, 0.0)
    time_table = score_pairs(rows, 1.0)
    assert energy_table[0]['planner'] == 'Safest'
    assert time_table[0]['planner'] == 'Shortest'
    assert 'energy (alpha 0): Safest' in text
    assert 'time (alpha 1): Shortest' in text
    assert os.path.isfile(os.path.join(destination, 'metrics.csv'))
    assert os.path.isfile(os.path.join(destination, 'scores.csv'))


def test_failed_run_is_not_the_energy_winner(tmp_path):
    _write_run(tmp_path, 'gap', 'Shortest', [(0.0, 0.2, 0.0, 0.0), (5.0, 0.2, 0.0, 0.0)])
    _summary(tmp_path, [
        {'pair': 'gap', 'planner': 'Shortest', 'success': True, 'duration_s': 5.0,
         'planned_length_m': 1, 'driven_length_m': 1, 'final_error_m': 0.05, 'replans': 0},
        {'pair': 'gap', 'planner': 'NavFn', 'success': False, 'duration_s': '',
         'planned_length_m': '', 'driven_length_m': '', 'final_error_m': '', 'replans': 0},
    ])
    rows, scored, text, _ = evaluate(str(tmp_path))
    assert [item['planner'] for item in scored] == ['Shortest']
    assert 'recommended at alpha 0.5: Shortest' in text
    assert recommend(rows, scored) == 'Shortest'


def test_equal_score_keeps_planner_list_order():
    rows = [
        {'pair': 'room', 'planner': 'NavFn', 'success': True, 'duration_s': 5.0, 'energy_j': 1.0},
        {'pair': 'room', 'planner': 'Shortest', 'success': True, 'duration_s': 5.0, 'energy_j': 1.0},
    ]
    scored = score_pairs(rows, 0.5)
    assert scored[0]['planner'] == 'Shortest'
    assert scored[0]['rank'] == 1
    assert recommend(rows, scored) == 'Shortest'


def test_plan_columns_are_copied(tmp_path):
    _write_run(tmp_path, 'gap', 'Shortest', [(0.0, 0.2, 0.0, 0.0), (5.0, 0.2, 0.0, 0.0)])
    _summary(tmp_path, [
        {'pair': 'gap', 'planner': 'Shortest', 'success': True, 'duration_s': 5.0,
         'planned_length_m': 1, 'driven_length_m': 1, 'final_error_m': 0.05, 'replans': 0},
    ])
    plans = tmp_path / 'plans'
    plans.mkdir()
    with open(plans / 'results.csv', 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            'pair', 'planner', 'success', 'length_m', 'poses', 'planning_time_ms',
            'total_turn_deg', 'mean_cost', 'max_cost'])
        writer.writeheader()
        writer.writerow({
            'pair': 'gap', 'planner': 'Shortest', 'success': True, 'length_m': 1.2,
            'poses': 8, 'planning_time_ms': 3.5, 'total_turn_deg': 90.0,
            'mean_cost': 12.0, 'max_cost': 40,
        })
    rows, _, _, destination = evaluate(str(tmp_path), plans_dir=str(plans))
    assert rows[0]['planning_time_ms'] == 3.5
    assert rows[0]['total_turn_deg'] == 90.0
    with open(os.path.join(destination, 'metrics.csv'), newline='') as stream:
        written = list(csv.DictReader(stream))
    assert written[0]['planning_time_ms'] == '3.5000'
    assert written[0]['mean_cost'] == '12.0000'


def test_missing_summary_exits(tmp_path):
    try:
        evaluate(str(tmp_path))
    except SystemExit as exc:
        assert 'summary.csv' in str(exc)
    else:
        raise AssertionError('expected SystemExit')
