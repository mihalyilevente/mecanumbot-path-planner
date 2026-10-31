"""Score drive_scenarios logs on travel time and rolling energy.

    python3 -m mecanumbot_eval.compare_runs results/drive_test_slalom_20261031_120000
    python3 -m mecanumbot_eval.compare_runs results/drive_test_slalom_20261031_120000 --plans results/test_slalom_20261031_120000
"""

import argparse
import csv
import math
import os

from mecanumbot_eval.kinematics import RUN_FIELDS, duration_of, rolling_energy

PLANNER_ORDER = ['Shortest', 'Safest', 'Smoothest', 'Combined', 'NavFn']
METRIC_FIELDS = [
    'pair', 'planner', 'success', 'duration_s', 'energy_j', 'wheel_m',
    'driven_length_m', 'planned_length_m', 'final_error_m', 'replans',
    'planning_time_ms', 'total_turn_deg', 'mean_cost', 'max_cost',
]
SCORE_FIELDS = ['pair', 'planner', 'time_ratio', 'energy_ratio', 'score', 'rank']


def planner_rank(name):
    try:
        return PLANNER_ORDER.index(name)
    except ValueError:
        return len(PLANNER_ORDER)


def as_bool(value):
    return str(value).strip().lower() in ('true', '1', 'yes')


def as_float(value):
    if value is None:
        return None
    text = str(value).strip()
    if text == '':
        return None
    return float(text)


def read_csv(path):
    with open(path, newline='') as stream:
        return list(csv.DictReader(stream))


def load_run(path):
    samples = []
    for raw in read_csv(path):
        samples.append({key: float(raw[key]) for key in RUN_FIELDS})
    return samples


def load_drive(drive_dir):
    summary_path = os.path.join(drive_dir, 'summary.csv')
    if not os.path.isfile(summary_path):
        raise SystemExit(f'no summary.csv in {drive_dir}')
    rows = []
    for raw in read_csv(summary_path):
        pair = raw['pair']
        planner = raw['planner']
        run_path = os.path.join(drive_dir, 'runs', f'{pair}_{planner}.csv')
        samples = load_run(run_path) if os.path.isfile(run_path) else []
        wheel_m, energy_j = rolling_energy(samples)
        duration = as_float(raw.get('duration_s'))
        if duration is None:
            duration = duration_of(samples)
        rows.append({
            'pair': pair,
            'planner': planner,
            'success': as_bool(raw.get('success')),
            'duration_s': duration,
            'energy_j': energy_j,
            'wheel_m': wheel_m,
            'driven_length_m': as_float(raw.get('driven_length_m')),
            'planned_length_m': as_float(raw.get('planned_length_m')),
            'final_error_m': as_float(raw.get('final_error_m')),
            'replans': as_float(raw.get('replans')),
            'planning_time_ms': None,
            'total_turn_deg': None,
            'mean_cost': None,
            'max_cost': None,
        })
    return rows


def attach_plans(rows, plans_dir):
    plan_path = os.path.join(plans_dir, 'results.csv')
    if not os.path.isfile(plan_path):
        raise SystemExit(f'no results.csv in {plans_dir}')
    found = {(raw['pair'], raw['planner']): raw for raw in read_csv(plan_path)}
    for row in rows:
        raw = found.get((row['pair'], row['planner']))
        if raw is None:
            continue
        row['planning_time_ms'] = as_float(raw.get('planning_time_ms'))
        row['total_turn_deg'] = as_float(raw.get('total_turn_deg'))
        row['mean_cost'] = as_float(raw.get('mean_cost'))
        row['max_cost'] = as_float(raw.get('max_cost'))


def usable(row):
    return (row['success'] and row['duration_s'] is not None and row['energy_j'] is not None
            and row['duration_s'] >= 0.0 and row['energy_j'] >= 0.0)


def ratio(value, baseline):
    if baseline > 0.0:
        return value / baseline
    if value == 0.0:
        return 0.0
    return math.inf


def score_pairs(rows, alpha):
    """Per pair, normalise time and energy by the best usable planner on that pair."""
    by_pair = {}
    for row in rows:
        by_pair.setdefault(row['pair'], []).append(row)
    scored = []
    for pair in sorted(by_pair):
        ok = [row for row in by_pair[pair] if usable(row)]
        if not ok:
            continue
        best_time = min(row['duration_s'] for row in ok)
        best_energy = min(row['energy_j'] for row in ok)
        ranked = []
        for row in ok:
            time_ratio = ratio(row['duration_s'], best_time)
            energy_ratio = ratio(row['energy_j'], best_energy)
            score = alpha * time_ratio + (1.0 - alpha) * energy_ratio
            ranked.append({
                'pair': pair,
                'planner': row['planner'],
                'time_ratio': time_ratio,
                'energy_ratio': energy_ratio,
                'score': score,
            })
        ranked.sort(key=lambda item: (item['score'], planner_rank(item['planner'])))
        for index, item in enumerate(ranked, start=1):
            item['rank'] = index
        scored.extend(ranked)
    return scored


def recommend(rows, scored):
    """Fewest missed pairs, then lowest mean score, then planner list order."""
    pairs = sorted({row['pair'] for row in rows})
    n_pairs = len(pairs)
    finished = {}
    for item in scored:
        finished.setdefault(item['planner'], []).append(item['score'])
    names = sorted({row['planner'] for row in rows}, key=planner_rank)
    if not names:
        return None

    def sort_key(name):
        scores = finished.get(name, [])
        missed = n_pairs - len(scores)
        mean = sum(scores) / len(scores) if scores else math.inf
        return (missed, mean, planner_rank(name))

    chosen = min(names, key=sort_key)
    if not finished.get(chosen):
        return None
    return chosen


def mean_score(scored, planner):
    values = [item['score'] for item in scored if item['planner'] == planner]
    if not values:
        return None
    return sum(values) / len(values)


def write_csv(path, fieldnames, rows):
    with open(path, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_field(key, row.get(key)) for key in fieldnames})


def format_field(key, value):
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        if math.isinf(value):
            return ''
        digits = 2 if key == 'duration_s' else 4
        return f'{value:.{digits}f}'
    return value


def format_score_rows(scored):
    written = []
    for item in scored:
        written.append({
            'pair': item['pair'],
            'planner': item['planner'],
            'time_ratio': item['time_ratio'],
            'energy_ratio': item['energy_ratio'],
            'score': item['score'],
            'rank': item['rank'],
        })
    return written


def summarise(rows, scored, alpha):
    names = sorted({row['planner'] for row in rows}, key=planner_rank)
    lines = [f'alpha {alpha:g}   score = alpha * time_ratio + (1 - alpha) * energy_ratio', '']
    for label, weight in (('energy', 0.0), ('balanced', 0.5), ('time', 1.0)):
        table = score_pairs(rows, weight)
        winner = recommend(rows, table)
        lines.append(f'{label} (alpha {weight:g}): {winner if winner else "none"}')
    lines.append('')
    lines.append(f"{'planner':<12}{'ok':>4}{'missed':>8}{'mean time s':>14}{'mean energy J':>16}{'mean score':>12}")
    pair_count = len({row['pair'] for row in rows})
    for name in names:
        own = [row for row in rows if row['planner'] == name]
        done = [row for row in own if usable(row)]
        missed = pair_count - len(done)
        time_mean = sum(row['duration_s'] for row in done) / len(done) if done else None
        energy_mean = sum(row['energy_j'] for row in done) / len(done) if done else None
        score_mean = mean_score(scored, name)
        lines.append(
            f"{name:<12}{len(done):>4}{missed:>8}"
            f"{_cell(time_mean, 14, 2)}{_cell(energy_mean, 16, 4)}{_cell(score_mean, 12, 4)}")
    chosen = recommend(rows, scored)
    lines.append('')
    if chosen is None:
        lines.append('recommended: none')
    else:
        lines.append(f'recommended at alpha {alpha:g}: {chosen}')
    return '\n'.join(lines) + '\n'


def _cell(value, width, digits):
    if value is None:
        return f"{'':>{width}}"
    return f'{value:>{width}.{digits}f}'


def print_metrics(rows):
    header = (f"{'pair':<18}{'planner':<12}{'ok':<4}{'time s':>8}"
              f"{'energy J':>10}{'wheel m':>10}{'driven m':>10}")
    print(header)
    print('-' * len(header))
    for row in rows:
        ok = 'yes' if row['success'] else 'NO'
        print(f"{row['pair']:<18}{row['planner']:<12}{ok:<4}"
              f"{_cell(row['duration_s'], 8, 2)}{_cell(row['energy_j'], 10, 4)}"
              f"{_cell(row['wheel_m'], 10, 3)}{_cell(row['driven_length_m'], 10, 3)}")


def evaluate(drive_dir, plans_dir=None, alpha=0.5, out_dir=None):
    rows = load_drive(drive_dir)
    if plans_dir:
        attach_plans(rows, plans_dir)
    scored = score_pairs(rows, alpha)
    text = summarise(rows, scored, alpha)
    destination = out_dir or os.path.join(drive_dir, 'eval')
    os.makedirs(destination, exist_ok=True)
    write_csv(os.path.join(destination, 'metrics.csv'), METRIC_FIELDS, rows)
    write_csv(os.path.join(destination, 'scores.csv'), SCORE_FIELDS, format_score_rows(scored))
    report_path = os.path.join(destination, 'recommendation.txt')
    with open(report_path, 'w', newline='\n') as stream:
        stream.write(text)
    return rows, scored, text, destination


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('drive_dir', help='Folder written by drive_scenarios (contains summary.csv)')
    parser.add_argument('--plans', help='Folder written by plan_scenarios (contains results.csv)')
    parser.add_argument('--alpha', type=float, default=0.5,
                        help='Weight on travel time. 0 is energy only, 1 is time only')
    parser.add_argument('--out', help='Output folder. Default: <drive_dir>/eval')
    args = parser.parse_args()
    if not 0.0 <= args.alpha <= 1.0:
        parser.error('alpha must be between 0 and 1')
    rows, _, text, destination = evaluate(args.drive_dir, args.plans, args.alpha, args.out)
    print_metrics(rows)
    print()
    print(text, end='')
    print(f'wrote {os.path.abspath(destination)}')


if __name__ == '__main__':
    main()
