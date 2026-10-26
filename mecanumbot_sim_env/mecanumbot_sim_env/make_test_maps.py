"""Generate the designed test maps and their scenario files.

Each map is a small room built from rectangles and circles (in metres), made
so that the planning criteria disagree in a known way. For every map this
writes maps/<name>/<name>.pgm + .yaml and scenarios/<name>.yaml. Run it from
the package folder after changing a layout:

    python3 -m mecanumbot_sim_env.make_test_maps        # or: ros2 run mecanumbot_sim_env make_test_maps

The output is deterministic, so regenerating gives the same files.
"""

import argparse
import math
import os
import random

import numpy as np
from PIL import Image

RESOLUTION = 0.05
MARGIN = 0.5      # free border around the room in the image, m
WALL = 0.15       # wall thickness, m


def outer_walls(width, height):
    return [
        ('rect', -WALL, -WALL, width + WALL, 0.0),
        ('rect', -WALL, height, width + WALL, height + WALL),
        ('rect', -WALL, 0.0, 0.0, height),
        ('rect', width, 0.0, width + WALL, height),
    ]


def gap_vs_detour():
    # A thick block across the room with a long narrow passage on the straight
    # line and a wide opening at the top. Shortest squeezes through, Safest
    # goes round. The passage has to be long, a gap in a thin wall costs
    # too little to be worth a detour.
    width, height = 10.0, 7.0
    shapes = outer_walls(width, height) + [
        ('rect', 4.0, 0.0, 6.0, 3.15),
        ('rect', 4.0, 3.85, 6.0, 5.0),
    ]
    pairs = [
        ('through_or_around', [1.5, 3.5, 0.0], [8.5, 3.5, 0.0]),
        ('low_start', [1.5, 1.5, 0.0], [8.5, 1.5, 0.0]),
        ('back_and_turned', [8.5, 6.0, math.pi], [1.5, 2.5, math.pi / 2]),
    ]
    return width, height, shapes, pairs, (
        'A block across the room with a 2 m long, 0.7 m wide passage in the middle '
        'and a 2 m opening at the top. Shortest should take the passage, Safest '
        'the opening.')


def slalom():
    # Baffles from alternating sides force an S-shaped path. Smoothest should
    # cut fewer corners than Shortest.
    width, height = 12.0, 3.5
    shapes = outer_walls(width, height)
    for index, x in enumerate((3.0, 5.0, 7.0, 9.0)):
        if index % 2 == 0:
            shapes.append(('rect', x - 0.1, 0.0, x + 0.1, 2.2))
        else:
            shapes.append(('rect', x - 0.1, 1.3, x + 0.1, height))
    pairs = [
        ('full_slalom', [1.0, 1.75, 0.0], [11.0, 1.75, 0.0]),
        ('half_slalom', [1.0, 2.8, 0.0], [6.0, 0.7, 0.0]),
        ('reverse_sideways', [11.0, 0.8, math.pi / 2], [1.0, 2.7, math.pi / 2]),
    ]
    return width, height, shapes, pairs, (
        'A 12 m corridor with four baffles from alternating sides, 1.3 m '
        'openings. Shows how much each planner turns.')


def cluttered():
    # Pillars on a jittered grid. Many nearly equal routes, so the criteria
    # separate on clearance and turning.
    width, height = 8.0, 8.0
    shapes = outer_walls(width, height)
    rng = random.Random(7)
    for gx in range(4):
        for gy in range(4):
            if (gx, gy) in ((0, 0), (3, 3), (0, 3), (3, 0)):
                continue  # keep the corners free for starts and goals
            x = 1.2 + gx * 1.85 + rng.uniform(-0.35, 0.35)
            y = 1.2 + gy * 1.85 + rng.uniform(-0.35, 0.35)
            if rng.random() < 0.5:
                shapes.append(('circle', x, y, rng.uniform(0.15, 0.3)))
            else:
                size = rng.uniform(0.25, 0.5)
                shapes.append(('rect', x - size / 2, y - size / 2, x + size / 2, y + size / 2))
    pairs = [
        ('diagonal', [0.8, 0.8, math.pi / 4], [7.2, 7.2, math.pi / 4]),
        ('other_diagonal', [7.2, 0.8, 3 * math.pi / 4], [0.8, 7.2, 3 * math.pi / 4]),
        ('across', [0.8, 4.0, 0.0], [7.2, 4.0, 0.0]),
        ('up', [4.0, 0.7, math.pi / 2], [4.0, 7.3, math.pi / 2]),
    ]
    return width, height, shapes, pairs, (
        'An 8x8 m room with 12 pillars of different sizes on a jittered grid '
        '(fixed seed).')


def dead_end():
    # A U-shaped trap opening towards the start, goal behind it.
    width, height = 10.0, 8.0
    shapes = outer_walls(width, height) + [
        ('rect', 6.4, 2.5, 6.6, 5.5),   # back of the U
        ('rect', 4.0, 2.5, 6.6, 2.7),   # lower arm
        ('rect', 4.0, 5.3, 6.6, 5.5),   # upper arm
    ]
    pairs = [
        ('behind_the_trap', [2.0, 4.0, 0.0], [8.5, 4.0, 0.0]),
        ('into_the_trap', [8.5, 1.0, math.pi], [5.5, 4.0, 0.0]),
        ('past_the_trap', [2.0, 1.2, 0.0], [8.5, 6.8, 0.0]),
    ]
    return width, height, shapes, pairs, (
        'A U-shaped trap opening towards the start with the goal behind it. '
        'Checks that no planner gets stuck and how long the search takes.')


def open_room():
    # Nothing in the way. All planners should agree, so differences here are
    # bias, not criteria. Also tests holonomic motion with turned goals.
    width, height = 8.0, 8.0
    shapes = outer_walls(width, height)
    pairs = [
        ('straight', [1.5, 4.0, 0.0], [6.5, 4.0, 0.0]),
        ('diagonal', [1.5, 1.5, 0.0], [6.5, 6.5, 0.0]),
        ('sideways', [4.0, 1.5, 0.0], [4.0, 6.5, 0.0]),
        ('turn_around', [1.5, 6.5, 0.0], [6.5, 1.5, math.pi]),
    ]
    return width, height, shapes, pairs, (
        'An empty 8x8 m room, the baseline. All planners should give nearly '
        'the same path; the goals also test sideways and turned arrivals.')


MAPS = {
    'test_gap_vs_detour': gap_vs_detour,
    'test_slalom': slalom,
    'test_cluttered': cluttered,
    'test_dead_end': dead_end,
    'test_open_room': open_room,
}


def rasterize(width, height, shapes):
    """Occupancy grid (row 0 at the top) with cell centres inside a shape set."""
    origin = -MARGIN
    cols = int(round((width + 2 * MARGIN) / RESOLUTION))
    rows = int(round((height + 2 * MARGIN) / RESOLUTION))
    xs = origin + (np.arange(cols) + 0.5) * RESOLUTION
    ys = origin + (rows - np.arange(rows) - 0.5) * RESOLUTION
    x, y = np.meshgrid(xs, ys)
    occupied = np.zeros((rows, cols), dtype=bool)
    for shape in shapes:
        if shape[0] == 'rect':
            _, x0, y0, x1, y1 = shape
            occupied |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        else:
            _, cx, cy, radius = shape
            occupied |= (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2
    return occupied, origin


def write_map(name, occupied, origin, maps_dir):
    folder = os.path.join(maps_dir, name)
    os.makedirs(folder, exist_ok=True)
    pixels = np.where(occupied, 0, 254).astype(np.uint8)
    Image.fromarray(pixels, mode='L').save(os.path.join(folder, name + '.pgm'))
    with open(os.path.join(folder, name + '.yaml'), 'w') as stream:
        stream.write(
            f'# Generated by make_test_maps.py, do not edit by hand.\n'
            f'image: {name}.pgm\n'
            f'mode: trinary\n'
            f'resolution: {RESOLUTION}\n'
            f'origin: [{origin}, {origin}, 0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.196\n')


def write_scenarios(name, pairs, description, scenarios_dir):
    lines = [
        f'# Generated by make_test_maps.py, do not edit by hand.',
        f'# {description}',
        '# Poses are [x, y, yaw] in the map frame (m, rad).',
        f'map: {name}',
        'pairs:',
    ]
    for pair_name, start, goal in pairs:
        lines.append(f'  - name: {pair_name}')
        lines.append('    start: [' + ', '.join(f'{v:.3f}' for v in start) + ']')
        lines.append('    goal: [' + ', '.join(f'{v:.3f}' for v in goal) + ']')
    with open(os.path.join(scenarios_dir, name + '.yaml'), 'w') as stream:
        stream.write('\n'.join(lines) + '\n')


def check_poses(name, occupied, origin, pairs, clearance=0.4):
    """Fail loudly if a start or goal is closer than `clearance` to an obstacle."""
    rows = occupied.shape[0]
    cells = np.argwhere(occupied)
    centres = np.column_stack((origin + (cells[:, 1] + 0.5) * RESOLUTION,
                               origin + (rows - cells[:, 0] - 0.5) * RESOLUTION))
    for pair_name, start, goal in pairs:
        for label, pose in (('start', start), ('goal', goal)):
            distance = np.min(np.hypot(centres[:, 0] - pose[0], centres[:, 1] - pose[1]))
            if distance < clearance:
                raise SystemExit(f'{name}/{pair_name}: {label} is only {distance:.2f} m from a wall')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default='.', help='Package folder that holds maps/ and scenarios/')
    parser.add_argument('--only', nargs='+', choices=sorted(MAPS), help='Generate only these maps')
    args = parser.parse_args()

    for name in args.only or MAPS:
        width, height, shapes, pairs, description = MAPS[name]()
        occupied, origin = rasterize(width, height, shapes)
        check_poses(name, occupied, origin, pairs)
        write_map(name, occupied, origin, os.path.join(args.out, 'maps'))
        write_scenarios(name, pairs, description, os.path.join(args.out, 'scenarios'))
        print(f'{name}: {occupied.shape[1]}x{occupied.shape[0]} px, {len(pairs)} pairs')


if __name__ == '__main__':
    main()
