"""Generate a scenario file with well-spread start/goal points on a map.

Points are picked in free space at least --clearance metres from any occupied or
unknown cell, inside the largest connected free region, and as far from each
other as possible (farthest-point sampling, deterministic). Every pair of
points becomes one start/goal pair.

    ros2 run mecanumbot_sim_env make_scenarios maps/LED_exp/LED_exp.yaml --points 5 > scenarios/LED_exp.yaml
"""

import argparse
from collections import deque
import itertools
import os

import numpy as np
from PIL import Image
import yaml


def load_free_mask(map_yaml):
    """True where the map is free, following map_server's trinary thresholds."""
    with open(map_yaml) as stream:
        meta = yaml.safe_load(stream)
    image = Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])).convert('L')
    pixels = np.asarray(image, dtype=float) / 255.0
    occupancy = pixels if meta.get('negate', 0) else 1.0 - pixels
    free = occupancy < meta['free_thresh']
    return free, meta


def erode(mask, steps):
    """Shrink the free area by `steps` pixels (8-connected)."""
    for _ in range(steps):
        padded = np.pad(mask, 1, constant_values=False)
        shrunk = mask.copy()
        for dy, dx in itertools.product((-1, 0, 1), repeat=2):
            shrunk &= padded[1 + dy:1 + dy + mask.shape[0], 1 + dx:1 + dx + mask.shape[1]]
        mask = shrunk
    return mask


def largest_region(mask):
    """Keep only the largest 4-connected region of the mask."""
    labels = np.zeros(mask.shape, dtype=int)
    best_label, best_size, label = 0, 0, 0
    for start in zip(*np.nonzero(mask)):
        if labels[start]:
            continue
        label += 1
        labels[start] = label
        queue, size = deque([start]), 0
        while queue:
            row, col = queue.popleft()
            size += 1
            for nrow, ncol in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
                if (0 <= nrow < mask.shape[0] and 0 <= ncol < mask.shape[1]
                        and mask[nrow, ncol] and not labels[nrow, ncol]):
                    labels[nrow, ncol] = label
                    queue.append((nrow, ncol))
        if size > best_size:
            best_label, best_size = label, size
    return labels == best_label


def spread_points(mask, count):
    """Farthest-point sampling, starting from the cell closest to the region's centroid."""
    cells = np.argwhere(mask)
    if len(cells) < count:
        raise SystemExit(f'only {len(cells)} usable cells; lower --clearance or --points')
    centroid = cells.mean(axis=0)
    chosen = [int(np.argmin(((cells - centroid) ** 2).sum(axis=1)))]
    distance = ((cells - cells[chosen[0]]) ** 2).sum(axis=1)
    while len(chosen) < count:
        chosen.append(int(np.argmax(distance)))
        distance = np.minimum(distance, ((cells - cells[chosen[-1]]) ** 2).sum(axis=1))
    return [tuple(cells[i]) for i in chosen]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('map_yaml')
    parser.add_argument('--points', type=int, default=5)
    parser.add_argument('--clearance', type=float, default=0.5,
                        help='Minimum distance from occupied/unknown cells in metres')
    args = parser.parse_args()

    free, meta = load_free_mask(args.map_yaml)
    resolution = meta['resolution']
    origin_x, origin_y = meta['origin'][:2]
    usable = largest_region(erode(free, int(np.ceil(args.clearance / resolution))))
    height = free.shape[0]

    points = []
    for row, col in spread_points(usable, args.points):
        # Cell centre; image row 0 is the top of the map.
        points.append((origin_x + (col + 0.5) * resolution,
                       origin_y + (height - row - 0.5) * resolution))

    name = os.path.splitext(os.path.basename(args.map_yaml))[0]
    print(f'# Start/goal pairs on the {name} map. Points were generated with make_scenarios')
    print(f'# (clearance {args.clearance} m, largest free region); every pair is used.')
    print('# Poses are [x, y, yaw] in the map frame (m, rad).')
    print(f'map: {name}')
    print('waypoints:')
    for index, (x, y) in enumerate(points, 1):
        print(f'  wp{index}: [{x:.3f}, {y:.3f}]')
    print('pairs:')
    for (i, a), (j, b) in itertools.combinations(enumerate(points, 1), 2):
        print(f'  - name: wp{i}_to_wp{j}')
        print(f'    start: [{a[0]:.3f}, {a[1]:.3f}, 0.0]')
        print(f'    goal: [{b[0]:.3f}, {b[1]:.3f}, 0.0]')


if __name__ == '__main__':
    main()
