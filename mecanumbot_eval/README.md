# mecanumbot_eval

Part 3. Read a `drive_scenarios` folder and compare Shortest, Safest, Smoothest, Combined and NavFn on travel time and rolling energy.

The numbers from the five test maps are in [results.md](results.md).

## Energy

Wheel centres come from `mecanumbot_sim_env/models/mecanumbot/model.sdf`: x = ±0.064 m, y = ±0.1522 m, radius 0.033 m, mass 3.0 kg.

Odometry `vx`, `vy`, `wz` (robot frame) gives the four wheel speeds

```
w = (vx ± vy ± (lx + ly) * wz) / r
```

with `lx + ly = 0.2162` m. Swapping the roller direction only swaps the wheels. The sum of absolute speeds does not change.

Rolling resistance is `c_rr = 0.02` on a hard floor, split across the four wheels:

```
E = c_rr * m * g * (sum of wheel arc lengths) / 4
```

Gazebo sets the body velocity directly. The log has no motor current, and every planner uses this same rolling work. Forward and sideways motion at the same speed cost the same. On a pure translation the energy follows the distance. Spinning costs energy while the robot stays in place.

## Score

On each start/goal pair, time and energy are divided by the best planner that actually finished that pair.

```
score = alpha * time_ratio + (1 - alpha) * energy_ratio
```

`alpha = 0` is energy, `alpha = 1` is travel time, `alpha = 0.5` is the default. Missed pairs are counted first, then the mean score. Equal scores follow the planner list above, Shortest first.

Pass a `plan_scenarios` folder to copy planning time, turning and costmap cost into the table. The score itself is travel time and energy from the drive.

## Run

`drive_scenarios` has to have finished first. From the repo, with this folder on `PYTHONPATH`, or after `colcon build` inside the dev container:

```bash
python3 -m mecanumbot_eval.compare_runs results/drive_test_slalom_20261031_120000
```

```bash
ros2 run mecanumbot_eval compare_runs results/drive_test_slalom_20261031_120000 \
  --plans results/test_slalom_20261031_115500
```

`--alpha 0.25` puts more weight on energy. Output lands in `<drive_dir>/eval/`:

| file | contents |
| --- | --- |
| `metrics.csv` | one row per drive: time, energy, wheel travel, lengths, and plan columns when `--plans` is set |
| `scores.csv` | time ratio, energy ratio, score, rank, for the chosen alpha |
| `recommendation.txt` | energy / balanced / time winners, and the mean table |
