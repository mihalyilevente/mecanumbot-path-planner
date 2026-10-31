# Drive comparison

`drive_scenarios` on the five test maps, `rviz:=false`. Success means the goal was reached before 180 s. Energy is rolling work from the odometry twists (`c_rr = 0.02`, mass 3.0 kg). At alpha 0.5 the score weights travel time and energy equally. A planner that missed a pair is placed after one that finished every pair on that map.

74 of 85 drives reached the goal.

| Map | Reached | Winner at alpha 0.5 | Mean time | Mean energy |
| --- | --- | --- | --- | --- |
| test_gap_vs_detour | 15/15 | Safest | 42.7 s | 4.70 J |
| test_slalom | 10/15 | NavFn | 59.2 s | 6.49 J |
| test_cluttered | 18/20 | NavFn | 34.9 s | 4.41 J |
| test_dead_end | 11/15 | NavFn | 44.7 s | 5.35 J |
| test_open_room | 20/20 | Shortest | 28.5 s | 3.61 J |

NavFn finished every pair on the slalom, the cluttered room and the dead end. The custom planners did not.

On test_open_room all five finished. Mean times are 28.5 s to 30.2 s. Smoothest had the lowest energy, 3.54 J, and the highest time, so alpha 0.5 still picks Shortest.

On `through_or_around`, Shortest took 28.7 s and 3.95 J. Safest took the longer way, 44.1 s and 4.32 J. On `low_start`, Shortest took 176 s and 6.20 J, Safest 40.6 s and 4.55 J. That second run is why Safest wins the gap map.

Runs that hit 180 s:

- `full_slalom`: Shortest
- `reverse_sideways`: Shortest, Safest, Smoothest, Combined. NavFn arrived in 73.6 s
- cluttered `diagonal`: Smoothest
- cluttered `other_diagonal`: Shortest
- `into_the_trap`: Shortest, Safest, Smoothest. Combined and NavFn arrived
- `past_the_trap`: Shortest

On the static harness, where all five plans succeeded, NavFn took about 1 ms and Combined often 100–300 ms. A few first requests failed with "AStarPlanner is not configured and active". Those were startup misses and are not counted above.
