# Maps

| Map | Size | Resolution | Source |
| --- | --- | --- | --- |
| `AI_dept` | 279×325 px, about 19.5×22.8 m | 0.07 m | Lab corridor map, recorded on the real robot |
| `LED_exp` | 208×186 px, about 10.4×9.3 m | 0.05 m | Experiment room, recorded on the real robot |

Both are copied unchanged from
[Fortuz/mecanumbot](https://github.com/Fortuz/mecanumbot) at commit
`5818369f8571942a3b21b3ed4861d5dae3752ab0`
(`mecanumbot_description/maps/`), licensed under Apache 2.0.
The keep-out mask for `LED_exp` is left out on purpose; the simulation
configuration does not use costmap filters.

## Generated test maps

| Map | Size | Resolution |
| --- | --- | --- |
| `test_gap_vs_detour` | 10×7 m room | 0.05 m |
| `test_slalom` | 12×3.5 m corridor | 0.05 m |
| `test_cluttered` | 8×8 m room | 0.05 m |
| `test_dead_end` | 10×8 m room | 0.05 m |
| `test_open_room` | 8×8 m room | 0.05 m |

These are made by `../mecanumbot_sim_env/make_test_maps.py`; edit the layout
there and regenerate instead of editing the images. The room's bottom-left
inner corner is at (0, 0) in the map frame, with a 0.5 m border around it.

## Scenarios

Start/goal pairs for each map are in `../scenarios/`. For `AI_dept` they are
built from the upstream waypoint file. The upstream `LED_exp` waypoints don't
fit the map, so its points were generated with `make_scenarios`. The test maps
have hand-placed pairs written by `make_test_maps.py`.
