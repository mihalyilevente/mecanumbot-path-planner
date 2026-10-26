import pytest

from mecanumbot_sim_env.make_test_maps import MAPS, RESOLUTION, check_poses, rasterize


def test_rectangle_fills_the_right_cells():
    occupied, origin = rasterize(2.0, 1.0, [('rect', 0.0, 0.0, 0.5, 0.5)])
    assert origin == -0.5
    # 0.5 m square at 0.05 m resolution is 10 x 10 cells
    assert occupied.sum() == 100


def test_every_map_is_closed_and_its_poses_are_clear():
    for name, build in MAPS.items():
        width, height, shapes, pairs, _ = build()
        occupied, origin = rasterize(width, height, shapes)
        check_poses(name, occupied, origin, pairs)
        # outer wall all the way round: first and last wall row/column inside the margin
        margin = int(round(-origin / RESOLUTION))
        assert occupied[margin - 1, margin:-margin].all(), name
        assert occupied[-margin, margin:-margin].all(), name


def test_pose_inside_a_wall_is_rejected():
    occupied, origin = rasterize(4.0, 4.0, [('rect', 1.0, 1.0, 3.0, 3.0)])
    with pytest.raises(SystemExit):
        check_poses('box', occupied, origin, [('bad', [2.0, 2.0, 0.0], [0.5, 0.5, 0.0])])
