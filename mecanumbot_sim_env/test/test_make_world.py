import numpy as np

from mecanumbot_sim_env.make_world import merge_rectangles


def covered(shape, rectangles):
    mask = np.zeros(shape, dtype=bool)
    for row, col, rows, cols in rectangles:
        assert not mask[row:row + rows, col:col + cols].any(), 'rectangles overlap'
        mask[row:row + rows, col:col + cols] = True
    return mask


def test_solid_block_is_one_box():
    mask = np.zeros((6, 6), dtype=bool)
    mask[1:4, 2:5] = True
    assert merge_rectangles(mask) == [(1, 2, 3, 3)]


def test_l_shape_is_covered_exactly():
    mask = np.zeros((5, 5), dtype=bool)
    mask[0:5, 0] = True
    mask[4, 0:5] = True
    rectangles = merge_rectangles(mask)
    assert (covered(mask.shape, rectangles) == mask).all()


def test_random_map_is_covered_exactly():
    mask = np.random.default_rng(0).random((40, 30)) > 0.7
    assert (covered(mask.shape, merge_rectangles(mask)) == mask).all()
