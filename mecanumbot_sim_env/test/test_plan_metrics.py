import math

from mecanumbot_sim_env.plan_scenarios import path_length, total_turn_deg


def test_length_of_l_shape():
    assert math.isclose(path_length([(0, 0), (1, 0), (1, 2)]), 3.0)


def test_straight_line_has_no_turn():
    assert total_turn_deg([(0, 0), (1, 0), (2, 0), (3, 0)]) == 0.0


def test_right_angle_and_back():
    points = [(0, 0), (1, 0), (1, 1), (2, 1)]
    assert math.isclose(total_turn_deg(points), 180.0)


def test_repeated_points_are_ignored():
    assert math.isclose(total_turn_deg([(0, 0), (1, 0), (1, 0), (1, 1)]), 90.0)
