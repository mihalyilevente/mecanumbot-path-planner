import math

from mecanumbot_planner_ui.pose_math import (
    downsample_path,
    grid_to_png_bytes,
    path_length,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def test_yaw_roundtrip():
    for yaw in (0.0, 1.0, -1.2, math.pi):
        _, _, z, w = yaw_to_quaternion(yaw)
        assert abs(quaternion_to_yaw(z, w) - yaw) < 1e-9


def test_path_length_and_downsample():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert abs(path_length(points) - 2.0) < 1e-9
    long_path = [(float(i), 0.0) for i in range(500)]
    thin = downsample_path(long_path, max_points=50)
    assert len(thin) <= 50
    assert thin[0] == (0.0, 0.0)
    assert thin[-1] == (499.0, 0.0)


def test_grid_to_png_bytes():
    data = [0, 100, -1, 0]
    png = grid_to_png_bytes(2, 2, data)
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
