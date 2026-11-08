"""Pose and path helpers with no ROS dependency."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for a planar yaw."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(z: float, w: float) -> float:
    """Yaw from a planar quaternion (z, w components)."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def path_length(points: Sequence[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def downsample_path(points: Sequence[Point], max_points: int = 200) -> List[Point]:
    """Keep endpoints; thin intermediate points for JSON/canvas."""
    if len(points) <= max_points or max_points < 2:
        return [(float(x), float(y)) for x, y in points]
    step = (len(points) - 1) / (max_points - 1)
    indices = [round(i * step) for i in range(max_points)]
    indices[-1] = len(points) - 1
    # Preserve order and drop duplicates from rounding.
    out: List[Point] = []
    seen = set()
    for index in indices:
        if index not in seen:
            seen.add(index)
            x, y = points[index]
            out.append((float(x), float(y)))
    return out


def occupancy_to_rgba(value: int) -> Tuple[int, int, int, int]:
    """Map OccupancyGrid cell to RGBA (free white, occupied black, unknown grey-green)."""
    if value < 0:
        return (140, 160, 140, 255)
    if value >= 50:
        return (20, 20, 20, 255)
    # 0..49 free / light
    shade = 255 - int(value * 2)
    return (shade, shade, shade, 255)


def grid_to_png_bytes(width: int, height: int, data: Iterable[int]) -> bytes:
    """Encode an OccupancyGrid as a top-left PNG (row 0 = map top = high y)."""
    from io import BytesIO

    from PIL import Image

    values = list(data)
    if len(values) != width * height:
        raise ValueError('Occupancy data length does not match width*height')
    # nav_msgs OccupancyGrid: row-major, row 0 is at origin (bottom in map frame).
    # Flip vertically so canvas y grows downward matching image rows.
    pixels = []
    for row in range(height - 1, -1, -1):
        start = row * width
        for value in values[start:start + width]:
            pixels.append(occupancy_to_rgba(int(value)))
    image = Image.new('RGBA', (width, height))
    image.putdata(pixels)
    buffer = BytesIO()
    image.save(buffer, format='PNG')
    return buffer.getvalue()
