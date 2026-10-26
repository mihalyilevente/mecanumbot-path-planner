"""Draw planned paths on top of a map image."""

import os

from PIL import Image, ImageDraw
import yaml

COLORS = {
    'Shortest': (31, 119, 180),
    'Safest': (44, 160, 44),
    'Smoothest': (214, 39, 40),
    'Combined': (148, 103, 189),
    'NavFn': (255, 127, 14),
}
FALLBACK_COLOR = (90, 90, 90)
SCALE = 3


def draw(map_yaml, pair, paths, out_file, thin_paths=None, planners=None):
    """Save a PNG of the map with every planner's path for one start/goal pair.

    thin_paths, if given, are drawn first as thin lighter lines (for example the
    planned paths under the driven ones). planners lists what was requested,
    so the legend can mark the ones without a path as failed.
    """
    with open(map_yaml) as stream:
        meta = yaml.safe_load(stream)
    image = Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])).convert('RGB')
    height = image.height
    resolution = meta['resolution']
    origin_x, origin_y = meta['origin'][:2]
    image = image.resize((image.width * SCALE, image.height * SCALE), Image.NEAREST)
    canvas = ImageDraw.Draw(image)

    def to_pixel(x, y):
        # Image row 0 is the top of the map, i.e. the largest y.
        return ((x - origin_x) / resolution * SCALE,
                (height - (y - origin_y) / resolution) * SCALE)

    for planner, points in (thin_paths or {}).items():
        color = COLORS.get(planner, FALLBACK_COLOR)
        light = tuple(channel + (255 - channel) // 2 for channel in color)
        canvas.line([to_pixel(x, y) for x, y in points], fill=light, width=1)

    for planner, points in paths.items():
        color = COLORS.get(planner, FALLBACK_COLOR)
        canvas.line([to_pixel(x, y) for x, y in points], fill=color, width=2)

    for key, color in (('start', (0, 0, 0)), ('goal', (220, 0, 0))):
        px, py = to_pixel(*pair[key][:2])
        canvas.ellipse((px - 5, py - 5, px + 5, py + 5), outline=color, width=2)

    # Legend in a strip above the map, so it never hides part of it.
    planners = list(planners or COLORS)
    strip = 24
    framed = Image.new('RGB', (image.width, image.height + strip), (255, 255, 255))
    framed.paste(image, (0, strip))
    canvas = ImageDraw.Draw(framed)
    x = 6
    canvas.text((x, 6), pair['name'], fill=(0, 0, 0))
    x += 8 * len(pair['name']) + 16
    for planner in planners:
        label = planner if planner in paths else planner + ' (failed)'
        canvas.text((x, 6), label, fill=COLORS.get(planner, FALLBACK_COLOR))
        x += 7 * len(label) + 14

    framed.save(out_file)
