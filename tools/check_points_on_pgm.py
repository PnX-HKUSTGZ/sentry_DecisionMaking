#!/usr/bin/env python3
"""Validate decision points against a Nav2 PGM map."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image
import yaml


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--map-yaml", required=True, help="Nav2 map yaml path")
    p.add_argument("--points-yaml", required=True, help="Decision points yaml path")
    p.add_argument("--search-radius-m", type=float, default=4.0, help="Obstacle clearance search radius")
    return p.parse_args()


def world_to_pixel(x: float, y: float, *, origin_x: float, origin_y: float, resolution: float, height: int) -> tuple[int, int]:
    px = int((x - origin_x) // resolution)
    py_bottom = int((y - origin_y) // resolution)
    py = height - 1 - py_bottom
    return px, py


def clearance_m(img, px: int, py: int, resolution: float, search_radius_px: int) -> float | None:
    width, height = img.size
    best = None
    pix = img.load()
    for yy in range(max(0, py - search_radius_px), min(height, py + search_radius_px + 1)):
        for xx in range(max(0, px - search_radius_px), min(width, px + search_radius_px + 1)):
            if pix[xx, yy] != 0:
                continue
            dist = math.hypot(xx - px, yy - py) * resolution
            if best is None or dist < best:
                best = dist
    return best


def main() -> int:
    args = parse_args()
    with Path(args.map_yaml).open("r", encoding="utf-8") as f:
        map_cfg = yaml.safe_load(f)
    with Path(args.points_yaml).open("r", encoding="utf-8") as f:
        points_cfg = yaml.safe_load(f)

    image_path = Path(args.map_yaml).parent / map_cfg["image"]
    image = Image.open(image_path)
    pixels = image.load()

    resolution = float(map_cfg["resolution"])
    origin_x, origin_y, _ = map_cfg["origin"]
    search_radius_px = max(1, int(args.search_radius_m / resolution))

    print(f"map={image_path} size={image.size} resolution={resolution} origin=({origin_x}, {origin_y})")
    print(f"points={args.points_yaml}")
    print("key\tworld_xy\tpixel\tcell\tclearance_m")

    failed = False
    for key, value in points_cfg["points"].items():
        x = float(value[0])
        y = float(value[1])
        px, py = world_to_pixel(
            x,
            y,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
            height=image.height,
        )
        inside = 0 <= px < image.width and 0 <= py < image.height
        cell = None if not inside else pixels[px, py]
        clearance = None if not inside else clearance_m(image, px, py, resolution, search_radius_px)
        print(
            f"{key}\t({x:.3f}, {y:.3f})\t({px}, {py})\t"
            f"{'out' if cell is None else cell}\t"
            f"{'n/a' if clearance is None else f'{clearance:.3f}'}"
        )
        if cell != 254:
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
