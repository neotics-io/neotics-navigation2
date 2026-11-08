"""Turn a Neotics Scene into a Nav2 map.    python scene_to_map.py samples/depot.scene.json out_dir

Writes <name>.pgm and <name>.yaml, the two files Nav2's map server loads, and <name>_waypoints.yaml with the
scene's named places in the map frame.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from neotics_scene import Scene

RESOLUTION = 0.05
FREE, WALL = 254, 0


def rasterize(scene: Scene, resolution: float = RESOLUTION) -> np.ndarray:
    """Row 0 is the top of the image, the far (+y) edge of the floor, as map files expect."""
    width, height = round(scene.floor[0] / resolution), round(scene.floor[1] / resolution)
    grid = np.full((height, width), FREE, dtype=np.uint8)
    for row in range(height):
        y = scene.floor[1] / 2 - (row + 0.5) * resolution
        for col in range(width):
            if scene.occupied(-scene.floor[0] / 2 + (col + 0.5) * resolution, y):
                grid[row, col] = WALL
    return grid


def export(scene: Scene, out_dir: str, resolution: float = RESOLUTION) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rasterize(scene, resolution), mode="L").save(out / f"{scene.name}.pgm")
    meta = {
        "image": f"{scene.name}.pgm",
        "mode": "trinary",
        "resolution": resolution,
        "origin": [-scene.floor[0] / 2, -scene.floor[1] / 2, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    (out / f"{scene.name}.yaml").write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
    places = {name: {"x": x, "y": y} for name, (x, y) in scene.waypoints.items()}
    (out / f"{scene.name}_waypoints.yaml").write_text(yaml.safe_dump(places, sort_keys=False), encoding="utf-8")
    return out / f"{scene.name}.yaml"


if __name__ == "__main__":
    print(export(Scene.load(sys.argv[1]), sys.argv[2]))
