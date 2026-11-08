import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_map import FREE, RESOLUTION, WALL, export, rasterize  # noqa: E402


@pytest.fixture
def scene():
    return Scene.load(KIT / "samples" / "depot.scene.json")


def cell(scene, x, y):
    return int((scene.floor[1] / 2 - y) / RESOLUTION), int((x + scene.floor[0] / 2) / RESOLUTION)


def test_grid_matches_the_floor_and_places(scene):
    grid = rasterize(scene)
    assert grid.shape == (120, 200)
    for x, y in scene.waypoints.values():
        assert grid[cell(scene, x, y)] == FREE
    assert grid[cell(scene, -2.5, 1.2)] == WALL
    assert grid[cell(scene, 0.0, 2.45)] == WALL


def test_map_is_not_upside_down(scene):
    grid = rasterize(scene)
    assert grid[cell(scene, 0.0, 2.45)] == WALL and grid[cell(scene, 0.0, -2.45)] == FREE


def test_files_are_what_the_map_server_reads(scene, tmp_path):
    meta = yaml.safe_load(export(scene, str(tmp_path)).read_text(encoding="utf-8"))
    assert meta["origin"] == [-5.0, -3.0, 0.0] and meta["resolution"] == RESOLUTION
    image = np.array(Image.open(tmp_path / meta["image"]))
    assert image.shape == (120, 200) and set(np.unique(image)) == {WALL, FREE}
    places = yaml.safe_load((tmp_path / "depot_waypoints.yaml").read_text(encoding="utf-8"))
    assert places["dock"] == {"x": -4.3, "y": 0.0}
