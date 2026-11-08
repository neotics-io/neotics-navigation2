"""Neotics Scene: one JSON file that describes a robot cell for every tool in the Neotics stack.

The same file becomes a MuJoCo model, a Gazebo world, a USD stage, a ROS robot, a Nav2 map and an Open-RMF
traffic graph. Each fork's `neotics` folder holds the exporter for that tool.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

SCHEMA = 1
AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


class SceneError(ValueError):
    """The scene or episode file breaks a rule of the format."""


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]  # full edge lengths, metres


@dataclass(frozen=True)
class Item(Box):
    mass: float


@dataclass(frozen=True)
class Link:
    joint: str
    axis: tuple[float, float, float]
    length: float
    radius: float
    mass: float
    lower: float
    upper: float

    @property
    def inertia(self) -> tuple[float, float, float]:
        """Solid cylinder about its centre: (ixx, iyy, izz)."""
        side = self.mass * (3 * self.radius**2 + self.length**2) / 12
        return side, side, self.mass * self.radius**2 / 2


class Scene:
    def __init__(self, data: dict):
        if data.get("neotics_scene") != SCHEMA:
            raise SceneError(f"expected neotics_scene: {SCHEMA}")
        self.name: str = data["name"]
        self.floor: tuple[float, float] = tuple(data["floor"]["size"])
        self.friction: float = data["floor"].get("friction", 0.9)
        self._walls = data.get("walls")
        self.obstacles = [
            Box(o["name"], (o["center"][0], o["center"][1], o["height"] / 2), (o["size"][0], o["size"][1], o["height"]))
            for o in data.get("obstacles", [])
        ]
        self.items = [Item(i["name"], tuple(i["center"]), tuple(i["size"]), i["mass"]) for i in data.get("items", [])]
        self.waypoints: dict[str, tuple[float, float]] = {w["name"]: tuple(w["at"]) for w in data.get("waypoints", [])}
        self.lanes: list[tuple[str, str]] = [tuple(l) for l in data.get("lanes", [])]
        robot = data["robot"]
        self.robot_name: str = robot["name"]
        self.base: tuple[float, float, float] = tuple(robot["base"])
        self.links = [
            Link(l["joint"], AXES[l["axis"]], l["length"], l["radius"], l["mass"], l["range"][0], l["range"][1])
            for l in robot["links"]
        ]
        self.home: list[float] = list(robot["home"])
        self.reach: list[float] = list(robot["reach"])
        self._check()

    @classmethod
    def load(cls, path: str | Path) -> "Scene":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    @property
    def joint_names(self) -> list[str]:
        return [l.joint for l in self.links]

    @property
    def walls(self) -> list[Box]:
        if not self._walls:
            return []
        lx, ly = self.floor
        t, h = self._walls["thickness"], self._walls["height"]
        z = h / 2
        return [
            Box("wall_north", (0.0, ly / 2 - t / 2, z), (lx, t, h)),
            Box("wall_south", (0.0, -ly / 2 + t / 2, z), (lx, t, h)),
            Box("wall_east", (lx / 2 - t / 2, 0.0, z), (t, ly - 2 * t, h)),
            Box("wall_west", (-lx / 2 + t / 2, 0.0, z), (t, ly - 2 * t, h)),
        ]

    @property
    def boxes(self) -> list[Box]:
        """Everything fixed to the floor: walls, then obstacles."""
        return self.walls + self.obstacles

    def occupied(self, x: float, y: float, margin: float = 0.0) -> bool:
        return any(
            abs(x - b.center[0]) <= b.size[0] / 2 + margin and abs(y - b.center[1]) <= b.size[1] / 2 + margin
            for b in self.boxes
        )

    def _check(self) -> None:
        names = [b.name for b in self.boxes] + [i.name for i in self.items] + self.joint_names
        repeated = {n for n in names if names.count(n) > 1}
        if repeated:
            raise SceneError(f"names used twice: {sorted(repeated)}")
        for b in self.boxes + self.items:
            if min(b.size) <= 0:
                raise SceneError(f"{b.name}: sizes must be positive")
        for name, (x, y) in self.waypoints.items():
            if abs(x) > self.floor[0] / 2 or abs(y) > self.floor[1] / 2:
                raise SceneError(f"waypoint {name} is off the floor")
            if self.occupied(x, y):
                raise SceneError(f"waypoint {name} is inside an obstacle")
        for a, b in self.lanes:
            if a not in self.waypoints or b not in self.waypoints:
                raise SceneError(f"lane {a}-{b} uses an unknown waypoint")
        for label, pose in (("home", self.home), ("reach", self.reach)):
            if len(pose) != len(self.links):
                raise SceneError(f"{label} needs one value per joint")
            for q, l in zip(pose, self.links):
                if not l.lower <= q <= l.upper:
                    raise SceneError(f"{label}: {l.joint} is outside its range")


def load_episode(path: str | Path) -> dict:
    """A recorded run: time, measured joint positions (qpos) and commanded positions (ctrl), one row per frame."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("neotics_episode") != SCHEMA:
        raise SceneError(f"expected neotics_episode: {SCHEMA}")
    n, width = len(data["time"]), len(data["joints"])
    for key in ("qpos", "ctrl"):
        if len(data[key]) != n or any(len(row) != width for row in data[key]):
            raise SceneError(f"{key} must have {n} rows of {width} values")
    return data
