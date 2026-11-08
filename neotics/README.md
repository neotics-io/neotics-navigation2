# Neotics kit for Nav2

This folder is Neotics' own work, added to our copy of [Nav2](https://github.com/ros-navigation/navigation2). It turns any Neotics scene into a Nav2 map with its named places.

It is one part of the Neotics stack. Every fork in [neotics-io](https://github.com/neotics-io) reads the same two files:

- a **Neotics Scene** (`samples/depot.scene.json`): the room, the obstacles, the arm and the named places;
- a **Neotics Episode** (`samples/depot.episode.json`): a run recorded in MuJoCo, with time, joint positions and commands.

The depot is a sample made for these tools. It is not an entry or an answer for any [Neotics challenge](https://lab.neotics.io).

## Use it

Run from this folder:

```bash
python scene_to_map.py samples/depot.scene.json out
```

## How it's checked

Loads the exported map in Nav2 on ROS 2 Jazzy and plans a real route from the dock to the drop-off, then confirms the route touches no obstacle. This runs on GitHub in `.github/workflows/neotics.yml`, on every change to this folder.

## Licence

See [`LICENSE`](LICENSE). It covers this folder only.
