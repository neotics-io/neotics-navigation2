"""Full check inside ROS 2: Nav2 loads the exported map and plans a real route through the scene."""
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_map import export  # noqa: E402


def params(map_yaml: Path) -> dict:
    return {
        "map_server": {"ros__parameters": {"yaml_filename": str(map_yaml)}},
        "planner_server": {"ros__parameters": {
            "expected_planner_frequency": 5.0,
            "planner_plugins": ["GridBased"],
            "GridBased": {"plugin": "nav2_navfn_planner::NavfnPlanner", "tolerance": 0.2, "use_astar": False, "allow_unknown": False},
        }},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "update_frequency": 1.0, "publish_frequency": 1.0, "global_frame": "map", "robot_base_frame": "base_link",
            "robot_radius": 0.22, "resolution": 0.05, "track_unknown_space": False, "always_send_full_costmap": True,
            "plugins": ["static_layer", "inflation_layer"],
            "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True},
            "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "cost_scaling_factor": 3.0, "inflation_radius": 0.35},
        }}},
        "lifecycle_manager": {"ros__parameters": {"autostart": True, "node_names": ["map_server", "planner_server"], "bond_timeout": 0.0}},
    }


def pose(x: float, y: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation.w = 1.0
    return msg


def wait_for_walls(node: Node, seconds: float = 120.0) -> bool:
    """The planner answers before its costmap holds the map, and an empty costmap gives a straight line through everything."""
    seen = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda grid: seen.append(100 in grid.data), latched)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not any(seen):
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_subscription(subscription)
    return any(seen)


def main() -> int:
    scene = Scene.load(KIT / "samples" / "depot.scene.json")
    work = Path(tempfile.mkdtemp())
    params_file = work / "params.yaml"
    params_file.write_text(yaml.safe_dump(params(export(scene, str(work)))), encoding="utf-8")
    dock = scene.waypoints["dock"]

    args = ["--ros-args", "--params-file", str(params_file)]
    processes = [subprocess.Popen(cmd) for cmd in (
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--x", str(dock[0]), "--y", str(dock[1]), "--frame-id", "map", "--child-frame-id", "base_link"],
        ["ros2", "run", "nav2_map_server", "map_server", *args],
        ["ros2", "run", "nav2_planner", "planner_server", *args],
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-r", "__node:=lifecycle_manager", "--params-file", str(params_file)],
    )]
    try:
        rclpy.init()
        node = Node("neotics_ci")
        if not wait_for_walls(node):
            print("FAIL: the costmap never received the map")
            return 1
        client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        goal = ComputePathToPose.Goal(start=pose(*dock), goal=pose(*scene.waypoints["dropoff"]), use_start=True)
        deadline = time.monotonic() + 180
        path = None
        while time.monotonic() < deadline and path is None:
            if not client.wait_for_server(timeout_sec=2.0):
                continue
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            handle = future.result()
            if handle is None or not handle.accepted:
                time.sleep(2)
                continue
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result, timeout_sec=30)
            poses = result.result().result.path.poses if result.result() else []
            if poses:
                path = poses
            else:
                time.sleep(2)  # the planner refuses goals until the costmap has the map
        if not path:
            print("FAIL: Nav2 never returned a route")
            return 1
        blocked = [p for p in path if scene.occupied(p.pose.position.x, p.pose.position.y)]
        end = path[-1].pose.position
        target = scene.waypoints["dropoff"]
        print(f"route: {len(path)} poses, end ({end.x:.2f}, {end.y:.2f}), through obstacles: {len(blocked)}")
        return 0 if not blocked and abs(end.x - target[0]) < 0.3 and abs(end.y - target[1]) < 0.3 else 1
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        time.sleep(1)
        for process in processes:
            process.kill()


if __name__ == "__main__":
    sys.exit(main())
