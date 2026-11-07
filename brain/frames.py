"""rosbridge JSON frames that smabo-brain sends (WebSocket transport only).

The shared logic (odometry.py, vision.py …) returns plain values; turning them
into rosbridge-shaped JSON with a wall-clock stamp happens here, so that
smabo-brain-ros can reuse the logic with ROS time and typed messages instead.
"""

import json
import math
import time

from .odometry import build_pose_covariance, build_twist_covariance


def stamp() -> dict:
    """``builtin_interfaces/Time`` for now (wall clock)."""
    now = time.time()
    return {"sec": int(now), "nanosec": int((now % 1) * 1e9)}


def pub(topic: str, msg: dict) -> str:
    """A rosbridge ``publish`` frame."""
    return json.dumps({"op": "publish", "topic": topic, "msg": msg})


def notice(source: str, event: str, total: int) -> str:
    """Connection notice for smabo-web, e.g. ESP32 connected (total=1)."""
    return json.dumps({"op": "notice", "source": source, "event": event, "total": total})


def data_json(frame: dict) -> dict:
    """``msg.data`` of a std_msgs/String publish frame, parsed as a JSON object.

    rosbridge clients send the JSON as a string; some send the object itself.
    Anything else gives ``{}``.
    """
    raw = (frame.get("msg") or {}).get("data")
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except ValueError:
            return {}
    return raw if isinstance(raw, dict) else {}


def odom(r: dict) -> str:
    """``/odom`` (nav_msgs/Odometry) publish frame from ``Odometry.integrate`` values."""
    qz = math.sin(r["theta"] / 2.0)
    qw = math.cos(r["theta"] / 2.0)
    return pub("/odom", {
        "header": {"stamp": stamp(), "frame_id": r["odom_frame"]},
        "child_frame_id": r["base_frame"],
        "pose": {
            "pose": {
                "position":    {"x": r["x"], "y": r["y"], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
            "covariance": build_pose_covariance(r["cov"]),
        },
        "twist": {
            "twist": {
                "linear":  {"x": r["vx"], "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": r["wz"]},
            },
            "covariance": build_twist_covariance(r["cov"]),
        },
    })
