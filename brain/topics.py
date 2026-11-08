"""The smabo WebSocket protocol: who publishes / subscribes which topic.

Every client (smabo-web, smabo-app, smabo-esp32 and smabo-sim, which plays
both the ESP32 and the phone) talks to the brain the same way, whichever brain
it is — smabo-brain or smabo-brain-ros (rosbridge_suite):

  1. connect to ``ws://<brain>:9090/``
  2. ``advertise`` every topic it publishes, ``subscribe`` every topic it reads
  3. ``publish`` with its source prefix (``/web``, ``/app``, ``/esp32``)
  4. ``publish`` ``/hello`` once: ``{"protocol": N, "client": name, "version": x.y.z}``.
     The brain answers on ``/brain/info`` (``{"protocol": N, "brain": name,
     "version": x.y.z}``, also repeated every 5 s) and each side warns when the
     protocol numbers differ.

The brain strips the prefix and delivers the message under the canonical name.
smabo-brain does that inline (brain/server.py); smabo-brain-ros, which wraps
smabo-brain, imports these tables and runs one ``topic_tools relay`` per entry
of :data:`PUBLISH` (launch/relays.launch.py).

This module is the single source of truth for the tables. Clients written in
other languages (web: TypeScript, app: Dart, ESP32: MicroPython) keep their own
copy, and ``tests/test_protocol.py`` checks each copy against this one.
"""

# Version of these tables and the meaning of the messages on them. Bump it when
# a topic is renamed or removed, or a message changes type or meaning; clients
# with a different number are warned on connect (see /hello and /brain/info).
PROTOCOL_VERSION = 1

# Source prefix each client puts in front of the topics it publishes.
SOURCE_PREFIXES: dict[str, str] = {
    "web":   "/web",
    "app":   "/app",
    "esp32": "/esp32",
}

# Canonical topics each client publishes (sent as <prefix><topic>), with types.
PUBLISH: dict[str, dict[str, str]] = {
    "web": {
        "/cmd_vel":           "geometry_msgs/msg/Twist",
        "/servo/command":     "trajectory_msgs/msg/JointTrajectory",
        "/initialpose":       "geometry_msgs/msg/PoseWithCovarianceStamped",
        "/goal_pose":         "geometry_msgs/msg/PoseStamped",
        "/nav/command":       "std_msgs/msg/String",   # nav_manager: SLAM / save map / navigate (JSON)
        "/speech/say":        "std_msgs/msg/String",
        "/expression":        "std_msgs/msg/String",   # expression name (smabo-app)
        "/look_at":           "geometry_msgs/msg/PoseStamped",
        "/ping":              "std_msgs/msg/String",
        "/vision/config":     "std_msgs/msg/String",
        "/webrtc/preview":    "std_msgs/msg/String",
        "/webrtc/web_answer": "std_msgs/msg/String",
        "/webrtc/web_ice":    "std_msgs/msg/String",
        "/hello":             "std_msgs/msg/String",   # protocol / version, once on connect
    },
    "app": {
        "/imu/data":          "sensor_msgs/msg/Imu",
        "/gps/fix":           "sensor_msgs/msg/NavSatFix",
        "/speech/audio":      "audio_common_msgs/msg/AudioData",
        "/webrtc/offer":      "std_msgs/msg/String",
        "/webrtc/app_ice":    "std_msgs/msg/String",
        "/phone/config":      "std_msgs/msg/String",   # its settings (JSON in data), on change
        "/phone/config_url":  "std_msgs/msg/String",   # its REST base URL, every 5 s
        "/hello":             "std_msgs/msg/String",
    },
    "esp32": {
        "/config":            "std_msgs/msg/String",   # config snapshot (minus wifi), JSON in data
        "/wheel_vel":         "smabo_interfaces/msg/WheelVel",
        "/joint_states":      "sensor_msgs/msg/JointState",
        "/scan":              "sensor_msgs/msg/LaserScan",
        "/pong":              "std_msgs/msg/String",
        "/config_url":        "std_msgs/msg/String",   # its REST base URL, every 5 s
        "/hello":             "std_msgs/msg/String",
    },
}

# Canonical topics each client subscribes to, with types.
SUBSCRIBE: dict[str, dict[str, str]] = {
    "web": {
        "/odom":              "nav_msgs/msg/Odometry",
        "/pong":              "std_msgs/msg/String",
        "/speech/recognized": "std_msgs/msg/String",
        "/imu/data":          "sensor_msgs/msg/Imu",
        "/gps/fix":           "sensor_msgs/msg/NavSatFix",
        "/vision/config":     "std_msgs/msg/String",
        "/vision/detections": "vision_msgs/msg/Detection2DArray",
        "/vision/markers":    "std_msgs/msg/String",
        "/webrtc/web_offer":  "std_msgs/msg/String",
        "/config":            "std_msgs/msg/String",   # ESP32 settings changed (re-read over REST)
        "/config_url":        "std_msgs/msg/String",
        "/phone/config":      "std_msgs/msg/String",
        "/phone/config_url":  "std_msgs/msg/String",
        "/brain/info":        "std_msgs/msg/String",   # the brain's protocol / version
    },
    "app": {
        "/expression":        "std_msgs/msg/String",
        "/look_at":           "geometry_msgs/msg/PoseStamped",
        "/speech/say":        "std_msgs/msg/String",
        "/webrtc/answer":     "std_msgs/msg/String",
        "/brain/info":        "std_msgs/msg/String",
    },
    "esp32": {
        "/cmd_vel":           "geometry_msgs/msg/Twist",
        "/servo/command":     "trajectory_msgs/msg/JointTrajectory",
        "/ping":              "std_msgs/msg/String",
        "/brain/info":        "std_msgs/msg/String",
    },
}

# Topics that smabo-web addresses to smabo-app (not the ESP32).  Used by
# smabo-brain to route ``/web`` publishes.
# Topics smabo-web sends only for smabo-brain-ros (Nav2 and its nav_manager).
# smabo-brain has no navigation, so it drops them instead of forwarding.
ROS_ONLY_TOPICS: frozenset[str] = frozenset({
    "/initialpose", "/goal_pose", "/nav/command",
})

APP_TOPICS: frozenset[str] = frozenset({
    "/speech/say", "/expression", "/look_at",
})


def strip_prefix(topic: str, prefix: str) -> str:
    """Remove a source prefix (e.g. ``/web``) from a publish topic.

    Returns the canonical topic when ``topic`` starts with ``prefix + "/"``,
    otherwise returns ``topic`` unchanged.
    """
    if topic.startswith(prefix + "/"):
        return topic[len(prefix):]
    return topic


def client_of_topic(topic: str) -> str | None:
    """The client whose source prefix ``topic`` carries (``"web"`` …), or None."""
    for client, prefix in SOURCE_PREFIXES.items():
        if topic.startswith(prefix + "/"):
            return client
    return None
