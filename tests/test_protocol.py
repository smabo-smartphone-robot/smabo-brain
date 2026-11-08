"""The clients' own copies of the topic tables must match brain/topics.py.

web (TypeScript), app (Dart), ESP32 (MicroPython) and sim each keep the list of
topics they advertise / subscribe, because they can't import this package.
These tests read those copies from sibling checkouts (smabo-repos/<repo>) and
compare them with brain.topics. A client whose repository is not checked out
next to smabo-brain is skipped.
"""

import importlib.util
import re
from pathlib import Path

import pytest

from brain.topics import PUBLISH, SOURCE_PREFIXES, SUBSCRIBE

REPOS = Path(__file__).resolve().parents[2]


def _source(rel: str) -> str:
    path = REPOS / rel
    if not path.exists():
        pytest.skip(f"{rel} is not checked out next to smabo-brain")
    return path.read_text(encoding="utf-8")


def _load(rel: str, name: str):
    _source(rel)  # skip if missing
    spec = importlib.util.spec_from_file_location(name, REPOS / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _prefixed(client: str) -> dict[str, str]:
    prefix = SOURCE_PREFIXES[client]
    return {prefix + t: typ for t, typ in PUBLISH[client].items()}


# ── smabo-web: src/ws/brain.ts ────────────────────────────────────────────
def _ts_table(src: str, name: str) -> dict[str, str]:
    m = re.search(name + r"[^{]*\{(.*?)\n\};", src, re.S)
    assert m, f"{name} not found in brain.ts"
    return dict(re.findall(r"'(/[^']+)':\s*'([^']+)'", m.group(1)))


def test_web_tables_match():
    src = _source("smabo-web/src/ws/brain.ts")
    publish = _ts_table(src, "ROSBRIDGE_PUBLISH")
    subscribe = _ts_table(src, "ROSBRIDGE_SUBSCRIBE")
    # /initialpose and /goal_pose are published by the Nav panel through
    # roslib (smabo-brain-ros only), so brain.ts lists a subset.
    assert publish.items() <= PUBLISH["web"].items()
    assert subscribe == SUBSCRIBE["web"]


# ── smabo-app: lib/core/wire/ros_compat.dart ─────────────────────────────
def _dart_table(src: str, name: str, consts: dict[str, str]) -> dict[str, str]:
    m = re.search(r"static const " + name + r" = <String, String>\{(.*?)\};", src, re.S)
    assert m, f"RosProtocol.{name} not found"
    out = {}
    for key, typ in re.findall(r"\n\s*([^:\n]+):\s*'([^']+)',", m.group(1)):
        key = key.strip()
        topic = key.strip("'") if key.startswith("'") else consts[key.split(".")[1]]
        out[topic] = typ
    return out


def test_app_tables_match():
    src = _source("smabo-app/lib/core/wire/ros_compat.dart")
    topics_block = re.search(r"class RosTopics \{(.*?)\n\}", src, re.S).group(1)
    consts = dict(re.findall(r"static const (\w+) = '([^']+)';", topics_block))
    assert re.search(r"sourcePrefix = '([^']+)'", src).group(1) == SOURCE_PREFIXES["app"]
    assert _dart_table(src, "publish", consts) == PUBLISH["app"]
    assert _dart_table(src, "subscribe", consts) == SUBSCRIBE["app"]


# ── smabo-esp32: brain_link.py ───────────────────────────────────────────
def test_esp32_tables_match():
    link = _load("smabo-esp32/brain_link.py", "esp32_brain_link")
    assert link.SOURCE_PREFIX == SOURCE_PREFIXES["esp32"]
    assert dict(link.ADVERTISE) == _prefixed("esp32")
    assert dict(link.SUBSCRIBE) == SUBSCRIBE["esp32"]


# ── smabo-sim: smabo_sim/protocol.py (simulated ESP32 and phone) ──────────
def test_sim_tables_match():
    proto = _load("smabo-sim/smabo_sim/protocol.py", "sim_protocol")
    assert proto.ESP32_PREFIX == SOURCE_PREFIXES["esp32"]
    assert proto.APP_PREFIX == SOURCE_PREFIXES["app"]
    assert dict(proto.ESP32_ADVERTISE) == _prefixed("esp32")
    assert dict(proto.ESP32_SUBSCRIBE) == SUBSCRIBE["esp32"]
    assert dict(proto.APP_ADVERTISE) == _prefixed("app")
    assert dict(proto.APP_SUBSCRIBE) == SUBSCRIBE["app"]


def test_every_subscribed_topic_has_a_publisher_or_comes_from_the_brain():
    """Catch typos: a topic someone reads must be published by a client, or be
    one the brain itself produces."""
    from_brain = {"/odom", "/brain/info", "/vision/config", "/vision/detections", "/vision/markers",
                  "/speech/recognized", "/webrtc/web_offer", "/webrtc/answer"}
    published = {t for topics in PUBLISH.values() for t in topics}
    for client, topics in SUBSCRIBE.items():
        for t in topics:
            assert t in published or t in from_brain, f"{client} reads {t}, which nobody sends"
