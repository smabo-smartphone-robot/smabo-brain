"""The protocol tables: brain.topics, smabo-protocol and the clients' copies.

smabo-protocol/protocol.json is the source; its gen.py writes brain/protocol_gen.py
and each client's copy (web, app, ESP32, sim). These tests check that the copies
are up to date and that every component's version agrees with this brain's.
Repositories that are not checked out next to smabo-brain are skipped.
"""

import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

from brain import __version__ as BRAIN_VERSION
from brain.topics import (APP_TOPICS, FROM_BRAIN, PROTOCOL_VERSION, PUBLISH,
                          ROS_ONLY_TOPICS, SOURCE_PREFIXES, SUBSCRIBE)

REPOS = Path(__file__).resolve().parents[2]
PROTOCOL = REPOS / "smabo-protocol"


def _source(rel: str) -> str:
    path = REPOS / rel
    if not path.exists():
        pytest.skip(f"{rel} is not checked out next to smabo-brain")
    return path.read_text(encoding="utf-8")


def test_tables_match_protocol_json():
    spec = json.loads(_source("smabo-protocol/protocol.json"))
    assert PROTOCOL_VERSION == spec["protocol"]
    assert SOURCE_PREFIXES == spec["source_prefixes"]
    for kind, table in (("publish", PUBLISH), ("subscribe", SUBSCRIBE)):
        assert {r: list(t) for r, t in table.items()} == spec[kind]
        assert all(typ == spec["types"][t] for topics in table.values() for t, typ in topics.items())
    assert ROS_ONLY_TOPICS == set(spec["ros_only_topics"])
    assert APP_TOPICS == set(spec["app_topics"])


def test_generated_copies_are_up_to_date():
    _source("smabo-protocol/gen.py")
    r = subprocess.run([sys.executable, str(PROTOCOL / "gen.py"), "--check"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_every_subscribed_topic_has_a_publisher_or_comes_from_the_brain():
    """Catch typos: a topic someone reads must be published by a client, or be
    one the brain itself produces."""
    published = {t for topics in PUBLISH.values() for t in topics}
    for client, topics in SUBSCRIBE.items():
        for t in topics:
            assert t in published or t in FROM_BRAIN, f"{client} reads {t}, which nobody sends"


# ── versions: same minor number everywhere (README の対応表) ─────────────────
VERSIONS = {
    "smabo-brain":     ("smabo-brain/pyproject.toml", r'^version = "([^"]+)"'),
    "smabo-web":       ("smabo-web/src/version.ts", r"WEB_VERSION = '([^']+)'"),
    "smabo-app":       ("smabo-app/pubspec.yaml", r"^version: ([0-9.]+)"),
    "smabo-esp32":     ("smabo-esp32/brain_link.py", r'FIRMWARE_VERSION = "([^"]+)"'),
    "smabo-sim":       ("smabo-sim/pyproject.toml", r'^version = "([^"]+)"'),
    "smabo-brain-ros": ("smabo-brain-ros/smabo_brain_ros/smabo_brain_ros/protocol_node.py",
                        r'BRAIN_VERSION = "([^"]+)"'),
}


@pytest.mark.parametrize("name", list(VERSIONS))
def test_component_shares_the_minor_version(name):
    rel, version_re = VERSIONS[name]
    version = re.search(version_re, _source(rel), re.M).group(1)
    assert version.split(".")[:2] == BRAIN_VERSION.split(".")[:2], (rel, version)


def test_package_version_matches_brain_init():
    src = (Path(__file__).resolve().parents[1] / "pyproject.toml").read_text(encoding="utf-8")
    assert re.search(r'^version = "([^"]+)"', src, re.M).group(1) == BRAIN_VERSION
