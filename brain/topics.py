"""Source-prefix conventions shared by every smabo transport.

Each client publishes with a source prefix so the hub can tell who sent a
message and re-broadcast it under a canonical (prefix-less) topic name:

    smabo-web   → ``/web``      smabo-app   → ``/app``
    smabo-esp32 → ``/esp32``

The WebSocket relay (``brain/relay.py``) strips these inline.  The ROS 2
wrapper (smabo-brain-ros) imports the same constants/helpers so the
prefix→canonical mapping lives in exactly one place — see the relay launch in
smabo-brain-ros, which mirrors this list as ``topic_tools`` remaps.

Keeping this here (not in relay.py) is what lets brain-ros reuse it without
pulling in aiohttp.
"""

# Canonical source prefixes, keyed by the client that publishes with them.
SOURCE_PREFIXES: dict[str, str] = {
    "web":   "/web",
    "app":   "/app",
    "esp32": "/esp32",
}

# Topics that smabo-web addresses to smabo-app (not the ESP32).  Used by the
# relay to route ``/web`` publishes; brain-ros exposes these as ROS topics too.
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
