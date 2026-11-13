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

The tables themselves live in smabo-protocol (protocol.json), which generates
this package's protocol_gen.py and each client's copy (web: TypeScript, app:
Dart, ESP32: MicroPython, sim); ``tests/test_protocol.py`` checks they are up
to date.
"""

# The tables are generated from smabo-protocol/protocol.json (protocol_gen.py).
#   PROTOCOL_VERSION  bump when a topic is renamed or removed, or a message
#                     changes type or meaning; clients with another number are
#                     warned on connect (/hello and /brain/info)
#   SOURCE_PREFIXES   the prefix each client puts in front of what it publishes
#   PUBLISH           canonical topics each client publishes, with types
#   SUBSCRIBE         canonical topics each client reads, with types
#   ROS_ONLY_TOPICS   topics smabo-web sends only for smabo-brain-ros (Nav2);
#                     smabo-brain drops them
#   APP_TOPICS        /web topics smabo-brain routes to smabo-app (not the ESP32)
from .protocol_gen import (APP_TOPICS, FROM_BRAIN, PROTOCOL_VERSION, PUBLISH,  # noqa: F401
                           ROS_ONLY_TOPICS, SOURCE_PREFIXES, SUBSCRIBE)


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
