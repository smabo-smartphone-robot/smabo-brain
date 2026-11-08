"""Protocol check on connect: the clients' ``/hello`` and the brain's ``/brain/info``.

Each client publishes ``/hello`` once after its handshake::

    {"protocol": 1, "client": "smabo-web", "version": "0.2.0"}

The brain answers that client on ``/brain/info`` with its own numbers. When the
protocol numbers differ, the brain logs it and tells smabo-web (a ``hello``
notice with ``ok: false``), and the client warns on its side as well. Version
strings are informational; only ``protocol`` decides compatibility.
"""

import json

from . import __version__, frames
from .topics import PROTOCOL_VERSION

BRAIN_NAME = "smabo-brain"


def info() -> dict:
    """What the brain says about itself on ``/brain/info``."""
    return {"protocol": PROTOCOL_VERSION, "brain": BRAIN_NAME, "version": __version__}


def info_frame() -> str:
    """``/brain/info`` publish frame (std_msgs/String, JSON in ``data``)."""
    return frames.pub("/brain/info", {"data": json.dumps(info())})


def parse(frame: dict) -> dict:
    """The fields of a ``/hello`` publish frame; unknown or missing values become None."""
    data = frames.data_json(frame)
    protocol = data.get("protocol")
    return {
        "protocol": protocol if isinstance(protocol, int) and not isinstance(protocol, bool) else None,
        "client": data.get("client") if isinstance(data.get("client"), str) else None,
        "version": data.get("version") if isinstance(data.get("version"), str) else None,
    }


def compatible(hello: dict) -> bool:
    """True when the client speaks this brain's protocol number."""
    return hello.get("protocol") == PROTOCOL_VERSION


def notice(role: str, hello: dict) -> str:
    """Notice for smabo-web: which client said hello, with what numbers, and whether they match."""
    return json.dumps({
        "op": "notice", "source": role, "event": "hello",
        "client": hello.get("client"), "version": hello.get("version"),
        "protocol": hello.get("protocol"), "expected": PROTOCOL_VERSION,
        "ok": compatible(hello),
    })
