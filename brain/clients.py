"""Connected clients, grouped by kind ("role"), and how a role is recognised.

Every client connects to the same path ``/`` (as with rosbridge in
smabo-brain-ros), so the brain recognises the role from the first frame that
reveals it: a publish/advertise carrying a source prefix (``/web/…``,
``/app/…``, ``/esp32/…``), or a subscribe to a topic only one role reads.
"""

import logging

from aiohttp import web

from . import frames
from .topics import SOURCE_PREFIXES, SUBSCRIBE, client_of_topic

log = logging.getLogger(__name__)

ROLES = tuple(SOURCE_PREFIXES)          # ("web", "app", "esp32")

# rosbridge bookkeeping ops. smabo-brain delivers by role, not by subscription,
# so these only serve to recognise the role and are not relayed.
HOUSEKEEPING_OPS = frozenset({"advertise", "unadvertise", "subscribe", "unsubscribe"})


def _subscribe_roles() -> dict[str, str]:
    """Topics subscribed by exactly one role → that role."""
    owners: dict[str, set[str]] = {}
    for role, topics in SUBSCRIBE.items():
        for t in topics:
            owners.setdefault(t, set()).add(role)
    return {t: next(iter(r)) for t, r in owners.items() if len(r) == 1}


_SUBSCRIBE_ROLE = _subscribe_roles()


def role_of_frame(frame: dict) -> str | None:
    """The role a frame reveals, or None if it does not tell."""
    op = frame.get("op")
    topic = frame.get("topic")
    if not isinstance(topic, str):
        return None
    if op in ("publish", "advertise"):
        return client_of_topic(topic)
    if op == "subscribe":
        return _SUBSCRIBE_ROLE.get(topic)
    return None


class Clients:
    """The open WebSocket connections of each role."""

    def __init__(self) -> None:
        self._by_role: dict[str, set[web.WebSocketResponse]] = {r: set() for r in ROLES}

    def count(self, role: str) -> int:
        return len(self._by_role[role])

    def add(self, ws: web.WebSocketResponse, role: str) -> None:
        self._by_role[role].add(ws)
        log.info("%s connected (total=%d)", role, self.count(role))

    def remove(self, ws: web.WebSocketResponse, role: str) -> None:
        self._by_role[role].discard(ws)
        log.info("%s disconnected (total=%d)", role, self.count(role))

    async def send(self, role: str, text: str) -> None:
        """Send one frame to every client of ``role``; drop the ones that fail."""
        targets = self._by_role[role]
        dead = set()
        for ws in targets:
            try:
                await ws.send_str(text)
            except Exception:
                dead.add(ws)
        targets -= dead

    async def notify_web(self, role: str, event: str) -> None:
        """Tell smabo-web that an app / ESP32 connected or disconnected."""
        await self.send("web", frames.notice(role, event, self.count(role)))

    def snapshot_notices(self) -> list[str]:
        """Notices describing who is connected now (sent to a newly joined web)."""
        return [frames.notice(r, "connected", self.count(r))
                for r in ("app", "esp32") if self.count(r)]
