"""Who may drive: one browser at a time moves the robot.

Several smabo-web tabs (or people) can watch the robot at once, but only one
of them drives (``/cmd_vel``) and moves the arm (``/servo/command``) at a time:
the **holder**. Otherwise two joysticks would fight, each sending its own
speed many times a second.

* Nobody holds control: the first browser that sends a drive or arm command
  gets it, without asking.
* Someone holds it: commands from the other browsers are dropped. They show
  "<holder> is driving" and a "Take over" button (``{"op": "control",
  "action": "claim"}``), which hands control over at once.
* Anyone may stop the robot: a zero ``/cmd_vel`` always goes through (and
  does not claim control).
* The holder lets go with ``{"op": "control", "action": "release"}``, or by
  closing the tab / losing the connection.

Whenever control passes from one holder to another (or is let go), the brain stops the robot (a zero ``/cmd_vel``)
so it does not keep going on the old holder's last command, and tells every
browser who holds control now (a ``control`` notice, see :meth:`Control.notice`).

Pure logic: connections are any hashable objects; no networking here.
"""

import json
from typing import Hashable

CONTROL_TOPICS = frozenset({"/cmd_vel", "/servo/command"})


def is_stop(frame: dict) -> bool:
    """A ``/cmd_vel`` with every speed zero (allowed from any browser)."""
    if frame.get("topic") != "/cmd_vel":
        return False
    msg = frame.get("msg")
    if not isinstance(msg, dict):
        return False
    try:
        return all(float((msg.get(part) or {}).get(axis, 0) or 0) == 0.0
                   for part in ("linear", "angular") for axis in ("x", "y", "z"))
    except (TypeError, ValueError, AttributeError):
        return False


class Control:
    def __init__(self) -> None:
        self.holder: Hashable | None = None
        self.holder_name = ""

    def claim(self, conn: Hashable, name: str) -> bool:
        """Take control (also from another holder). True when the holder changed."""
        if self.holder is conn:
            return False
        self.holder, self.holder_name = conn, name
        return True

    def release(self, conn: Hashable) -> bool:
        """Let go, if ``conn`` holds control. True when the holder changed."""
        if self.holder is not conn:
            return False
        self.holder, self.holder_name = None, ""
        return True

    def allow(self, conn: Hashable, name: str) -> tuple[bool, bool]:
        """May ``conn`` send a drive / arm command? ``(allowed, holder changed)``.

        A command from anyone while nobody holds control claims it.
        """
        if self.holder is None:
            self.claim(conn, name)
            return True, True
        return self.holder is conn, False

    def notice(self, conn: Hashable) -> str:
        """The ``control`` notice for one browser: who holds control, and whether it is them."""
        return json.dumps({"op": "notice", "event": "control",
                           "holder": self.holder_name if self.holder is not None else None,
                           "you": self.holder is not None and self.holder is conn})
