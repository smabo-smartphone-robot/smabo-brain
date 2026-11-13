"""Who may connect: paired clients, their tokens, and invite codes.

Every client (smabo-web in a browser, smabo-app, smabo-esp32 / smabo-sim)
shows a token when it connects. The brain issued that token when the client
paired, and keeps only its SHA-256 hash with the client's role and name, so a
leaked pairings file cannot be turned back into tokens.

A client pairs by showing a one-time code:

* the **first browser** uses the code the brain prints to its log while no
  browser is paired (``bootstrap``);
* every other client uses an **invite** that a paired browser asked for.

Both expire after 5 minutes and work once. Five wrong codes in a row throw away
every open code (and print a new bootstrap code), so guessing does not pay.

Devices (app, ESP32) also get a **relay key** when they pair. The brain keeps
it as is and sends it when it forwards smabo-web's settings to the device's
REST API; the device keeps only the hash and refuses requests without it.

The store is a JSON file (``~/.smabo/brain/pairings.json`` by default),
written with owner-only permissions. Pure logic: no networking here.
"""

import hashlib
import hmac
import json
import logging
import os
import secrets
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

log = logging.getLogger(__name__)

ROLES = ("web", "app", "esp32")
DEVICE_ROLES = ("app", "esp32")
ROLE_LABELS = {"web": "browser", "app": "phone", "esp32": "ESP32"}

CODE_TTL_S = 5 * 60
MAX_FAILURES = 5
# No 0/O, 1/I/L: codes are read off a log or a screen and typed by hand
_ALPHABET = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"
_CODE_LEN = 8

DEFAULT_PATH = Path.home() / ".smabo" / "brain" / "pairings.json"


class PairingError(Exception):
    """A refused pairing request; ``reason`` is a short machine-readable word."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason


def sha256(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def normalize_code(code: str) -> str:
    """Accept codes typed with spaces, dashes or in lower case."""
    return "".join(c for c in str(code).upper() if c.isalnum())


def format_code(code: str) -> str:
    return f"{code[:4]}-{code[4:]}"


@dataclass
class Pairing:
    id: str
    role: str
    name: str
    token_sha256: str
    created: float
    relay_key: str = ""          # devices only: sent to the device when relaying
    last_seen: float = 0.0

    def public(self) -> dict:
        """What smabo-web may see (no hash, no relay key)."""
        return {"id": self.id, "role": self.role, "name": self.name,
                "created": self.created, "last_seen": self.last_seen}


@dataclass
class _Code:
    role: str
    expires: float
    name: str = ""
    by: str = ""                 # pairing id of the browser that asked for it


@dataclass
class Paired:
    """The result of a successful pairing, sent once to the new client."""
    pairing: Pairing
    token: str
    relay_key: str = ""


@dataclass
class Pairings:
    path: Path = DEFAULT_PATH
    clock: Callable[[], float] = time.time
    on_bootstrap: Callable[[str], None] | None = None
    entries: list[Pairing] = field(default_factory=list)
    _codes: dict[str, _Code] = field(default_factory=dict)
    _bootstrap: str = ""
    _failures: int = 0

    # ── persistence ──────────────────────────────────────────────────────
    @classmethod
    def load(cls, path: Path | str = DEFAULT_PATH, **kwargs) -> "Pairings":
        p = cls(path=Path(path), **kwargs)
        try:
            raw = json.loads(p.path.read_text(encoding="utf-8"))
            p.entries = [Pairing(**e) for e in raw.get("pairings", [])]
        except FileNotFoundError:
            pass
        except (ValueError, TypeError, AttributeError) as e:
            # Never start with an empty store silently: that would print a
            # bootstrap code and let anyone on the LAN pair a browser.
            raise SystemExit(f"{p.path} is broken ({e}). Fix it, or start with --reset-pairing.")
        return p

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.chmod(self.path.parent, 0o700)
        except OSError:
            pass
        tmp = self.path.with_suffix(".tmp")
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump({"pairings": [asdict(e) for e in self.entries]}, f, indent=2)
        os.replace(tmp, self.path)

    def reset(self) -> None:
        """Forget every pairing (``--reset-pairing``)."""
        self.entries = []
        self._codes.clear()
        self._bootstrap = ""
        self.save()

    # ── codes ────────────────────────────────────────────────────────────
    def _new_code(self) -> str:
        while True:
            code = "".join(secrets.choice(_ALPHABET) for _ in range(_CODE_LEN))
            if code not in self._codes:
                return code

    def has_browser(self) -> bool:
        return any(e.role == "web" for e in self.entries)

    def ensure_bootstrap(self) -> str | None:
        """The code for the first browser, made (and announced) if needed.

        None once a browser is paired: further browsers are invited by it.
        """
        self._expire()
        if self.has_browser():
            self._drop_bootstrap()
            return None
        if self._bootstrap not in self._codes:
            self._bootstrap = self._new_code()
            self._codes[self._bootstrap] = _Code("web", self.clock() + CODE_TTL_S, name="")
            if self.on_bootstrap:
                self.on_bootstrap(format_code(self._bootstrap))
        return format_code(self._bootstrap)

    def _drop_bootstrap(self) -> None:
        self._codes.pop(self._bootstrap, None)
        self._bootstrap = ""

    def invite(self, role: str, by: str = "", name: str = "") -> tuple[str, int]:
        """A 5-minute, single-use code for a new client of ``role``."""
        if role not in ROLES:
            raise PairingError("bad_role", f"role must be one of {', '.join(ROLES)}")
        self._expire()
        code = self._new_code()
        self._codes[code] = _Code(role, self.clock() + CODE_TTL_S, name=name, by=by)
        return format_code(code), CODE_TTL_S

    def _expire(self) -> None:
        now = self.clock()
        for code in [c for c, v in self._codes.items() if v.expires <= now]:
            del self._codes[code]

    def _fail(self) -> None:
        self._failures += 1
        if self._failures >= MAX_FAILURES:
            log.warning("pairing: %d wrong codes in a row; every open code is cancelled", self._failures)
            self._failures = 0
            self._codes.clear()
            self._bootstrap = ""
            self.ensure_bootstrap()

    # ── pairing ──────────────────────────────────────────────────────────
    def pair(self, code: str, name: str = "", role: str | None = None) -> Paired:
        """Trade a code for a token. ``role``, if given, must match the code's."""
        key = normalize_code(code)
        now = self.clock()
        entry = self._codes.get(key)
        if entry is None:
            self._fail()
            raise PairingError("wrong_code", "This code is not valid (wrong, used or expired)")
        if entry.expires <= now:
            del self._codes[key]
            if key == self._bootstrap:
                self._bootstrap = ""
                self.ensure_bootstrap()
            raise PairingError("expired", "This code has expired")
        if role is not None and role != entry.role:
            self._fail()
            raise PairingError("wrong_role", f"This code is for a {ROLE_LABELS[entry.role]}")
        del self._codes[key]
        if key == self._bootstrap:
            self._bootstrap = ""
        self._failures = 0

        token = secrets.token_urlsafe(32)
        relay_key = secrets.token_urlsafe(32) if entry.role in DEVICE_ROLES else ""
        pairing = Pairing(
            id=secrets.token_hex(4),
            role=entry.role,
            name=self._unique_name(entry.name or name or ROLE_LABELS[entry.role]),
            token_sha256=sha256(token),
            created=now,
            relay_key=relay_key,
        )
        self.entries.append(pairing)
        self.save()
        log.info("paired %s %r (%s)", ROLE_LABELS[pairing.role], pairing.name, pairing.id)
        return Paired(pairing, token, relay_key)

    def _unique_name(self, name: str) -> str:
        name = name.strip()[:40] or "client"
        taken = {e.name for e in self.entries}
        if name not in taken:
            return name
        n = 2
        while f"{name} {n}" in taken:
            n += 1
        return f"{name} {n}"

    # ── checking and managing ────────────────────────────────────────────
    def check(self, token: str) -> Pairing | None:
        """The pairing a token belongs to, or None."""
        if not isinstance(token, str) or not token:
            return None
        digest = sha256(token)
        for e in self.entries:
            if hmac.compare_digest(e.token_sha256, digest):
                return e
        return None

    def get(self, pairing_id: str) -> Pairing | None:
        return next((e for e in self.entries if e.id == pairing_id), None)

    def seen(self, pairing: Pairing) -> None:
        """Record a connection (kept in memory; saved with the next change)."""
        pairing.last_seen = self.clock()

    def remove(self, pairing_id: str) -> Pairing:
        e = self.get(pairing_id)
        if e is None:
            raise PairingError("not_found", "No such pairing")
        if e.role == "web" and sum(1 for x in self.entries if x.role == "web") == 1:
            raise PairingError("last_browser", "The last browser cannot be removed")
        self.entries.remove(e)
        self.save()
        log.info("removed %s %r (%s)", ROLE_LABELS[e.role], e.name, e.id)
        return e

    def rename(self, pairing_id: str, name: str) -> Pairing:
        e = self.get(pairing_id)
        if e is None:
            raise PairingError("not_found", "No such pairing")
        others = [x for x in self.entries if x is not e]
        name = name.strip()[:40]
        if not name:
            raise PairingError("bad_name", "The name is empty")
        if any(x.name == name for x in others):
            raise PairingError("name_taken", "Another client has this name")
        e.name = name
        self.save()
        return e

    def public(self) -> list[dict]:
        return [e.public() for e in self.entries]
