"""The brain's HTTP API for smabo-web: pairing management and the device relay.

smabo-web talks to the brain only. Settings for the phone and the ESP32 go
through the brain, which forwards them to the device's own REST API with the
device's relay key; the devices accept nothing else once paired, so they need
no CORS for browsers.

Endpoints (all but OPTIONS need ``Authorization: Bearer <browser token>``):

  GET    /pairings                   paired clients, and which one is you
  POST   /invites    {role, name?}   a 5-minute single-use code for a new client
  PATCH  /pairings/{id} {name}       rename
  DELETE /pairings/{id}              unpair (closes its connections)
  *      /devices/{app|esp32}/{path} forwarded to that device's REST API

Without ``--pairing`` (the default) nothing needs a token, and the relay sends no key.

Browsers send the token as a header, never a cookie, so allowing any origin
(CORS ``*``) lets no other site act with the user's rights.
"""

import json
import logging
from typing import TYPE_CHECKING

import aiohttp
from aiohttp import web

from .pairing import DEVICE_ROLES, Pairing, PairingError

if TYPE_CHECKING:
    from .server import Brain

log = logging.getLogger(__name__)

RELAY_KEY_HEADER = "X-Smabo-Relay-Key"
RELAY_TIMEOUT_S = 5
# Forwarded both ways; everything else (cookies, the browser's token) stays behind.
_FORWARD_REQUEST_HEADERS = ("Content-Type", "Accept")
_FORWARD_RESPONSE_HEADERS = ("Content-Type",)

_CORS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, PATCH, DELETE, OPTIONS",
    "Access-Control-Allow-Headers": "Authorization, Content-Type",
    "Access-Control-Max-Age": "600",
}


def _error(status: int, reason: str, message: str) -> web.Response:
    return web.json_response({"error": reason, "message": message}, status=status)


@web.middleware
async def cors_middleware(request: web.Request, handler):
    if request.method == "OPTIONS":
        return web.Response(status=204, headers=_CORS)
    try:
        resp = await handler(request)
    except web.HTTPException as e:
        e.headers.update(_CORS)
        raise
    if not isinstance(resp, web.WebSocketResponse):
        resp.headers.update(_CORS)
    return resp


class Gateway:
    def __init__(self, brain: "Brain") -> None:
        self.brain = brain
        self._session: aiohttp.ClientSession | None = None

    def routes(self) -> list[web.RouteDef]:
        return [
            web.get("/pairings", self.list_pairings),
            web.post("/invites", self.create_invite),
            web.patch("/pairings/{id}", self.rename_pairing),
            web.delete("/pairings/{id}", self.remove_pairing),
            web.route("*", "/devices/{role}/{tail:.*}", self.relay),
        ]

    async def close(self) -> None:
        if self._session is not None:
            await self._session.close()

    # ── auth ────────────────────────────────────────────────────────────
    def _browser(self, request: web.Request) -> Pairing | None:
        """The browser pairing behind this request; raises 401/403 when there is none."""
        pairings = self.brain.pairings
        if pairings is None:
            return None
        header = request.headers.get("Authorization", "")
        token = header[7:] if header.startswith("Bearer ") else ""
        who = pairings.check(token)
        if who is None:
            raise web.HTTPUnauthorized(
                text=json.dumps({"error": "unauthorized", "message": "Pair this browser again"}),
                content_type="application/json")
        if who.role != "web":
            raise web.HTTPForbidden(
                text=json.dumps({"error": "forbidden", "message": "Only a browser can do this"}),
                content_type="application/json")
        return who

    def _need_pairings(self):
        if self.brain.pairings is None:
            raise web.HTTPNotFound(
                text=json.dumps({"error": "no_auth", "message": "The brain runs without pairing (start it with --pairing)"}),
                content_type="application/json")
        return self.brain.pairings

    # ── pairing management ──────────────────────────────────────────────
    async def list_pairings(self, request: web.Request) -> web.Response:
        pairings = self._need_pairings()
        me = self._browser(request)
        online = self.brain.online_pairing_ids()
        items = [dict(p, online=p["id"] in online) for p in pairings.public()]
        return web.json_response({"pairings": items, "you": me.id if me else None})

    async def create_invite(self, request: web.Request) -> web.Response:
        pairings = self._need_pairings()
        me = self._browser(request)
        body = await _json_body(request)
        try:
            code, ttl = pairings.invite(str(body.get("role", "")), by=me.id if me else "",
                                        name=str(body.get("name") or ""))
        except PairingError as e:
            return _error(400, e.reason, str(e))
        log.info("invite for a new %s created by %s", body.get("role"), me.name if me else "?")
        return web.json_response({"code": code, "role": body["role"], "expires_in": ttl})

    async def rename_pairing(self, request: web.Request) -> web.Response:
        pairings = self._need_pairings()
        self._browser(request)
        body = await _json_body(request)
        try:
            e = pairings.rename(request.match_info["id"], str(body.get("name", "")))
        except PairingError as err:
            return _error(404 if err.reason == "not_found" else 400, err.reason, str(err))
        return web.json_response(e.public())

    async def remove_pairing(self, request: web.Request) -> web.Response:
        pairings = self._need_pairings()
        self._browser(request)
        try:
            e = pairings.remove(request.match_info["id"])
        except PairingError as err:
            return _error(404 if err.reason == "not_found" else 409, err.reason, str(err))
        await self.brain.disconnect_pairing(e.id)
        return web.Response(status=204)

    # ── device relay ────────────────────────────────────────────────────
    async def relay(self, request: web.Request) -> web.StreamResponse:
        role = request.match_info["role"]
        if role not in DEVICE_ROLES:
            return _error(404, "no_device", "Devices are 'app' and 'esp32'")
        self._browser(request)
        target = self.brain.device_target(role)
        if target is None:
            label = "phone" if role == "app" else "ESP32"
            return _error(504, "device_offline", f"The {label} is not connected to the brain")
        base, relay_key = target
        url = base.rstrip("/") + "/" + request.match_info["tail"]
        if request.query_string:
            url += "?" + request.query_string
        headers = {h: request.headers[h] for h in _FORWARD_REQUEST_HEADERS if h in request.headers}
        if relay_key:
            headers[RELAY_KEY_HEADER] = relay_key
        if self._session is None:
            self._session = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=RELAY_TIMEOUT_S))
        try:
            async with self._session.request(request.method, url, headers=headers,
                                             data=await request.read()) as resp:
                body = await resp.read()
                if resp.status in (401, 403):
                    return _error(502, "device_refused",
                                  "The device refused the brain's relay key. Pair the device again")
                out = {h: resp.headers[h] for h in _FORWARD_RESPONSE_HEADERS if h in resp.headers}
                return web.Response(status=resp.status, body=body, headers=out)
        except (aiohttp.ClientError, TimeoutError) as e:
            log.info("relay to %s failed: %s", url, e)
            return _error(504, "device_unreachable", "The device did not answer")


async def _json_body(request: web.Request) -> dict:
    try:
        body = await request.json()
    except (ValueError, UnicodeDecodeError):
        return {}
    return body if isinstance(body, dict) else {}
