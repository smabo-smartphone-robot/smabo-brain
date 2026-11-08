"""Pairing over the WebSocket, roles from tokens, and the /devices relay."""

import asyncio
import json

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from brain.gateway import RELAY_KEY_HEADER
from brain.pairing import Pairings
from brain.server import create_app


def run(coro):
    return asyncio.run(coro)


async def _client(tmp_path):
    pairings = Pairings.load(tmp_path / "pairings.json")
    client = TestClient(TestServer(create_app(stt_config={"engine": "vosk", "model": "/nonexistent"},
                                              pairings=pairings)))
    await client.start_server()
    return client, pairings


async def _frames(ws, timeout=0.3):
    out = []
    while True:
        try:
            m = await ws.receive(timeout=timeout)
        except asyncio.TimeoutError:
            return out
        if m.type.name != "TEXT":
            return out
        out.append(json.loads(m.data))


async def _pair(client, code, role=None, name=""):
    ws = await client.ws_connect("/")
    await ws.send_json({"op": "pair", "code": code, "name": name, **({"role": role} if role else {})})
    reply = await ws.receive_json(timeout=2)
    return ws, reply


async def _auth(client, token):
    ws = await client.ws_connect("/")
    await ws.send_json({"op": "auth", "token": token})
    return ws, await ws.receive_json(timeout=2)


def test_first_browser_pairs_then_reconnects_with_its_token(tmp_path):
    async def go():
        client, pairings = await _client(tmp_path)
        try:
            ws, reply = await _pair(client, pairings.ensure_bootstrap(), name="PC1")
            assert reply["ok"] and reply["role"] == "web" and reply["token"]
            info = [f for f in await _frames(ws) if f.get("topic") == "/brain/info"]
            assert json.loads(info[0]["msg"]["data"])["auth"] is True
            await ws.close()

            ws, reply = await _auth(client, reply["token"])
            assert reply == {"op": "auth", "ok": True, "role": "web", "name": "PC1", "id": reply["id"]}
            await ws.close()
        finally:
            await client.close()
    run(go())


def test_old_clients_and_bad_tokens_are_closed(tmp_path):
    async def go():
        client, _ = await _client(tmp_path)
        try:
            ws = await client.ws_connect("/")
            await ws.send_json({"op": "advertise", "topic": "/esp32/wheel_vel", "type": "x"})
            reply = await ws.receive_json(timeout=2)
            assert reply["ok"] is False and reply["reason"] == "unauthorized"
            closed = await ws.receive(timeout=2)
            assert closed.type.name in ("CLOSE", "CLOSED") and ws.close_code == 4001

            ws, reply = await _auth(client, "nope")
            assert reply["reason"] == "unauthorized"
            ws, reply = await _pair(client, "AAAA-AAAA")
            assert reply["op"] == "pair" and reply["reason"] == "wrong_code"
        finally:
            await client.close()
    run(go())


async def _setup_web_and_esp32(client, pairings):
    web_ws, w = await _pair(client, pairings.ensure_bootstrap(), name="PC1")
    resp = await client.post("/invites", json={"role": "esp32"},
                             headers={"Authorization": f"Bearer {w['token']}"})
    assert resp.status == 200
    code = (await resp.json())["code"]
    esp_ws, e = await _pair(client, code, role="esp32", name="smabo")
    assert e["ok"] and e["relay_key"]
    await _frames(web_ws)
    return web_ws, w, esp_ws, e


def test_role_comes_from_the_token_and_other_prefixes_are_dropped(tmp_path):
    async def go():
        client, pairings = await _client(tmp_path)
        try:
            web_ws, _, esp_ws, _ = await _setup_web_and_esp32(client, pairings)
            await _frames(esp_ws)
            # the browser pretends to be the ESP32: dropped
            await web_ws.send_json({"op": "publish", "topic": "/esp32/joint_states", "msg": {}})
            await web_ws.send_json({"op": "publish", "topic": "/web/cmd_vel", "msg": {"linear": {"x": 1}}})
            got = await _frames(esp_ws)
            assert [f["topic"] for f in got if f.get("op") == "publish"] == ["/cmd_vel"]
            assert not [f for f in await _frames(web_ws) if f.get("topic") == "/joint_states"]
        finally:
            await client.close()
    run(go())


def test_management_needs_a_browser_token(tmp_path):
    async def go():
        client, pairings = await _client(tmp_path)
        try:
            web_ws, w, esp_ws, e = await _setup_web_and_esp32(client, pairings)
            assert (await client.get("/pairings")).status == 401
            r = await client.get("/pairings", headers={"Authorization": f"Bearer {e['token']}"})
            assert r.status == 403
            r = await client.get("/pairings", headers={"Authorization": f"Bearer {w['token']}"})
            body = await r.json()
            assert body["you"] == w["id"]
            assert {(p["role"], p["online"]) for p in body["pairings"]} == {("web", True), ("esp32", True)}
            assert r.headers["Access-Control-Allow-Origin"] == "*"
            # CORS preflight needs no token
            assert (await client.options("/pairings")).status == 204

            # last browser stays; removing the ESP32 closes its connection
            auth = {"Authorization": f"Bearer {w['token']}"}
            assert (await client.delete(f"/pairings/{w['id']}", headers=auth)).status == 409
            assert (await client.delete(f"/pairings/{e['id']}", headers=auth)).status == 204
            await _frames(esp_ws, timeout=0.5)
            assert esp_ws.closed
            _, again = await _auth(client, e["token"])
            assert again["reason"] == "unauthorized"
        finally:
            await client.close()
    run(go())


def test_devices_relay_forwards_with_the_relay_key(tmp_path):
    async def go():
        seen = []

        async def device_config(request):
            seen.append((request.method, request.headers.get(RELAY_KEY_HEADER), await request.text(),
                         request.headers.get("Authorization")))
            if request.headers.get(RELAY_KEY_HEADER) != expected["key"]:
                return web.json_response({"error": "relay_key"}, status=401)
            return web.json_response({"ok": True, "q": request.query_string})

        device = web.Application()
        device.router.add_route("*", "/config", device_config)
        dev = TestServer(device)
        await dev.start_server()
        expected = {}

        client, pairings = await _client(tmp_path)
        try:
            web_ws, w, esp_ws, e = await _setup_web_and_esp32(client, pairings)
            auth = {"Authorization": f"Bearer {w['token']}"}
            r = await client.get("/devices/esp32/config", headers=auth)
            assert r.status == 504  # no /config_url yet

            await esp_ws.send_json({"op": "publish", "topic": "/esp32/config_url",
                                    "msg": {"data": str(dev.make_url("/"))}})
            await asyncio.sleep(0.1)
            expected["key"] = e["relay_key"]
            r = await client.post("/devices/esp32/config?x=1", json={"dc": {"max_linear": 0.2}}, headers=auth)
            assert r.status == 200 and (await r.json()) == {"ok": True, "q": "x=1"}
            method, key, body, browser_auth = seen[-1]
            assert method == "POST" and key == e["relay_key"] and json.loads(body)["dc"]["max_linear"] == 0.2
            assert browser_auth is None   # the browser's token stays at the brain

            expected["key"] = "something else"
            r = await client.get("/devices/esp32/config", headers=auth)
            assert r.status == 502 and (await r.json())["error"] == "device_refused"

            assert (await client.get("/devices/esp32/config")).status == 401
            assert (await client.get("/devices/robot/config", headers=auth)).status == 404

            await esp_ws.close()
            await asyncio.sleep(0.1)
            assert (await client.get("/devices/esp32/config", headers=auth)).status == 504
        finally:
            await client.close()
            await dev.close()
    run(go())


def test_no_auth_mode_relays_without_a_key(tmp_path):
    async def go():
        client = TestClient(TestServer(create_app(stt_config={"engine": "vosk", "model": "/nonexistent"})))
        await client.start_server()
        try:
            assert (await client.get("/pairings")).status == 404
            assert (await client.get("/devices/esp32/config")).status == 504
        finally:
            await client.close()
    run(go())
