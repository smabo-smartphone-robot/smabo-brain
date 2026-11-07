"""smabo-brain の振り分け: 全クライアントが "/" につなぎ、役割はフレームから判別する。"""

import asyncio
import json

from aiohttp.test_utils import TestClient, TestServer

from brain.server import BRAIN_KEY, create_app

TWIST = {"linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}


def run(coro):
    return asyncio.run(coro)


async def _client():
    client = TestClient(TestServer(create_app(stt_config={"engine": "vosk", "model": "/nonexistent"})))
    await client.start_server()
    return client


async def _recv_all(ws, timeout=0.3):
    out = []
    while True:
        try:
            m = await ws.receive(timeout=timeout)
        except asyncio.TimeoutError:
            return out
        if m.type.name != "TEXT":
            return out
        out.append(json.loads(m.data))


async def _connect_esp32(client):
    ws = await client.ws_connect("/")
    await ws.send_json({"op": "advertise", "topic": "/esp32/wheel_vel", "type": "smabo_interfaces/msg/WheelVel"})
    await ws.send_json({"op": "subscribe", "topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"})
    return ws


async def _connect_web(client):
    ws = await client.ws_connect("/")
    await ws.send_json({"op": "advertise", "topic": "/web/cmd_vel", "type": "geometry_msgs/msg/Twist"})
    return ws


async def _connect_app(client):
    ws = await client.ws_connect("/")
    await ws.send_json({"op": "subscribe", "topic": "/expression", "type": "std_msgs/msg/Int32"})
    return ws


def test_only_root_path_is_served():
    async def go():
        client = await _client()
        try:
            for path in ("/ui", "/esp32"):
                resp = await client.get(path)
                assert resp.status == 404
        finally:
            await client.close()
    run(go())


def test_web_drive_command_reaches_esp32_without_prefix():
    async def go():
        client = await _client()
        try:
            esp = await _connect_esp32(client)
            web = await _connect_web(client)
            await asyncio.sleep(0.05)
            await web.send_json({"op": "publish", "topic": "/web/cmd_vel", "msg": TWIST})
            got = await _recv_all(esp)
            assert {"op": "publish", "topic": "/cmd_vel", "msg": TWIST} in got
            # housekeeping ops from the web are not relayed to the ESP32
            assert all(f["op"] == "publish" for f in got)
        finally:
            await client.close()
    run(go())


def test_web_gets_snapshot_and_connection_notices():
    async def go():
        client = await _client()
        try:
            await _connect_esp32(client)
            await asyncio.sleep(0.05)
            web = await _connect_web(client)
            first = await _recv_all(web)
            assert first[0]["topic"] == "/vision/config"
            assert {"op": "notice", "source": "esp32", "event": "connected", "total": 1} in first
            app = await _connect_app(client)
            await asyncio.sleep(0.05)
            await app.close()
            notices = [(f["source"], f["event"]) for f in await _recv_all(web) if f["op"] == "notice"]
            assert notices == [("app", "connected"), ("app", "disconnected")]
        finally:
            await client.close()
    run(go())


def test_app_topics_go_to_app_not_esp32():
    async def go():
        client = await _client()
        try:
            esp = await _connect_esp32(client)
            app = await _connect_app(client)
            web = await _connect_web(client)
            await asyncio.sleep(0.05)
            await web.send_json({"op": "publish", "topic": "/web/expression", "msg": {"data": 3}})
            assert {"op": "publish", "topic": "/expression", "msg": {"data": 3}} in await _recv_all(app)
            assert not [f for f in await _recv_all(esp) if f.get("topic") == "/expression"]
        finally:
            await client.close()
    run(go())


def test_esp32_config_sets_odometry_and_wheel_vel_becomes_odom():
    async def go():
        client = await _client()
        try:
            esp = await _connect_esp32(client)
            web = await _connect_web(client)
            await asyncio.sleep(0.05)
            await _recv_all(web)
            cfg = {"dc": {"wheel_separation": 0.5}, "servos": {"joints": {"head_pan": {}}}}
            await esp.send_json({"op": "publish", "topic": "/esp32/config", "msg": {"data": json.dumps(cfg)}})
            # turning in place: wz = (right - left) / separation = 0.2 / 0.5
            await esp.send_json({"op": "publish", "topic": "/esp32/wheel_vel",
                                 "msg": {"left": -0.1, "right": 0.1, "dt": 0.1}})
            got = await _recv_all(web)
            assert not [f for f in got if f.get("topic") == "/config"]   # config stays in the brain
            odom = [f for f in got if f.get("topic") == "/odom"]
            assert odom and abs(odom[-1]["msg"]["twist"]["twist"]["angular"]["z"] - 0.4) < 1e-6
            assert client.app[BRAIN_KEY].vision.joints == {"head_pan"}
        finally:
            await client.close()
    run(go())


def test_frames_before_the_role_is_known_are_ignored():
    async def go():
        client = await _client()
        try:
            esp = await _connect_esp32(client)
            stranger = await client.ws_connect("/")
            await stranger.send_json({"op": "call_service", "service": "/rosapi/topics"})
            await stranger.send_json({"op": "publish", "topic": "/cmd_vel", "msg": TWIST})
            assert not [f for f in await _recv_all(esp) if f.get("op") == "publish"]
            assert client.app[BRAIN_KEY].clients.count("web") == 0
        finally:
            await client.close()
    run(go())
