"""One browser drives at a time (brain/control.py)."""

import asyncio
import json

from aiohttp.test_utils import TestClient, TestServer

from brain.control import Control, is_stop
from brain.server import create_app


def test_first_command_claims_and_others_are_refused():
    c = Control()
    assert c.allow("a", "PC1") == (True, True)
    assert c.allow("a", "PC1") == (True, False)
    assert c.allow("b", "PC2") == (False, False)
    assert json.loads(c.notice("b")) == {"op": "notice", "event": "control", "holder": "PC1", "you": False}
    assert json.loads(c.notice("a"))["you"] is True


def test_take_over_and_release():
    c = Control()
    c.allow("a", "PC1")
    assert c.claim("b", "PC2") is True
    assert c.allow("a", "PC1") == (False, False)
    assert c.release("a") is False          # only the holder lets go
    assert c.release("b") is True
    assert json.loads(c.notice("a"))["holder"] is None
    assert c.allow("a", "PC1") == (True, True)


def test_stop_is_a_zero_cmd_vel():
    zero = {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0.0, "y": 0, "z": 0}}
    assert is_stop({"topic": "/cmd_vel", "msg": zero})
    assert is_stop({"topic": "/cmd_vel", "msg": {}})
    assert not is_stop({"topic": "/cmd_vel", "msg": {"linear": {"x": 0.1}}})
    assert not is_stop({"topic": "/servo/command", "msg": zero})
    assert not is_stop({"topic": "/cmd_vel", "msg": {"linear": {"x": "fast"}}})


STOP = {"op": "publish", "topic": "/web/cmd_vel",
        "msg": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}}
CMD = {"op": "publish", "topic": "/web/cmd_vel",
       "msg": {"linear": {"x": 0.3, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}}


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


def _control(frames):
    return [f for f in frames if f.get("event") == "control"]


def _cmd_x(frames):
    return [f["msg"]["linear"]["x"] for f in frames if f.get("topic") == "/cmd_vel"]


def test_two_browsers_over_the_brain():
    async def go():
        client = TestClient(TestServer(create_app(stt_config={"engine": "vosk", "model": "/nonexistent"})))
        await client.start_server()
        try:
            esp = await client.ws_connect("/")
            await esp.send_json({"op": "advertise", "topic": "/esp32/wheel_vel", "type": "x"})
            webs = []
            for _ in range(2):
                ws = await client.ws_connect("/")
                await ws.send_json({"op": "advertise", "topic": "/web/cmd_vel", "type": "x"})
                webs.append(ws)
            a, b = webs
            await _frames(esp)
            # The joining notice (via the first frame below) says nobody holds control
            await a.send_json({"op": "publish", "topic": "/web/ping", "msg": {"data": "x"}})
            assert _control(await _frames(a))[-1] == {"op": "notice", "event": "control",
                                                      "holder": None, "you": False}
            await _frames(b)

            # A drives first: it gets control, and the ESP32 its command
            await a.send_json(CMD)
            assert _cmd_x(await _frames(esp)) == [0.3]
            assert _control(await _frames(a))[-1]["you"] is True
            assert _control(await _frames(b))[-1] == {"op": "notice", "event": "control",
                                                      "holder": "browser 1", "you": False}

            # B's commands are dropped, but B may always stop the robot
            await b.send_json(CMD)
            assert _cmd_x(await _frames(esp)) == []
            await b.send_json(STOP)
            assert _cmd_x(await _frames(esp)) == [0]
            assert _control(await _frames(b)) == []           # A still holds control

            # B takes over: stop, and B drives
            await b.send_json({"op": "control", "action": "claim"})
            assert _cmd_x(await _frames(esp)) == [0.0]
            assert _control(await _frames(b))[-1]["you"] is True
            assert _control(await _frames(a))[-1]["holder"] == "browser 2"
            await a.send_json(CMD)
            await b.send_json(CMD)
            assert _cmd_x(await _frames(esp)) == [0.3]

            # B closes its tab: stop, and nobody holds control
            await b.close()
            assert _cmd_x(await _frames(esp)) == [0.0]
            assert _control(await _frames(a))[-1]["holder"] is None
            await a.close()
            await esp.close()
        finally:
            await client.close()
    asyncio.run(go())
