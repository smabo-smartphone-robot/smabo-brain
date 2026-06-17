import json
import logging
import math
import time
import aiohttp
from aiohttp import web

from .odometry import Odometry, build_pose_covariance, build_twist_covariance

log = logging.getLogger(__name__)

_app_clients:   set[web.WebSocketResponse] = set()
_ui_clients:    set[web.WebSocketResponse] = set()
_esp32_clients: set[web.WebSocketResponse] = set()

# web から来た publish のうち、esp32 ではなく smabo-app へ流すトピック
_APP_TOPICS = {'/speech/say', '/expression'}

_odom = Odometry()


def _odom_frame(r: dict) -> str:
    """Build a rosbridge ``/odom`` publish frame (JSON text) from integrator values.

    Wall-clock stamping and the nav_msgs/Odometry layout live here (transport
    side); the integrator in odometry.py stays time- and format-agnostic so
    smabo-brain-ros can reuse it with ROS time + tf instead.
    """
    now = time.time()
    qz = math.sin(r["theta"] / 2.0)
    qw = math.cos(r["theta"] / 2.0)
    return json.dumps({
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "header": {
                "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                "frame_id": r["odom_frame"],
            },
            "child_frame_id": r["base_frame"],
            "pose": {
                "pose": {
                    "position":    {"x": r["x"], "y": r["y"], "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
                "covariance": build_pose_covariance(r["cov"]),
            },
            "twist": {
                "twist": {
                    "linear":  {"x": r["vx"], "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": r["wz"]},
                },
                "covariance": build_twist_covariance(r["cov"]),
            },
        },
    })


def _strip_prefix(topic: str, prefix: str) -> str:
    """publish トピックから送信元 prefix（例 '/web'）を取り除く。

    クライアントは送信元を示す prefix 付きで配信し、brain はそれを剥がして
    canonical なトピック名で再配信する。prefix が付いていなければそのまま返す。
    """
    if topic.startswith(prefix + "/"):
        return topic[len(prefix):]
    return topic


def _strip_frame_topic(text: str, prefix: str) -> str:
    """raw JSON テキストが publish フレームなら topic から prefix を剥がして返す。

    パース不能・publish 以外・prefix 無しの場合は元の text をそのまま返す。
    """
    try:
        frame = json.loads(text)
    except Exception:
        return text
    if isinstance(frame, dict) and frame.get("op") == "publish":
        topic = frame.get("topic")
        if isinstance(topic, str):
            stripped = _strip_prefix(topic, prefix)
            if stripped != topic:
                frame["topic"] = stripped
                return json.dumps(frame)
    return text


async def _broadcast(targets: set[web.WebSocketResponse], text: str) -> None:
    dead: set[web.WebSocketResponse] = set()
    for ws in targets:
        try:
            await ws.send_str(text)
        except Exception:
            dead.add(ws)
    targets -= dead


async def _app_ws(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    _app_clients.add(ws)
    log.info("app connected (total=%d)", len(_app_clients))
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                await _broadcast(_ui_clients, _strip_frame_topic(msg.data, "/app"))
            elif msg.type == aiohttp.WSMsgType.ERROR:
                log.warning("app ws error: %s", ws.exception())
    finally:
        _app_clients.discard(ws)
        log.info("app disconnected (total=%d)", len(_app_clients))
    return ws


async def _ui_ws(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    _ui_clients.add(ws)
    log.info("UI connected (total=%d)", len(_ui_clients))
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    frame = json.loads(msg.data)
                except Exception:
                    frame = None

                if not isinstance(frame, dict):
                    continue

                op = frame.get("op")

                if op == "publish":
                    topic = _strip_prefix(frame.get("topic", ""), "/web")
                    frame["topic"] = topic
                    data = json.dumps(frame)
                    if topic in _APP_TOPICS:
                        await _broadcast(_app_clients, data)
                    else:
                        await _broadcast(_esp32_clients, data)
                else:
                    await _broadcast(_esp32_clients, msg.data)

            elif msg.type == aiohttp.WSMsgType.ERROR:
                log.warning("UI ws error: %s", ws.exception())
    finally:
        _ui_clients.discard(ws)
        log.info("UI disconnected (total=%d)", len(_ui_clients))
    return ws


async def _esp32_ws(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    _esp32_clients.add(ws)
    log.info("ESP32 connected (total=%d)", len(_esp32_clients))
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                await _on_esp32_message(msg.data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                log.warning("ESP32 ws error: %s", ws.exception())
    finally:
        _esp32_clients.discard(ws)
        log.info("ESP32 disconnected (total=%d)", len(_esp32_clients))
    return ws


async def _on_esp32_message(text: str) -> None:
    try:
        m = json.loads(text)
    except Exception:
        await _broadcast(_ui_clients, text)
        return

    op = m.get("op")

    if op == "publish":
        topic = _strip_prefix(m.get("topic", ""), "/esp32")
        m["topic"] = topic

        if topic == "/wheel_vel":
            body = m.get("msg") or {}
            r = _odom.integrate(
                v_left  = body.get("left",  0.0),
                v_right = body.get("right", 0.0),
                dt      = body.get("dt",    0.0),
            )
            if r is not None:
                await _broadcast(_ui_clients, _odom_frame(r))
            return

        # その他の publish は canonical なトピック名で web に再配信
        await _broadcast(_ui_clients, json.dumps(m))
        return

    if op == "set_config" and m.get("config"):
        # config は web ↔ esp32 の REST 直通で管理される。esp32 はここに
        # 全 config スナップショットを push してくるので、brain は自身の
        # オドメトリ積分（車輪ジオメトリ・共分散・frame 名）の同期にのみ使う。
        _odom.update_config(m["config"])
        return

    await _broadcast(_ui_clients, text)


def create_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/",      _app_ws)
    app.router.add_get("/ui",    _ui_ws)
    app.router.add_get("/esp32", _esp32_ws)
    return app
