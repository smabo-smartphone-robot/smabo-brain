"""smabo-brain の WebSocket サーバー: 受信 → 役割ごとの処理 → 振り分け。

接続口は ``ws://<host>:9090/`` の1つだけで、smabo-brain-ros（rosbridge）と同じ
手順でつなぐ。どのクライアントかは最初のフレームで判別する（clients.py）。
つないだクライアントには /brain/info を送り、クライアントの /hello で
プロトコル番号を確かめる（hello.py）。

振り分け（送り主の名前 /web /app /esp32 を外して配信）:
  web   → /speech/say, /expression, /look_at は app へ。/vision/config と
          /webrtc/* は brain が受け取る。それ以外は esp32 へ。
  app   → /webrtc/offer, /webrtc/app_ice, /speech/audio は brain が受け取る。
          それ以外は web へ。
  esp32 → /wheel_vel は積分して /odom を web へ。/config は brain も受け取り
          （オドメトリ用）、変更の知らせとして web へも流す。それ以外は web へ。

設定は機器の REST（ESP32 は :80、スマホは :8081 の GET/POST /config）にある。
web は brain の /devices/{esp32,app}/… に送り、brain が機器へ転送する（gateway.py）。
brain は場所（/config_url, /phone/config_url）を覚え、変更の知らせ
（/config, /phone/config）を web へ中継する。

認証（pairing.py）: つないだら最初のフレームで token を見せる（{"op":"auth"}）か、
招待コードでペアリングする（{"op":"pair"}）。役割は token の控えで決まり、
接頭辞の違うトピックは捨てる。--no-auth（pairings=None）のときは従来どおり、
最初のフレームの接頭辞で役割を判別し、web は機器の REST へ直接つなぐ。
"""

import asyncio
import json
import logging

import aiohttp
from aiohttp import web

from . import frames, hello
from .clients import HOUSEKEEPING_OPS, Clients, role_of_frame
from .gateway import Gateway, cors_middleware
from .odometry import Odometry
from .pairing import Pairing, PairingError, Pairings
from .speech import Speech
from .topics import APP_TOPICS, ROS_ONLY_TOPICS, SOURCE_PREFIXES, strip_prefix
from .vision_pipeline import VisionPipeline
from .webrtc_hub import WebRtcHub

log = logging.getLogger(__name__)

AUTH_TIMEOUT_S = 5
CLOSE_UNAUTHORIZED = 4001
# Topics a device publishes to say where its REST API is (kept for the relay)
_URL_TOPICS = {"esp32": "/config_url", "app": "/phone/config_url"}


class Brain:
    def __init__(self, vision_config: dict | None = None,
                 vision_config_path: str | None = None,
                 stt_config: dict | None = None,
                 pairings: Pairings | None = None) -> None:
        self.clients = Clients()
        # None = --no-auth. Otherwise every connection shows a token first.
        self.pairings = pairings
        self.conn_pairing: dict[web.WebSocketResponse, Pairing] = {}
        # Where each device's REST API is: role → (url, connection that announced it)
        self.device_urls: dict[str, tuple[str, web.WebSocketResponse]] = {}
        # /hello of each connected client (for the protocol check and new webs)
        self.hellos: dict[web.WebSocketResponse, tuple[str, dict]] = {}
        self.odom = Odometry()
        self.vision = VisionPipeline(self.clients.send, vision_config, vision_config_path)
        self.speech = Speech(self.clients.send, stt_config)
        # WebRTC: app のカメラ映像を受け取り（画像処理へ）、プレビューを開いた web へ中継する。
        self.hub = WebRtcHub(
            process_frame=self.vision.process_frame,
            send_app=self._hub_send_app,
            send_web=self._hub_send_web,
        )
        self.hub.set_fps(self.vision.config.capture_fps)
        if not self.hub.available:
            log.warning("aiortc が無いため WebRTC カメラ/vision は無効です。"
                        "pip install -r requirements.txt を実行してください。")

    def _info_frame(self) -> str:
        return hello.info_frame(auth=self.pairings is not None)

    # ── pairing and the device relay ──────────────────────────────────
    def online_pairing_ids(self) -> set[str]:
        return {p.id for p in self.conn_pairing.values()}

    async def disconnect_pairing(self, pairing_id: str) -> None:
        """Close the connections of a pairing that was just removed."""
        for ws, p in list(self.conn_pairing.items()):
            if p.id == pairing_id:
                await ws.close(code=CLOSE_UNAUTHORIZED, message=b"unpaired")

    def device_target(self, role: str) -> tuple[str, str] | None:
        """(REST base URL, relay key) of the connected device of ``role``, or None."""
        entry = self.device_urls.get(role)
        if entry is None:
            return None
        url, ws = entry
        if ws.closed:
            return None
        p = self.conn_pairing.get(ws)
        return url, p.relay_key if p else ""

    async def _refuse(self, ws: web.WebSocketResponse, op: str, reason: str, message: str) -> None:
        await self._send(ws, json.dumps({"op": op, "ok": False, "reason": reason, "message": message}))
        await ws.close(code=CLOSE_UNAUTHORIZED, message=reason.encode()[:120])

    async def _authenticate(self, ws: web.WebSocketResponse, remote: str | None) -> Pairing | None:
        """Read the first frame: a token ("auth") or an invite code ("pair")."""
        assert self.pairings is not None
        try:
            msg = await ws.receive(timeout=AUTH_TIMEOUT_S)
        except asyncio.TimeoutError:
            log.warning("%s sent nothing; an old client without pairing?", remote)
            await self._refuse(ws, "auth", "timeout", "Send a token or an invite code first")
            return None
        if msg.type != aiohttp.WSMsgType.TEXT:
            return None
        try:
            frame = json.loads(msg.data)
        except ValueError:
            frame = None
        op = frame.get("op") if isinstance(frame, dict) else None
        if op == "auth":
            p = self.pairings.check(frame.get("token"))
            if p is None:
                if frame.get("token"):
                    log.warning("%s showed an unknown token; it needs to pair again", remote)
                else:
                    log.info("%s is not paired yet", remote)
                await self._refuse(ws, "auth", "unauthorized", "Unknown token. Pair again")
                return None
            self.pairings.seen(p)
            await self._send(ws, json.dumps({"op": "auth", "ok": True, "role": p.role,
                                             "name": p.name, "id": p.id}))
            return p
        if op == "pair":
            role = frame.get("role")
            try:
                done = self.pairings.pair(str(frame.get("code", "")), name=str(frame.get("name") or ""),
                                          role=role if isinstance(role, str) else None)
            except PairingError as e:
                log.warning("%s: pairing refused (%s)", remote, e.reason)
                await self._refuse(ws, "pair", e.reason, str(e))
                return None
            p = done.pairing
            self.pairings.seen(p)
            await self._send(ws, json.dumps({"op": "pair", "ok": True, "role": p.role, "name": p.name,
                                             "id": p.id, "token": done.token, "relay_key": done.relay_key}))
            await self.clients.send("web", json.dumps(
                {"op": "notice", "event": "paired", "source": p.role, "name": p.name, "id": p.id}))
            return p
        log.warning("%s did not authenticate (first op %r); is it paired?", remote, op)
        await self._refuse(ws, "auth", "unauthorized", "This brain needs pairing. Send a token first")
        return None

    async def _hub_send_app(self, msg: dict) -> None:
        await self.clients.send("app", json.dumps(msg))

    @staticmethod
    async def _hub_send_web(ws: web.WebSocketResponse, msg: dict) -> None:
        try:
            await ws.send_str(json.dumps(msg))
        except Exception:
            pass

    # ── 接続の出入り ───────────────────────────────────────────────────
    async def _join(self, ws: web.WebSocketResponse, role: str) -> None:
        self.clients.add(ws, role)
        if role != "web":
            await self._send(ws, self._info_frame())
            await self.clients.notify_web(role, "connected")
            return
        # web には現在の画像処理設定と接続状況を送り、起動直後から正しく表示させる。
        # プレビュー映像は web が /webrtc/preview {on:true} を送ったときだけ流す。
        await self._send(ws, self.vision.config_frame())
        await self._send(ws, self._info_frame())
        for text in self.clients.snapshot_notices():
            await self._send(ws, text)
        for r, h in self.hellos.values():
            if r != "web":
                await self._send(ws, hello.notice(r, h))

    @staticmethod
    async def _send(ws: web.WebSocketResponse, text: str) -> None:
        try:
            await ws.send_str(text)
        except Exception:
            pass

    async def _on_hello(self, ws: web.WebSocketResponse, role: str, frame: dict) -> None:
        """A client's /hello: remember it, answer with /brain/info, warn on a mismatch."""
        h = hello.parse(frame)
        self.hellos[ws] = (role, h)
        if hello.compatible(h):
            log.info("%s hello: %s %s (protocol %s)", role, h["client"], h["version"], h["protocol"])
        else:
            log.warning("%s (%s %s) はプロトコル %s です。brain は %s なので、"
                        "一部の機能が正しく動かない可能性があります",
                        role, h["client"], h["version"], h["protocol"], hello.info()["protocol"])
        await self._send(ws, self._info_frame())
        if role != "web":
            await self.clients.send("web", hello.notice(role, h))

    async def _leave(self, ws: web.WebSocketResponse, role: str) -> None:
        self.clients.remove(ws, role)
        self.hellos.pop(ws, None)
        self.conn_pairing.pop(ws, None)
        if role in self.device_urls and self.device_urls[role][1] is ws:
            del self.device_urls[role]
        if role == "web":
            await self.hub.stop_web_preview(ws)
            return
        if role == "app" and not self.clients.count("app"):
            await self.hub.close_app()   # 映像源が切れたら WebRTC を畳む
        await self.clients.notify_web(role, "disconnected")

    async def handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        # heartbeat: 5 s ごとに ping。電源を切った ESP32 も ~10 s で切断を検知できる。
        ws = web.WebSocketResponse(heartbeat=5)
        await ws.prepare(request)
        role: str | None = None
        if self.pairings is not None:
            pairing = await self._authenticate(ws, request.remote)
            if pairing is None:
                return ws
            self.conn_pairing[ws] = pairing
            role = pairing.role
            await self._join(ws, role)
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.ERROR:
                    log.warning("ws error: %s", ws.exception())
                    continue
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                try:
                    frame = json.loads(msg.data)
                except ValueError:
                    continue
                if not isinstance(frame, dict):
                    continue
                if role is None and frame.get("op") in ("auth", "pair"):
                    # --no-auth: say so at once, so a client need not wait for a reply
                    await self._send(ws, json.dumps({"op": frame["op"], "ok": frame["op"] == "auth",
                                                     "auth": False, "reason": "no_auth"}))
                    continue
                if role is None:
                    # 役割が分かるまでのフレーム（roslibjs の call_service など）は捨てる
                    role = role_of_frame(frame)
                    if role is None:
                        continue
                    await self._join(ws, role)
                if frame.get("op") in HOUSEKEEPING_OPS:
                    continue
                await self._dispatch(ws, role, frame)
        finally:
            if role is not None:
                await self._leave(ws, role)
        return ws

    async def _dispatch(self, ws: web.WebSocketResponse, role: str, frame: dict) -> None:
        if frame.get("op") != "publish":
            # ESP32 の notice などはそのまま web へ見せる。web からの publish 以外は使わない。
            if role != "web":
                await self.clients.send("web", json.dumps(frame))
            return
        topic = frame.get("topic")
        if not isinstance(topic, str):
            return
        prefix = SOURCE_PREFIXES[role]
        if self.pairings is not None and not topic.startswith(prefix + "/"):
            # The token decides the role; a browser cannot speak as the ESP32
            log.warning("%s %r published %s; dropped", role, self.conn_pairing[ws].name, topic)
            return
        frame["topic"] = strip_prefix(topic, prefix)
        if frame["topic"] == _URL_TOPICS.get(role):
            url = (frame.get("msg") or {}).get("data")
            if isinstance(url, str) and url.startswith(("http://", "https://")):
                self.device_urls[role] = (url, ws)
        if frame["topic"] == "/hello":
            await self._on_hello(ws, role, frame)
        elif role == "web":
            await self._from_web(ws, frame)
        elif role == "app":
            await self._from_app(frame)
        else:
            await self._from_esp32(frame)

    # ── 役割ごとの処理（frame["topic"] は送り主の名前を外した正式名） ─────────
    async def _from_web(self, ws: web.WebSocketResponse, frame: dict) -> None:
        topic = frame["topic"]
        if topic == "/vision/config":
            self.vision.apply_config(frame.get("msg") or {})
            self.hub.set_fps(self.vision.config.capture_fps)
            # 全 web に正規化済みの設定を返して同期する。
            await self.clients.send("web", self.vision.config_frame())
        elif topic == "/webrtc/preview":
            if frames.data_json(frame).get("on"):
                await self.hub.start_web_preview(ws)
            else:
                await self.hub.stop_web_preview(ws)
        elif topic == "/webrtc/web_answer":
            await self.hub.handle_web_answer(ws, frames.data_json(frame))
        elif topic == "/webrtc/web_ice":
            await self.hub.add_web_ice(ws, frames.data_json(frame))
        elif topic in ROS_ONLY_TOPICS:
            pass        # navigation exists only in smabo-brain-ros
        else:
            await self.clients.send("app" if topic in APP_TOPICS else "esp32", json.dumps(frame))

    async def _from_app(self, frame: dict) -> None:
        topic = frame["topic"]
        if topic == "/webrtc/offer":
            await self.hub.handle_app_offer(frames.data_json(frame))
        elif topic == "/webrtc/app_ice":
            await self.hub.add_app_ice(frames.data_json(frame))
        elif topic == "/speech/audio":
            asyncio.create_task(self.speech.handle_audio(frame.get("msg") or {}))
        else:
            await self.clients.send("web", json.dumps(frame))

    async def _from_esp32(self, frame: dict) -> None:
        topic = frame["topic"]
        msg = frame.get("msg") or {}
        if topic == "/wheel_vel":
            r = self.odom.integrate(v_left=msg.get("left", 0.0),
                                    v_right=msg.get("right", 0.0),
                                    dt=msg.get("dt", 0.0))
            if r is not None:
                await self.clients.send("web", frames.odom(r))
        elif topic == "/config":
            # 車輪の寸法・共分散・frame 名（オドメトリ用）と実在関節名（画像処理の
            # 首振り用）の同期に使う。web へは「設定が変わった」知らせとして流す
            # （web は REST で読み直す。config 自体は web と ESP32 が直接やりとり）。
            cfg = frames.data_json(frame)
            if cfg:
                self.odom.update_config(cfg)
                self.vision.update_joints(cfg)
            await self.clients.send("web", json.dumps(frame))
        else:
            if topic == "/joint_states":
                self.vision.update_joint_angles(msg)
            await self.clients.send("web", json.dumps(frame))


BRAIN_KEY = web.AppKey("brain", Brain)


def create_app(vision_config: dict | None = None,
               vision_config_path: str | None = None,
               stt_config: dict | None = None,
               pairings: Pairings | None = None) -> web.Application:
    """smabo-brain の aiohttp アプリを作る。

    vision_config: 画像処理設定の起動時初期値（VisionConfig.to_dict() 形状）。
    None なら組み込み既定（mode=off）。stt_config: engine / model / language。
    pairings: ペアリングの控え。None なら認証なし（--no-auth）。
    """
    brain = Brain(vision_config, vision_config_path, stt_config, pairings)
    gateway = Gateway(brain)
    app = web.Application(middlewares=[cors_middleware])
    app[BRAIN_KEY] = brain
    app.router.add_get("/", brain.handle_ws)
    app.router.add_routes(gateway.routes())

    async def close_gateway(_app: web.Application) -> None:
        await gateway.close()
    app.on_cleanup.append(close_gateway)

    if pairings is not None:
        async def bootstrap_codes(_app: web.Application):
            # While no browser is paired, keep a fresh code in the log
            # (each one lasts 5 minutes).
            async def loop() -> None:
                while True:
                    pairings.ensure_bootstrap()
                    await asyncio.sleep(15)
            task = asyncio.create_task(loop())
            yield
            task.cancel()
        app.cleanup_ctx.append(bootstrap_codes)
    return app
