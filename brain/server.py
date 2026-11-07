"""smabo-brain の WebSocket サーバー: 受信 → 役割ごとの処理 → 振り分け。

接続口は ``ws://<host>:9090/`` の1つだけで、smabo-brain-ros（rosbridge）と同じ
手順でつなぐ。どのクライアントかは最初のフレームで判別する（clients.py）。

振り分け（送り主の名前 /web /app /esp32 を外して配信）:
  web   → /speech/say, /expression, /look_at は app へ。/vision/config と
          /webrtc/* は brain が受け取る。それ以外は esp32 へ。
  app   → /webrtc/offer, /webrtc/app_ice, /speech/audio は brain が受け取る。
          それ以外は web へ。
  esp32 → /wheel_vel は積分して /odom を web へ。/config は brain が受け取る。
          それ以外は web へ。
"""

import asyncio
import json
import logging

import aiohttp
from aiohttp import web

from . import frames
from .clients import HOUSEKEEPING_OPS, Clients, role_of_frame
from .odometry import Odometry
from .speech import Speech
from .topics import APP_TOPICS, SOURCE_PREFIXES, strip_prefix
from .vision_pipeline import VisionPipeline
from .webrtc_hub import WebRtcHub

log = logging.getLogger(__name__)


class Brain:
    def __init__(self, vision_config: dict | None = None,
                 vision_config_path: str | None = None,
                 stt_config: dict | None = None) -> None:
        self.clients = Clients()
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
            await self.clients.notify_web(role, "connected")
            return
        # web には現在の画像処理設定と接続状況を送り、起動直後から正しく表示させる。
        # プレビュー映像は web が /webrtc/preview {on:true} を送ったときだけ流す。
        try:
            await ws.send_str(self.vision.config_frame())
            for text in self.clients.snapshot_notices():
                await ws.send_str(text)
        except Exception:
            pass

    async def _leave(self, ws: web.WebSocketResponse, role: str) -> None:
        self.clients.remove(ws, role)
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
        frame["topic"] = strip_prefix(topic, SOURCE_PREFIXES[role])
        if role == "web":
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
            # 首振り用）の同期にだけ使い、web へは流さない。config 自体は web と
            # ESP32 が REST で直接やりとりする。
            cfg = frames.data_json(frame)
            if cfg:
                self.odom.update_config(cfg)
                self.vision.update_joints(cfg)
        else:
            if topic == "/joint_states":
                self.vision.update_joint_angles(msg)
            await self.clients.send("web", json.dumps(frame))


BRAIN_KEY = web.AppKey("brain", Brain)


def create_app(vision_config: dict | None = None,
               vision_config_path: str | None = None,
               stt_config: dict | None = None) -> web.Application:
    """smabo-brain の aiohttp アプリを作る。

    vision_config: 画像処理設定の起動時初期値（VisionConfig.to_dict() 形状）。
    None なら組み込み既定（mode=off）。stt_config: engine / model / language。
    """
    brain = Brain(vision_config, vision_config_path, stt_config)
    app = web.Application()
    app[BRAIN_KEY] = brain
    app.router.add_get("/", brain.handle_ws)
    return app
