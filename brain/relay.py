import asyncio
import json
import logging
import math
import time
import aiohttp
from aiohttp import web

from . import vision
from .odometry import Odometry, build_pose_covariance, build_twist_covariance
from .stt import SttEngine, wav_to_samples
from .topics import APP_TOPICS as _APP_TOPICS, SOURCE_PREFIXES as _SOURCE_PREFIXES, strip_prefix as _strip_prefix
from .vision import VisionConfig, merge_config
from .webrtc_hub import WebRtcHub

log = logging.getLogger(__name__)

_app_clients:   set[web.WebSocketResponse] = set()
_ui_clients:    set[web.WebSocketResponse] = set()
_esp32_clients: set[web.WebSocketResponse] = set()

# 送信元 prefix / app 宛トピックの定義は brain.topics に集約し、smabo-brain-ros の
# relay launch（topic_tools remap）も同じ定義を import して単一の正とする。

_odom = Odometry()

# 画像処理の現在設定。起動時に create_app(vision_config=...) で初期値を入れ、
# 実行中は /vision/config（std_msgs/String の data に JSON）で部分上書きする。
# brain 自身がこの状態を保持し、新規 UI 接続時にスナップショットを送って同期する。
_vision = VisionConfig()


def _vision_config_frame() -> str:
    """現在の _vision を /vision/config の publish フレーム（JSON テキスト）にする。"""
    return json.dumps({
        "op": "publish",
        "topic": "/vision/config",
        "msg": {"data": json.dumps(_vision.to_dict())},
    })


# 画像処理パイプライン用の状態。CV は重いので executor で実行し、処理中は後続
# フレームを間引く（busy フラグ）。インターバルは設けず、処理が終わり次第
# 次フレームを受け付ける（実効 fps = 1 / 検出時間 で自然に決まる）。
_vision_config_path: str | None = None   # vision_config.json の保存先パス
_pan_pid       = vision.PIDState()       # pan 軸 PID 状態
_tilt_pid      = vision.PIDState()       # tilt 軸 PID 状態
_drive_ang_pid = vision.PIDState()       # drive follow: 角速度 PID 状態
_drive_lin_pid = vision.PIDState()       # drive follow: 直進速度 PID 状態
_servo_lost_count = 0                    # gaze/servo: ターゲット連続ロストフレーム数
_drive_lost_count = 0                    # drive: ターゲット連続ロストフレーム数
_vision_busy = False
_vision_last_spoken: str | None = None   # 発話の重複抑制（brain 側 dedup）
_esp32_joints: set[str] = set()          # set_config で得た実在関節名（servo policy 用）
_esp32_joint_angles: dict[str, float] = {}  # /joint_states から取得した現在角度（rad）
_warned_no_cv2 = False                    # OpenCV 未導入の警告は一度だけ出す
_vision_last_count = -1                   # 検出件数の変化時だけログを出す（診断用）


def _pub(topic: str, msg: dict) -> str:
    """rosbridge publish フレーム（JSON テキスト）を作る。"""
    return json.dumps({"op": "publish", "topic": topic, "msg": msg})


# WebRTC ハブ: brain が app からの映像を受信（vision 用にフレーム取り出し）し、
# プレビュー ON の web クライアントへ中継する。process_vision_frame 定義後に
# create_app で生成する（コールバックが _broadcast を実行時参照するため順序を満たす）。
async def _hub_send_app(msg: dict) -> None:
    await _broadcast(_app_clients, json.dumps(msg))


async def _hub_send_web(ws: web.WebSocketResponse, msg: dict) -> None:
    try:
        await ws.send_str(json.dumps(msg))
    except Exception:
        pass


_hub: WebRtcHub | None = None

# Speech-to-text engine. smabo-app sends a recorded utterance on /speech/audio;
# the brain transcribes it and republishes the text on /speech/recognized.
# Created in create_app from --stt-* options. None until then.
_stt: SttEngine | None = None
_warned_no_stt = False


def _transcribe_b64_wav(b64: str) -> str:
    """Decode a base64 WAV (16-bit PCM) and run STT. Returns '' on any failure.

    Runs on a worker thread (STT is CPU-heavy). WAV parsing lives in brain.stt
    so smabo-brain-ros reuses it.
    """
    import base64

    try:
        raw = base64.b64decode(b64)
    except Exception:
        return ""
    samples, sr = wav_to_samples(raw)
    if samples is None or samples.size == 0 or _stt is None:
        return ""
    return _stt.transcribe(samples, sr)


async def _handle_speech_audio(body: dict) -> None:
    """Receive a recorded utterance (/speech/audio) → STT → /speech/recognized."""
    global _warned_no_stt
    if _stt is None or not _stt.available:
        if not _warned_no_stt:
            _warned_no_stt = True
            log.warning("/speech/audio を受信しましたが STT が無効です。"
                        "vosk（または faster-whisper）を導入してください。")
        return
    b64 = (body or {}).get("data")
    if not b64:
        return
    loop = asyncio.get_running_loop()
    text = await loop.run_in_executor(None, _transcribe_b64_wav, b64)
    if text:
        log.info("STT: %r", text)
        await _broadcast(_ui_clients, _pub("/speech/recognized", {"data": text}))


def _stamp() -> dict:
    now = time.time()
    return {"sec": int(now), "nanosec": int((now % 1) * 1e9)}


def _update_esp32_joints(config: dict) -> None:
    """set_config スナップショットから実在関節名を取り出す（servo policy が参照）。"""
    global _esp32_joints
    try:
        joints = (config.get("servos") or {}).get("joints") or {}
        if isinstance(joints, dict):
            _esp32_joints = set(joints.keys())
    except Exception:
        pass


def _run_vision_sync(bgr, cfg: VisionConfig, joints_available, joint_angles=None,
                     pan_pid=None, tilt_pid=None,
                     drive_ang_pid=None, drive_lin_pid=None) -> dict | None:
    """CV ＋ policy（純ロジック）をワーカースレッドで実行し、送信用フレームを返す。

    bgr は BGR の ndarray（WebRTC ハブが取り出したフレーム）。run_detector は
    cv2 が無い環境では空を返すので安全に no-op になる。
    """
    if bgr is None:
        return None
    orig_h, orig_w = bgr.shape[:2]
    # Downscale before detection — CV operations are O(W×H) so a 1080p→640p
    # resize cuts detection time ~6-10x with negligible accuracy loss.
    detect_bgr, inv_scale = vision.resize_for_detection(bgr)
    detections, strings, _ = vision.run_detector(detect_bgr, cfg)
    # Scale bbox coordinates back to original (display) resolution.
    if inv_scale != 1.0:
        for d in detections:
            d["cx"] *= inv_scale
            d["cy"] *= inv_scale
            d["w"]  *= inv_scale
            d["h"]  *= inv_scale
    img_wh = (orig_w, orig_h)
    stamp = _stamp()
    det_msg = vision.to_detection2darray(detections, "camera", stamp, img_wh)
    target = vision.pick_target(detections, cfg.target_marker_id)
    look_at = servo = cmd_vel = None
    if target is not None:
        direction = vision.bbox_to_direction(
            target["cx"], target["cy"], img_wh[0], img_wh[1], cfg.hfov_deg)
        if cfg.do_look_at:
            look_at = vision.direction_to_look_at(direction, "base_link", stamp)
        if cfg.do_servo:
            servo = vision.direction_to_servo(
                direction, cfg, stamp, joints_available, joint_angles, pan_pid, tilt_pid)
    if cfg.do_drive:
        # ターゲット無しでも停止 twist を出して暴走を防ぐ
        cmd_vel = vision.to_cmd_vel(target, img_wh, cfg, drive_ang_pid, drive_lin_pid)
    return {
        "img_wh": img_wh,
        "detections": det_msg, "strings": strings,
        "look_at": look_at, "servo": servo, "cmd_vel": cmd_vel,
        "target_found": target is not None,
    }


async def process_vision_frame(bgr) -> None:
    """WebRTC ハブが取り出した BGR フレームを検出 → 配信 → policy 指令する。

    宛先ルーティング:
      - /vision/detections, /vision/markers → web（UI）
      - /look_at, /speech/say               → app（APP_TOPICS）
      - /servo/command, /cmd_vel            → esp32
    """
    global _vision_busy, _vision_last_spoken
    global _warned_no_cv2, _vision_last_count
    global _servo_lost_count, _drive_lost_count
    cfg = _vision
    if not cfg.enabled or cfg.mode == "off":
        return
    if not vision.HAVE_CV2:
        if not _warned_no_cv2:
            _warned_no_cv2 = True
            log.warning("vision is enabled but OpenCV/numpy are not installed; "
                        "detection is disabled. Run: pip install -r requirements.txt")
        return
    if _vision_busy:
        return
    _vision_busy = True
    try:
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(
            None, _run_vision_sync, bgr, cfg, _esp32_joints or None, dict(_esp32_joint_angles),
            _pan_pid, _tilt_pid, _drive_ang_pid, _drive_lin_pid)
    except Exception:
        log.exception("vision processing failed")
        result = None
    finally:
        _vision_busy = False
    if not result:
        return

    # ── tolerance 付きロスト処理（PID リセット判定） ───────────────────────────
    # target_found が False のフレームが tolerance を超えて連続したときにのみ
    # PID をリセットする。これにより短時間のドロップアウトで積分が失われない。
    if result["target_found"]:
        _servo_lost_count = 0
        _drive_lost_count = 0
    else:
        _servo_lost_count += 1
        _drive_lost_count += 1
        if _servo_lost_count > cfg.servo_lost_tolerance:
            _pan_pid.reset()
            _tilt_pid.reset()
        if _drive_lost_count > cfg.drive_lost_tolerance:
            _drive_ang_pid.reset()
            _drive_lin_pid.reset()

    # 検出件数が変わったときだけ、解像度つきでログ。
    #   ・"vision[aruco] 640x480: 0 detection(s)" が出続ける → 検出は走るが0件
    #     （= 辞書ミスマッチ / マーカーが小さい・ぼやけ）→ Vision タブで辞書を合わせる
    #   ・このログが一切出ない → 検出に到達していない（busy/間引き/設定 off 等）
    count = len(result["detections"]["detections"])
    if count != _vision_last_count:
        _vision_last_count = count
        w, h = result["img_wh"]
        log.info("vision[%s] %dx%d: %d detection(s)", cfg.mode, w, h, count)

    await _broadcast(_ui_clients, _pub("/vision/detections", result["detections"]))
    strings = result["strings"]
    if strings:
        await _broadcast(_ui_clients, _pub("/vision/markers", {"data": ",".join(strings)}))
    if result["look_at"] is not None:
        await _broadcast(_app_clients, _pub("/look_at", result["look_at"]))
    if result["servo"] is not None:
        await _broadcast(_esp32_clients, _pub("/servo/command", result["servo"]))
    if result["cmd_vel"] is not None:
        await _broadcast(_esp32_clients, _pub("/cmd_vel", result["cmd_vel"]))
    if cfg.speak and strings:
        joined = ",".join(strings)
        if joined != _vision_last_spoken:
            _vision_last_spoken = joined
            await _broadcast(_app_clients, _pub("/speech/say", {"data": joined}))


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


def _apply_vision_config(body: dict) -> None:
    """/vision/config の msg（std_msgs/String）から部分設定を取り出し _vision を更新する。

    data は JSON 文字列を想定するが、rosbridge クライアントによっては既に
    dict の場合もあるため両対応。部分 patch を現在値に deep-merge する。
    """
    global _vision
    raw = body.get("data")
    if isinstance(raw, str):
        try:
            patch = json.loads(raw)
        except Exception:
            log.warning("vision config: data is not valid JSON; ignored")
            return
    elif isinstance(raw, dict):
        patch = raw
    else:
        return
    if not isinstance(patch, dict):
        return
    _vision = VisionConfig(merge_config(_vision.to_dict(), patch))
    if _vision_config_path:
        try:
            with open(_vision_config_path, "w", encoding="utf-8") as f:
                json.dump(_vision.to_dict(), f, ensure_ascii=False, indent=2)
        except Exception as e:
            log.warning("vision config の保存に失敗しました (%s)", e)


async def _broadcast(targets: set[web.WebSocketResponse], text: str) -> None:
    dead: set[web.WebSocketResponse] = set()
    for ws in targets:
        try:
            await ws.send_str(text)
        except Exception:
            dead.add(ws)
    targets -= dead


def _notice(source: str, event: str, total: int) -> str:
    return json.dumps({"op": "notice", "source": source, "event": event, "total": total})


# ── クライアントの種類（role）と接続の出入り ─────────────────────────────
#
# 接続口は 2 系統ある:
#   - 種類ごとのパス: "/ui"（web）, "/esp32"（ESP32）。パスで role が決まる。
#   - 共通の "/": rosbridge_suite（smabo-brain-ros）と同じ接続口。どのクライアント
#     も rosbridge と同じ手順（advertise / subscribe / publish）でつなげば、相手が
#     smabo-brain でも smabo-brain-ros でも設定を変えずに動く。role は最初に届いた
#     フレームから判別する（_role_of_frame）。

_ROLE_CLIENTS: dict[str, set[web.WebSocketResponse]] = {
    "app": _app_clients, "web": _ui_clients, "esp32": _esp32_clients,
}

# "/" で、subscribe しただけで role が分かるトピック（それを受け取るのはその role だけ）。
# smabo-app は publish より先に /expression を subscribe するのでこれで判別できる。
_SUBSCRIBE_ROLE: dict[str, str] = {
    "/cmd_vel": "esp32", "/servo/command": "esp32",
    **{t: "app" for t in _APP_TOPICS},
}

# rosbridge の接続管理 op。smabo-brain は購読に関係なく role ごとに配信するので、
# role の判別に使うだけで中継はしない。
_HOUSEKEEPING_OPS = frozenset({"advertise", "unadvertise", "subscribe", "unsubscribe"})


def _role_of_frame(frame: dict) -> str | None:
    """"/" に届いたフレームから送り主の role を推定する。分からなければ None。"""
    op = frame.get("op")
    topic = frame.get("topic")
    if not isinstance(topic, str):
        topic = ""
    if op in ("publish", "advertise"):
        for role, prefix in _SOURCE_PREFIXES.items():
            if topic.startswith(prefix + "/"):
                return role
        return None
    if op == "subscribe":
        return _SUBSCRIBE_ROLE.get(topic)
    if op == "set_config":
        return "esp32"
    return None


async def _join(ws: web.WebSocketResponse, role: str) -> None:
    """role の接続を登録し、web に接続通知を出す（web 自身には現在状態を送る）。"""
    clients = _ROLE_CLIENTS[role]
    clients.add(ws)
    if role == "web":
        log.info("UI connected (total=%d)", len(clients))
        # 接続直後に現在の画像処理設定（起動時初期値 or 直近の上書き）を送り、
        # UI が起動時から正しい状態を表示できるようにする。プレビュー映像は web が
        # 明示的に /webrtc/preview {on:true} を送ったときだけ開始する（補助用途なので
        # 既定では映像を流さない）。
        try:
            await ws.send_str(_vision_config_frame())
            # 接続済みクライアントの現在状態をスナップショットとして送る
            if _app_clients:
                await ws.send_str(_notice("app", "connected", len(_app_clients)))
            if _esp32_clients:
                await ws.send_str(_notice("esp32", "connected", len(_esp32_clients)))
        except Exception:
            pass
        return
    log.info("%s connected (total=%d)", "ESP32" if role == "esp32" else role, len(clients))
    await _broadcast(_ui_clients, _notice(role, "connected", len(clients)))


async def _leave(ws: web.WebSocketResponse, role: str) -> None:
    clients = _ROLE_CLIENTS[role]
    clients.discard(ws)
    if role == "web":
        if _hub is not None:
            await _hub.stop_web_preview(ws)
        log.info("UI disconnected (total=%d)", len(clients))
        return
    if role == "app" and not clients and _hub is not None:
        # 映像源（app）が切れたら WebRTC ピアと中継を畳む。
        await _hub.close_app()
    log.info("%s disconnected (total=%d)", "ESP32" if role == "esp32" else role, len(clients))
    await _broadcast(_ui_clients, _notice(role, "disconnected", len(clients)))


async def _dispatch(ws: web.WebSocketResponse, role: str, text: str) -> None:
    if role == "app":
        await _on_app_message(text)
    elif role == "web":
        await _on_ui_message(ws, text)
    else:
        await _on_esp32_message(text)


async def _role_ws(request: web.Request, role: str, heartbeat: float | None = None) -> web.WebSocketResponse:
    """パスで role が決まる接続口（"/ui", "/esp32"）。"""
    ws = web.WebSocketResponse(heartbeat=heartbeat)
    await ws.prepare(request)
    await _join(ws, role)
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                await _dispatch(ws, role, msg.data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                log.warning("%s ws error: %s", role, ws.exception())
    finally:
        await _leave(ws, role)
    return ws


async def _ui_ws(request: web.Request) -> web.WebSocketResponse:
    return await _role_ws(request, "web")


async def _esp32_ws(request: web.Request) -> web.WebSocketResponse:
    # heartbeat: send a WS ping every 5 s; close if no pong within 5 s.
    # Detects a powered-off ESP32 within ~10 s (worst case: just missed a ping).
    # The ESP32 ws_client.py handles opcode 0x9 (ping) and replies with 0xA (pong).
    return await _role_ws(request, "esp32", heartbeat=5)


async def _root_ws(request: web.Request) -> web.WebSocketResponse:
    """rosbridge 互換の共通接続口 "/"。role はフレームの中身から判別する。

    rosbridge クライアント（advertise / subscribe してから publish）はそのまま
    動く。advertise / subscribe 自体は role の判別にだけ使い、配信は従来どおり
    role ごとの宛先ルールで行う。role が分かる前の判別できないフレームは捨てる
    （例: roslibjs の call_service。smabo-brain には ROS サービスが無い）。
    """
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    role: str | None = None
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    frame = json.loads(msg.data)
                except Exception:
                    frame = None
                if not isinstance(frame, dict):
                    continue
                if role is None:
                    role = _role_of_frame(frame)
                    if role is None:
                        continue
                    await _join(ws, role)
                if frame.get("op") in _HOUSEKEEPING_OPS:
                    continue
                await _dispatch(ws, role, msg.data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                log.warning("ws error: %s", ws.exception())
    finally:
        if role is not None:
            await _leave(ws, role)
    return ws


def _frame_data_json(frame: dict) -> dict:
    """publish フレームの msg.data（JSON 文字列 or dict）を dict にして返す。"""
    body = frame.get("msg") or {}
    raw = body.get("data")
    if isinstance(raw, str):
        try:
            return json.loads(raw)
        except Exception:
            return {}
    return raw if isinstance(raw, dict) else {}


async def _on_app_message(text: str) -> None:
    """app → brain。canonical 名にして web へ再配信。WebRTC シグナリングは brain が消費。

    映像はもう WebSocket では届かない。smabo-app は WebRTC で brain にのみ映像を
    送り、brain（webrtc_hub）がフレームを取り出して vision に通す。ここでは
    そのシグナリング（offer/ICE）を捌き、それ以外の publish は web へ中継する。
    """
    try:
        frame = json.loads(text)
    except Exception:
        await _broadcast(_ui_clients, text)
        return

    if isinstance(frame, dict) and frame.get("op") == "publish":
        topic = frame.get("topic")
        if isinstance(topic, str):
            stripped = _strip_prefix(topic, "/app")
            # WebRTC シグナリング（app → brain）。brain が消費し web へは流さない。
            if stripped == "/webrtc/offer":
                if _hub is not None:
                    await _hub.handle_app_offer(_frame_data_json(frame))
                return
            if stripped == "/webrtc/app_ice":
                if _hub is not None:
                    await _hub.add_app_ice(_frame_data_json(frame))
                return
            # 録音音声（app → brain）。brain が STT して /speech/recognized を配信。
            if stripped == "/speech/audio":
                asyncio.create_task(_handle_speech_audio(frame.get("msg") or {}))
                return
            if stripped != topic:
                frame["topic"] = stripped
                forwarded = text.replace(json.dumps(topic), json.dumps(stripped), 1)
            else:
                forwarded = text
            await _broadcast(_ui_clients, forwarded)
            return

    await _broadcast(_ui_clients, text)


async def _on_ui_message(ws: web.WebSocketResponse, text: str) -> None:
    """web → brain。brain が消費するもの以外は宛先デバイス（app / esp32）へ中継。"""
    try:
        frame = json.loads(text)
    except Exception:
        return
    if not isinstance(frame, dict):
        return

    if frame.get("op") != "publish":
        await _broadcast(_esp32_clients, text)
        return

    topic = _strip_prefix(frame.get("topic", ""), "/web")
    frame["topic"] = topic
    if topic == "/vision/config":
        _apply_vision_config(frame.get("msg") or {})
        if _hub is not None:
            _hub.set_fps(_vision.capture_fps)
        # 全 UI に正規化済みスナップショットを返して同期する。
        await _broadcast(_ui_clients, _vision_config_frame())
        return
    # WebRTC プレビュー（web ↔ brain）。brain が中継ピアとして消費。
    if topic == "/webrtc/preview":
        if _hub is not None:
            if _frame_data_json(frame).get("on"):
                await _hub.start_web_preview(ws)
            else:
                await _hub.stop_web_preview(ws)
        return
    if topic == "/webrtc/web_answer":
        if _hub is not None:
            await _hub.handle_web_answer(ws, _frame_data_json(frame))
        return
    if topic == "/webrtc/web_ice":
        if _hub is not None:
            await _hub.add_web_ice(ws, _frame_data_json(frame))
        return
    data = json.dumps(frame)
    if topic in _APP_TOPICS:
        await _broadcast(_app_clients, data)
    else:
        await _broadcast(_esp32_clients, data)


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

        if topic == "/joint_states":
            global _esp32_joint_angles
            body = m.get("msg") or {}
            names = body.get("name") or []
            positions = body.get("position") or []
            _esp32_joint_angles = {n: p for n, p in zip(names, positions)}
            # joint_states は UI にも転送する
            await _broadcast(_ui_clients, json.dumps(m))
            return

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
        # （カメラ＝画像処理の入口は app 経路。_on_app_message を参照）
        if topic == "/config":
            # rosbridge 手順でつなぐ ESP32 / smabo-sim は、set_config op の代わりに
            # config（wifi を除く）を /esp32/config（std_msgs/String の data に JSON）
            # で publish する。web へは流さない。
            cfg = _frame_data_json(m)
            if cfg:
                _odom.update_config(cfg)
                _update_esp32_joints(cfg)
            return

        await _broadcast(_ui_clients, json.dumps(m))
        return

    if op == "set_config" and m.get("config"):
        # config は web ↔ esp32 の REST 直通で管理される。esp32 はここに
        # 全 config スナップショットを push してくるので、brain は自身の
        # オドメトリ積分（車輪ジオメトリ・共分散・frame 名）の同期と、
        # 画像処理 servo policy 用の実在関節名の把握に使う。
        _odom.update_config(m["config"])
        _update_esp32_joints(m["config"])
        return

    await _broadcast(_ui_clients, text)


def create_app(vision_config: dict | None = None,
               vision_config_path: str | None = None,
               stt_config: dict | None = None) -> web.Application:
    """リレーサーバの aiohttp アプリを構築する。

    vision_config: 画像処理設定の起動時初期値（VisionConfig.to_dict() 形状の
    dict）。--vision-config / SMABO_VISION_CONFIG から読んだものを渡す。None の
    場合は VisionConfig の組み込み既定（mode=off, 検出無効）で起動する。

    stt_config: 音声認識の設定（engine / model / language）。None の場合は
    既定（vosk / 日本語）で起動する。
    """
    global _vision, _hub, _stt, _vision_config_path
    _vision_config_path = vision_config_path
    if vision_config is not None:
        _vision = VisionConfig(vision_config)

    # WebRTC ハブを生成（process_vision_frame はこの時点で定義済み）。
    _hub = WebRtcHub(
        process_frame=process_vision_frame,
        send_app=_hub_send_app,
        send_web=_hub_send_web,
    )
    _hub.set_fps(_vision.capture_fps)
    if not _hub.available:
        log.warning("aiortc が無いため WebRTC カメラ/vision は無効です。"
                    "pip install -r requirements.txt を実行してください。")

    # 音声認識エンジン（/speech/audio → /speech/recognized）。
    _stt = SttEngine(**(stt_config or {}))

    app = web.Application()
    app.router.add_get("/",      _root_ws)
    app.router.add_get("/ui",    _ui_ws)
    app.router.add_get("/esp32", _esp32_ws)
    return app
