"""画像処理の実行と状態（smabo-brain の WebSocket 版）。

検出や方向計算・視線/首/走行の方針そのものは共通ロジック ``brain.vision``
にあり、smabo-brain-ros も同じものを使う。ここは smabo-brain 側の
「いつ実行し、結果を誰に送るか」と、そのための状態（設定・PID・ロスト回数）
だけを持つ。

宛先:
  - /vision/detections, /vision/markers → web
  - /look_at, /speech/say               → app
  - /servo/command, /cmd_vel            → esp32
"""

import asyncio
import json
import logging
from typing import Awaitable, Callable

from . import frames, vision
from .vision import VisionConfig, merge_config

log = logging.getLogger(__name__)

# (role, frame text) を送る関数。server が Clients.send を渡す。
Send = Callable[[str, str], Awaitable[None]]


def _detect_and_decide(bgr, cfg: VisionConfig, joints_available, joint_angles,
                       pan_pid, tilt_pid, drive_ang_pid, drive_lin_pid) -> dict | None:
    """CV ＋ policy（純ロジック）をワーカースレッドで実行し、送る内容を返す。

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
    stamp = frames.stamp()
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


class VisionPipeline:
    """画像処理の設定と実行状態。WebRTC ハブからフレームを受けて処理する。

    config: 起動時の設定（VisionConfig.to_dict() 形状。部分指定可）。
    config_path: 設定の保存先。変更のたびに書き出す（None なら保存しない）。
    """

    def __init__(self, send: Send, config: dict | None = None,
                 config_path: str | None = None) -> None:
        self._send = send
        self.config = VisionConfig(config) if config is not None else VisionConfig()
        self._config_path = config_path
        self._pan_pid = vision.PIDState()        # pan 軸
        self._tilt_pid = vision.PIDState()       # tilt 軸
        self._drive_ang_pid = vision.PIDState()  # drive follow: 角速度
        self._drive_lin_pid = vision.PIDState()  # drive follow: 直進速度
        self._servo_lost = 0                     # gaze/servo: ターゲット連続ロスト数
        self._drive_lost = 0                     # drive: ターゲット連続ロスト数
        self._busy = False                       # 処理中は後続フレームを間引く
        self._last_spoken: str | None = None     # 発話の重複抑制
        self._last_count = -1                    # 検出件数の変化時だけログ
        self._warned_no_cv2 = False
        # ESP32 の実在関節（servo policy 用）。/esp32/config と /joint_states から更新。
        self.joints: set[str] = set()
        self.joint_angles: dict[str, float] = {}

    # ── 設定 ───────────────────────────────────────────────────────────
    def config_frame(self) -> str:
        """現在の設定を /vision/config の publish フレームにする（web の同期用）。"""
        return frames.pub("/vision/config", {"data": json.dumps(self.config.to_dict())})

    def apply_config(self, body: dict) -> None:
        """/vision/config の msg（std_msgs/String）の部分設定を deep-merge して保存する。

        data は JSON 文字列を想定するが、rosbridge クライアントによっては既に
        dict の場合もあるため両対応。
        """
        raw = body.get("data")
        if isinstance(raw, str):
            try:
                patch = json.loads(raw)
            except ValueError:
                log.warning("vision config: data is not valid JSON; ignored")
                return
        else:
            patch = raw
        if not isinstance(patch, dict):
            return
        self.config = VisionConfig(merge_config(self.config.to_dict(), patch))
        if self._config_path:
            try:
                with open(self._config_path, "w", encoding="utf-8") as f:
                    json.dump(self.config.to_dict(), f, ensure_ascii=False, indent=2)
            except OSError as e:
                log.warning("vision config の保存に失敗しました (%s)", e)

    def update_joints(self, esp32_config: dict) -> None:
        """ESP32 の config から実在関節名を取り出す。"""
        joints = (esp32_config.get("servos") or {}).get("joints") or {}
        if isinstance(joints, dict):
            self.joints = set(joints)

    def update_joint_angles(self, msg: dict) -> None:
        """/joint_states（sensor_msgs/JointState）から現在角度（rad）を覚える。"""
        names = msg.get("name") or []
        positions = msg.get("position") or []
        self.joint_angles = dict(zip(names, positions))

    # ── 実行 ───────────────────────────────────────────────────────────
    async def process_frame(self, bgr) -> None:
        """WebRTC ハブが取り出した BGR フレームを検出 → 配信 → policy 指令する。"""
        cfg = self.config
        if not cfg.enabled or cfg.mode == "off":
            return
        if not vision.HAVE_CV2:
            if not self._warned_no_cv2:
                self._warned_no_cv2 = True
                log.warning("vision is enabled but OpenCV/numpy are not installed; "
                            "detection is disabled. Run: pip install -r requirements.txt")
            return
        if self._busy:
            return
        self._busy = True
        try:
            result = await asyncio.get_running_loop().run_in_executor(
                None, _detect_and_decide, bgr, cfg, self.joints or None, dict(self.joint_angles),
                self._pan_pid, self._tilt_pid, self._drive_ang_pid, self._drive_lin_pid)
        except Exception:
            log.exception("vision processing failed")
            result = None
        finally:
            self._busy = False
        if not result:
            return
        self._track_lost(result["target_found"], cfg)
        self._log_count(result, cfg)
        await self._deliver(result, cfg)

    def _track_lost(self, found: bool, cfg: VisionConfig) -> None:
        """tolerance を超えてロストが続いたときだけ PID をリセットする
        （短いドロップアウトで積分を失わないため）。"""
        if found:
            self._servo_lost = self._drive_lost = 0
            return
        self._servo_lost += 1
        self._drive_lost += 1
        if self._servo_lost > cfg.servo_lost_tolerance:
            self._pan_pid.reset()
            self._tilt_pid.reset()
        if self._drive_lost > cfg.drive_lost_tolerance:
            self._drive_ang_pid.reset()
            self._drive_lin_pid.reset()

    def _log_count(self, result: dict, cfg: VisionConfig) -> None:
        # 検出件数が変わったときだけ、解像度つきでログ。
        #   ・"vision[aruco] 640x480: 0 detection(s)" が出続ける → 検出は走るが0件
        #     （= 辞書ミスマッチ / マーカーが小さい・ぼやけ）→ Vision パネルで辞書を合わせる
        #   ・このログが一切出ない → 検出に到達していない（busy/間引き/設定 off 等）
        count = len(result["detections"]["detections"])
        if count != self._last_count:
            self._last_count = count
            w, h = result["img_wh"]
            log.info("vision[%s] %dx%d: %d detection(s)", cfg.mode, w, h, count)

    async def _deliver(self, result: dict, cfg: VisionConfig) -> None:
        send = self._send
        await send("web", frames.pub("/vision/detections", result["detections"]))
        strings = result["strings"]
        if strings:
            await send("web", frames.pub("/vision/markers", {"data": ",".join(strings)}))
        if result["look_at"] is not None:
            await send("app", frames.pub("/look_at", result["look_at"]))
        if result["servo"] is not None:
            await send("esp32", frames.pub("/servo/command", result["servo"]))
        if result["cmd_vel"] is not None:
            await send("esp32", frames.pub("/cmd_vel", result["cmd_vel"]))
        if cfg.speak and strings:
            joined = ",".join(strings)
            if joined != self._last_spoken:
                self._last_spoken = joined
                await send("app", frames.pub("/speech/say", {"data": joined}))
