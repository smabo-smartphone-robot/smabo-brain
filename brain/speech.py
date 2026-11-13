"""音声認識の受付（smabo-brain の WebSocket 版）。

smabo-app が録音した発話を /speech/audio（audio_common_msgs/AudioData、data は
base64 の WAV）で送ってくるので、文字にして /speech/recognized で web に送る。
エンジンと WAV の解釈は共通ロジック ``brain.stt`` にある。
"""

import asyncio
import base64
import binascii
import logging

from . import frames
from .stt import SttEngine, wav_to_samples
from .vision_pipeline import Send

log = logging.getLogger(__name__)


class Speech:
    def __init__(self, send: Send, stt_config: dict | None = None) -> None:
        self._send = send
        self._stt = SttEngine(**(stt_config or {}))
        self._warned = False

    def _transcribe(self, b64: str) -> str:
        """base64 WAV → 文字列。失敗時は ''。STT は重いのでワーカースレッドで呼ぶ。"""
        try:
            raw = base64.b64decode(b64)
        except (binascii.Error, ValueError):
            return ""
        samples, sr = wav_to_samples(raw)
        if samples is None or samples.size == 0:
            return ""
        return self._stt.transcribe(samples, sr)

    async def handle_audio(self, msg: dict) -> None:
        """/speech/audio を受けて STT し、結果を /speech/recognized で web へ。"""
        if not self._stt.available:
            if not self._warned:
                self._warned = True
                log.warning("/speech/audio を受信しましたが STT が無効です。"
                            "vosk（または faster-whisper）を導入してください。")
            return
        b64 = (msg or {}).get("data")
        if not b64:
            return
        text = await asyncio.get_running_loop().run_in_executor(None, self._transcribe, b64)
        if text:
            log.info("STT: %r", text)
            await self._send("web", frames.pub("/speech/recognized", {"data": text}))
