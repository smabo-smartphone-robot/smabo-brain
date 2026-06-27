"""Pluggable speech-to-text for smabo-brain.

smabo-app handles only the wake word now: on wake it records the utterance
(16 kHz mono WAV) and sends it on ``/speech/audio``. This module turns that
audio into text, which the relay republishes on ``/speech/recognized`` (visible
in smabo-web).

Two backends, chosen at startup (``--stt-engine``, default ``vosk``):

  - ``vosk``    — lightweight, offline, runs on an SBC. Needs a model directory
                  (``--stt-model``); without one it asks vosk to fetch a small
                  model for the language on first use (needs internet once).
  - ``whisper`` — faster-whisper: higher accuracy, offline, heavier
                  (``--stt-model`` selects the size, e.g. ``small``).

Both libraries are imported lazily, so the brain runs without them installed —
STT just stays disabled (a warning is logged) and ``/speech/audio`` is ignored.

``transcribe`` takes the decoded PCM as an ``int16`` numpy array plus its sample
rate, so the WAV parsing lives in one place (relay.py) and each backend only
deals with samples.
"""

import logging

log = logging.getLogger(__name__)


def wav_to_samples(raw: bytes):
    """Parse 16-bit PCM WAV bytes → (int16 mono ndarray, sample_rate).

    Returns (None, 0) on failure or unsupported format. Multi-channel audio is
    downmixed to the first channel. Shared by smabo-brain (WS) and
    smabo-brain-ros so the WAV handling lives in one place.
    """
    import io
    import wave

    import numpy as np

    try:
        with wave.open(io.BytesIO(raw), "rb") as wf:
            if wf.getsampwidth() != 2:
                return None, 0
            sr = wf.getframerate()
            ch = wf.getnchannels()
            frames = wf.readframes(wf.getnframes())
    except Exception:
        return None, 0
    samples = np.frombuffer(frames, dtype=np.int16)
    if ch > 1:
        samples = samples.reshape(-1, ch)[:, 0]
    return samples, sr


class SttEngine:
    def __init__(self, *, engine: str = "vosk", model: str = "",
                 language: str = "ja") -> None:
        self.engine = (engine or "vosk").lower()
        self.language = language or "ja"
        self._model = model
        self._impl = None
        self._load()

    @property
    def available(self) -> bool:
        return self._impl is not None

    def _load(self) -> None:
        try:
            if self.engine == "whisper":
                self._impl = _WhisperImpl(self._model or "small", self.language)
            else:
                self.engine = "vosk"
                self._impl = _VoskImpl(self._model, self.language)
            log.info("STT engine ready: %s (language=%s)", self.engine, self.language)
        except Exception as e:
            log.warning("STT(%s) を初期化できませんでした（/speech/audio は無視されます）: %s",
                        self.engine, e)
            self._impl = None

    def transcribe(self, samples, sample_rate: int) -> str:
        """PCM int16 ndarray (mono) → recognized text ('' on failure/empty)."""
        if self._impl is None:
            return ""
        try:
            return self._impl.transcribe(samples, sample_rate)
        except Exception:
            log.exception("STT transcription failed")
            return ""


class _VoskImpl:
    def __init__(self, model_path: str, language: str) -> None:
        from vosk import Model, KaldiRecognizer  # lazy import
        import vosk
        vosk.SetLogLevel(-1)  # silence vosk's verbose stderr logging
        self._KaldiRecognizer = KaldiRecognizer
        if model_path:
            self._model = Model(model_path)
        else:
            # Auto-fetch a small model for the language (needs internet once;
            # cached under ~/.cache/vosk). Pass --stt-model for an offline path.
            self._model = Model(lang=language)
        # vosk's Japanese model emits spaces between tokens; drop them so the
        # recognized string reads naturally.
        self._strip_spaces = language.lower().startswith("ja")

    def transcribe(self, samples, sample_rate: int) -> str:
        import json
        rec = self._KaldiRecognizer(self._model, float(sample_rate))
        rec.AcceptWaveform(samples.tobytes())
        text = json.loads(rec.FinalResult()).get("text", "")
        if self._strip_spaces:
            text = text.replace(" ", "")
        return text.strip()


class _WhisperImpl:
    def __init__(self, model_size: str, language: str) -> None:
        from faster_whisper import WhisperModel  # lazy import
        # int8 on CPU is the practical default; users with a GPU can edit this.
        self._model = WhisperModel(model_size, device="cpu", compute_type="int8")
        self._language = language

    def transcribe(self, samples, sample_rate: int) -> str:
        # faster-whisper wants float32 mono at 16 kHz; the app records 16 kHz.
        audio = samples.astype("float32") / 32768.0
        segments, _info = self._model.transcribe(
            audio, language=self._language, beam_size=1)
        return "".join(seg.text for seg in segments).strip()
