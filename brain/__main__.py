import argparse
import json
import logging
import os
from aiohttp import web
from .server import create_app

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)

log = logging.getLogger("brain")


def _load_vision_config(path: str | None) -> dict | None:
    """画像処理設定の起動時初期値を JSON ファイルから読む。

    ファイルの中身は /vision/config の data と同じ形（VisionConfig.to_dict()
    形状）。部分指定でよく、欠けたキーは VisionConfig の既定で補完される。
    読めない場合は警告して None（＝組み込み既定で起動）を返す。
    """
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            cfg = json.load(f)
    except Exception as e:
        log.warning("vision config %r を読めませんでした (%s); 既定値で起動します", path, e)
        return None
    if not isinstance(cfg, dict):
        log.warning("vision config %r は JSON オブジェクトではありません; 既定値で起動します", path)
        return None
    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(description="smabo-brain")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument(
        "--vision-config",
        default=os.environ.get("SMABO_VISION_CONFIG", "vision_config.json"),
        metavar="PATH",
        help="画像処理設定の JSON ファイルパス（既定: vision_config.json）。"
             "起動時に読み込み、変更のたびに自動保存される。"
             "環境変数 SMABO_VISION_CONFIG でも指定可。",
    )
    parser.add_argument(
        "--stt-engine",
        default=os.environ.get("SMABO_STT_ENGINE", "vosk"),
        choices=["vosk", "whisper"],
        help="音声認識エンジン（既定: vosk）。whisper は faster-whisper を使用。",
    )
    parser.add_argument(
        "--stt-model",
        default=os.environ.get("SMABO_STT_MODEL", ""),
        metavar="PATH_OR_SIZE",
        help="vosk はモデルディレクトリのパス（空なら言語から小モデルを自動取得）。"
             "whisper はサイズ名（例: small）。環境変数 SMABO_STT_MODEL でも指定可。",
    )
    parser.add_argument(
        "--stt-language",
        default=os.environ.get("SMABO_STT_LANGUAGE", "ja"),
        help="音声認識の言語（既定: ja）。",
    )
    args = parser.parse_args()

    vision_config = _load_vision_config(args.vision_config)
    vision_config_path = args.vision_config
    stt_config = {
        "engine": args.stt_engine,
        "model": args.stt_model,
        "language": args.stt_language,
    }

    print("smabo-brain")
    print(f"  WebSocket     : ws://<this-host>:{args.port}/  (app / web / esp32 / sim, rosbridge-style)")
    if vision_config is not None:
        print(f"  vision config : {vision_config_path} (loaded)")
    else:
        print(f"  vision config : {vision_config_path} (not found, using defaults; will be created on first change)")
    print(f"  STT engine    : {args.stt_engine} (language={args.stt_language})")

    app = create_app(vision_config=vision_config, vision_config_path=vision_config_path, stt_config=stt_config)
    web.run_app(app, host=args.host, port=args.port, print=None)


main()
