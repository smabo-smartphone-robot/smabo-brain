import argparse
import logging
from aiohttp import web
from .relay import create_app

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)


def main() -> None:
    parser = argparse.ArgumentParser(description="smabo-brain relay server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    print("smabo-brain relay server")
    print(f"  smabo-app WS  : ws://<this-host>:{args.port}/")
    print(f"  smabo-web WS  : ws://<this-host>:{args.port}/ui")
    print(f"  smabo-esp32 WS: ws://<this-host>:{args.port}/esp32")

    app = create_app()
    web.run_app(app, host=args.host, port=args.port, print=None)


main()
