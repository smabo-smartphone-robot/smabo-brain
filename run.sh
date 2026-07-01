#!/usr/bin/env bash
# smabo-brain 起動スクリプト
#   初回は venv を作成して依存をインストールし、リレーサーバを起動する。
#   引数はそのまま `python -m brain` に渡る（--host / --port）。
#
# 例:
#   ./run.sh                 # 0.0.0.0:9090 で起動
#   ./run.sh --port 9091
set -euo pipefail

cd "$(dirname "$0")"

# mDNS ホスト名チェック
if [ "$(hostname)" != "smabo-brain" ]; then
  echo "[run.sh] 警告: ホスト名が 'smabo-brain' ではありません（現在: $(hostname)）"
  echo "[run.sh]   smabo-brain.local で接続するには ./setup.sh を先に実行してください"
fi

VENV="${SMABO_BRAIN_VENV:-.venv}"
PYTHON="${PYTHON:-python3}"

if [ ! -d "$VENV" ]; then
  echo "[run.sh] creating venv at $VENV ..."
  "$PYTHON" -m venv "$VENV"
  "$VENV/bin/pip" install --upgrade pip
  "$VENV/bin/pip" install -r requirements.txt
fi

exec "$VENV/bin/python" -m brain "$@"
