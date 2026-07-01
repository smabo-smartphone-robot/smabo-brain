#!/usr/bin/env bash
# smabo-brain セットアップスクリプト
#   ホスト名を "smabo-brain" に設定し、mDNS (smabo-brain.local) を有効にする。
#   macOS / Linux / WSL を自動判定。sudo 権限が必要。初回のみ実行すればよい（冪等）。
#
# 例:
#   ./setup.sh

set -euo pipefail

HOSTNAME="smabo-brain"

# ── 環境判定 ─────────────────────────────────────────────────
OS="$(uname -s)"
IS_WSL=false
if [ "$OS" = "Linux" ] && grep -qi "microsoft\|wsl" /proc/version 2>/dev/null; then
  IS_WSL=true
fi

if [ "$OS" = "Darwin" ]; then
  ENV_NAME="macOS"
elif [ "$IS_WSL" = "true" ]; then
  ENV_NAME="WSL"
else
  ENV_NAME="Linux"
fi

echo "[setup] 環境: ${ENV_NAME}"
echo "[setup] ホスト名を ${HOSTNAME} に設定します (${HOSTNAME}.local で接続可能になります)"

# ── macOS ────────────────────────────────────────────────────
if [ "$OS" = "Darwin" ]; then
  sudo scutil --set HostName      "$HOSTNAME"
  sudo scutil --set LocalHostName "$HOSTNAME"
  sudo scutil --set ComputerName  "$HOSTNAME"
  echo "[setup] 完了。Bonjour が組み込まれているため追加設定は不要です。"

# ── WSL ──────────────────────────────────────────────────────
elif [ "$IS_WSL" = "true" ]; then
  # ホスト名設定
  sudo hostnamectl set-hostname "$HOSTNAME" 2>/dev/null \
    || echo "$HOSTNAME" | sudo tee /etc/hostname > /dev/null

  if ! grep -q "127\.0\.1\.1\s*${HOSTNAME}" /etc/hosts 2>/dev/null; then
    echo "127.0.1.1 ${HOSTNAME}" | sudo tee -a /etc/hosts > /dev/null
  fi

  # avahi-daemon インストール
  if ! command -v avahi-daemon &>/dev/null; then
    echo "[setup] avahi-daemon をインストールしています..."
    sudo apt-get update -qq
    sudo apt-get install -y avahi-daemon
  fi

  # WSL では systemd の有無に応じてサービス起動を切り替える
  if systemctl is-active --quiet avahi-daemon 2>/dev/null; then
    sudo systemctl restart avahi-daemon
  elif command -v service &>/dev/null; then
    sudo service avahi-daemon restart
  else
    sudo avahi-daemon --daemonize --no-drop-root 2>/dev/null || true
  fi

  # mirror モード判定（/proc/net 内のインターフェースが Windows と同一か確認）
  IS_MIRROR=false
  if grep -q "mirrored" /proc/sys/kernel/osrelease 2>/dev/null; then
    IS_MIRROR=true
  elif ip addr 2>/dev/null | grep -q "eth0\|Wi-Fi\|wlan0" \
    && ! ip addr 2>/dev/null | grep -q "172\.1[6-9]\|172\.2[0-9]\|172\.3[01]"; then
    IS_MIRROR=true
  fi

  echo "[setup] 完了。"
  if [ "$IS_MIRROR" = "true" ]; then
    echo "[setup] mirror モードが有効です。${HOSTNAME}.local で LAN から接続できます。"
  else
    echo "[setup] 注意: WSL2 のデフォルト(NAT)モードでは他デバイスから"
    echo "[setup]   ${HOSTNAME}.local へ到達できません。"
    echo "[setup]"
    echo "[setup]   mirror モードを有効にすると解決します:"
    echo "[setup]     1. Windows で %USERPROFILE%\\.wslconfig をテキストエディタで開く"
    echo "[setup]     2. 以下を追記して保存:"
    echo "[setup]          [wsl2]"
    echo "[setup]          networkingMode=mirrored"
    echo "[setup]     3. PowerShell で: wsl --shutdown  （WSL を再起動）"
    echo "[setup]     4. WSL を起動し直してから ./setup.sh を再実行"
  fi

# ── Linux (native) ───────────────────────────────────────────
else
  sudo hostnamectl set-hostname "$HOSTNAME"

  if ! grep -q "127\.0\.1\.1\s*${HOSTNAME}" /etc/hosts 2>/dev/null; then
    echo "127.0.1.1 ${HOSTNAME}" | sudo tee -a /etc/hosts > /dev/null
    echo "[setup] /etc/hosts に ${HOSTNAME} を追記しました"
  fi

  if ! command -v avahi-daemon &>/dev/null; then
    echo "[setup] avahi-daemon をインストールしています..."
    sudo apt-get update -qq
    sudo apt-get install -y avahi-daemon
  fi

  sudo systemctl enable avahi-daemon
  sudo systemctl restart avahi-daemon
  echo "[setup] 完了。同じ LAN 内から ${HOSTNAME}.local で接続できます。"
fi
