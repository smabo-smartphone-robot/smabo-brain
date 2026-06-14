# smabo-brain

smabo システムの中継・処理サーバ（Python / aiohttp）。

全コンポーネントが WebSocket クライアントとして接続してくる唯一のサーバで、
センサデータの中継と、マイコンではできない処理（オドメトリ積分など）を担います。

```
smabo-app  ──►  smabo-brain  ◄──  smabo-web
                    ▲
                    └──  smabo-esp32
```

## エンドポイント

サーバはデフォルト `0.0.0.0:9090` で待ち受け、接続元ごとにパスを分けます。

| パス | 接続元 | 役割 |
|------|--------|------|
| `/` | smabo-app | センサデータ（IMU/GPS/カメラ）送信 → web へ中継 |
| `/ui` | smabo-web | 制御指令・設定送信 → esp32 へ中継、フィードバック受信 |
| `/esp32` | smabo-esp32 | 制御指令受信、`/wheel_vel`・`/joint_states` 等を送信 |

メッセージ形式は rosbridge v2.0 互換 JSON。

## 送信元 prefix

各クライアントは publish するトピックに送信元 prefix を付けて送ります
（smabo-app → `/app`、smabo-web → `/web`、smabo-esp32 → `/esp32`）。
brain はこの prefix を**剥がしてから** canonical なトピック名で宛先へ再配信します
（例: esp32 が送る `/esp32/wheel_vel` → 積分後 `/odom`、web が送る `/web/cmd_vel` → `/cmd_vel`）。
受信側は常に prefix 無しの canonical 名で受け取ります。
`set_config` / `get_config` / `call_service` など publish 以外の op には prefix は付きません。
将来 ROS 化する際は、この剥離箇所を twist_mux や topic remap に置き換えることで自然に移行できます。

## 処理

- **オドメトリ積分**: smabo-esp32 が送る `/wheel_vel`（left/right m/s, dt）を受信し、
  `brain/odometry.py` で x/y/θ を積分して nav_msgs/Odometry 形式の `/odom` を web へ送出します。
  ホイール径などのパラメータは esp32 の `set_config` 応答から自動同期します。
  （将来 IMU/GPS とのフュージョンをここに追加できます）

## 起動

```bash
pip3 install -r requirements.txt
python3 -m brain                 # --host / --port で変更可
```

## web UI

操作用 web フロントエンド（`smabo-web`）は別リポジトリです。
ブラウザから `ws://<brain-host>:9090/ui` に接続して使用します。
