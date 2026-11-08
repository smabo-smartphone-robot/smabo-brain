# smabo-brain

smabo システムの中継・処理サーバ（Python / aiohttp）。

全コンポーネントが WebSocket クライアントとして接続してくる唯一のサーバで、
センサデータの中継と、マイコンではできない処理（オドメトリ積分・画像処理・音声認識・
カメラ映像の WebRTC 終端）を担います。

```
smabo-app  ──►  smabo-brain  ◄──  smabo-web
                    ▲
                    └──  smabo-esp32
```

## エンドポイント

サーバはデフォルト `0.0.0.0:9090` で待ち受けます。接続口は `ws://<host>:9090/` の1つだけで、
smabo-brain-ros（rosbridge）と同じ手順でつなぎます。クライアントは brain がどちらでも
設定を変えずに動きます。

1. `/` に接続し、最初のフレームで token を見せる（`{"op": "auth", "token": "…"}`）か、
   招待コードでペアリングする（`{"op": "pair", "code": "ABCD-EFGH", "name": "iPhone"}`）。
   brain は `{"op": "auth" | "pair", "ok": true, "role", "name", "id"}` で答える
   （ペアリングでは `token` と、機器には `relay_key` も付く）。下の「ペアリング」を参照
2. publish するトピックを `advertise`、受け取るトピックを `subscribe` する
3. 送信元 prefix（下記）を付けて `publish` する
4. `/hello` を 1 回 publish する（`{"protocol": 1, "client": "smabo-web", "version": "0.1.0"}`）

brain は `/brain/info`（`{"protocol", "brain", "version", "auth"}`）で答えます。プロトコル番号
（`brain/topics.py` の `PROTOCOL_VERSION`）が違うと、brain のログと smabo-web の画面に
警告が出ます。クライアントも `/brain/info` を見て、自分の側で警告します。トピック名を
変える・消す、メッセージの型や意味を変えるときは、番号を 1 つ上げてください。

相手が web / app / esp32 のどれかは、token の控えに記録された役割で決まります。
役割と違う prefix のトピックは捨てます（ブラウザが ESP32 になりすませない）。
`--no-auth` のときだけ、従来どおり最初に届いたフレーム（prefix 付きの advertise /
publish、または特定のクライアントだけが受け取るトピックの subscribe）から判別します。誰が何を送受信するかの表の正本は smabo-protocol の `protocol.json`
で、全トピックの一覧は smabo-protocol の `TOPICS.md` にあります。brain は生成された
`brain/protocol_gen.py` を `brain/topics.py` から読みます。

| 接続元 | おもな送信 | おもな受信 |
|--------|-----------|-----------|
| smabo-app | IMU/GPS、録音音声（`/speech/audio`）、WebRTC シグナリング | `/look_at`・`/speech/say`・`/expression`（表情の名前、String） |
| smabo-web | 制御指令（esp32 / app へ中継） | `/odom`・センサ・画像処理の結果 |
| smabo-esp32 | `/wheel_vel`・`/joint_states`・`/scan`・`/config` | `/cmd_vel`・`/servo/command` |

カメラ映像は WebSocket ではなく WebRTC で smabo-app から brain に届きます
（シグナリングのみ WebSocket。`brain/webrtc_hub.py`）。

メッセージ形式は rosbridge v2.0 互換 JSON。

## バージョンと互換性

smabo のリポジトリは、同じ minor 番号（`0.2.x` など）どうしで組み合わせます。
つながるかどうかを決めるのは、`/hello` と `/brain/info` で送り合うプロトコル番号です。

| プロトコル | smabo-brain / brain-ros | smabo-web | smabo-app | smabo-esp32 | smabo-sim | 変更点 |
|-----------|------------------------|-----------|-----------|-------------|-----------|--------|
| 1 | 0.2.x | 0.2.x | 0.2.x | 0.2.x | 0.2.x | `/hello`・`/brain/info` でプロトコル番号を確かめる。`GET /config` は Wi-Fi パスワードを返さない |
| （なし） | 0.1.x | 0.1.x | 0.1.x | （番号なし） | 0.1.x | プロトコル番号の確認なし |

番号を上げたら、この表に行を足してください。`tests/test_protocol.py` は、隣にチェックアウト
されている各リポジトリのプロトコル番号とバージョンが、brain と合っているかを確かめます。

設定（config / mode）は brain を経由しません。smabo-web は ESP32 の REST API
（`http://<esp32>/config`・`/mode`）へ直接アクセスします。brain はオドメトリ積分に
必要な車輪ジオメトリ等のため、ESP32 が `/esp32/config`（std_msgs/String の data に JSON、
Wi-Fi 設定は除く）で送ってくる config スナップショットを内部の同期にのみ使います。

## 送信元 prefix

各クライアントは publish するトピックに送信元 prefix を付けて送ります
（smabo-app → `/app`、smabo-web → `/web`、smabo-esp32 → `/esp32`）。
brain はこの prefix を**剥がしてから** canonical なトピック名で宛先へ再配信します
（例: esp32 が送る `/esp32/wheel_vel` → 積分後 `/odom`、web が送る `/web/cmd_vel` → `/cmd_vel`）。
受信側は常に prefix 無しの canonical 名で受け取ります。
smabo-brain-ros では、同じ表（`brain/topics.py`）から topic_tools relay を作って prefix を剥がします。

## 処理

- **オドメトリ積分**: smabo-esp32 が送る `/wheel_vel`（left/right m/s, dt）を受信し、
  `brain/odometry.py` で x/y/θ を積分して nav_msgs/Odometry 形式の `/odom` を web へ送出します。
  ホイール径などのパラメータは、esp32 が接続時・設定変更時に送る config スナップショット
  （`/esp32/config`）から自動同期します（config 自体の読み書きは web ↔ esp32 の REST 直通）。
  （将来 IMU/GPS とのフュージョンをここに追加できます）
- **画像処理**: WebRTC で受けたカメラ映像から検出（aruco / color / face / qr、`brain/vision.py`）し、
  `/vision/detections`・`/vision/markers` を web へ送出。`/vision/config` の `behaviors` に応じて
  `/look_at`（app）・`/servo/command`・`/cmd_vel`（esp32）を出力します（首・走行の追従は PID）。
  web がプレビューを要求したときは映像を web へ中継します。
- **音声認識**: smabo-app の録音（`/speech/audio`、base64 WAV）を STT（vosk / faster-whisper、
  `brain/stt.py`）にかけ、`/speech/recognized` を web へ送出します。

## ペアリング

brain は、ペアリングしたクライアントだけを受け付けます（`brain/pairing.py`）。

| 何を | どうやって |
|------|-----------|
| 最初のブラウザ | brain のログに出るコード（`最初のブラウザ用のペアリングコード: ABCD-EFGH`）を smabo-web に入力する。5 分ごとに新しいコードが出る |
| 2 台目以降のブラウザ・スマホ・ESP32 | ペアリング済みの smabo-web の「Devices」で招待コードを出し、その機器に入力する（5 分間・1 回だけ有効） |
| 外す | smabo-web の「Devices」で Remove。その機器の接続はすぐ切れる。最後のブラウザは外せない |
| 全部やり直す | `python3 -m brain --reset-pairing`（brain を動かす PC を触れる人だけができる） |

- brain は token そのものを持たず、SHA-256 のハッシュだけを `~/.smabo/brain/pairings.json`
  （`--pairings` で変更）に保存します。ファイルは本人だけが読める権限で書きます。
- 間違ったコードが 5 回続くと、出ているコードをすべて無効にします。
- スマホと ESP32 はペアリングのときに **中継用の鍵**（relay key）を受け取り、ハッシュだけを保存します。
  smabo-web の設定は brain の `/devices/esp32/…`・`/devices/app/…` に送られ、brain がこの鍵を
  `X-Smabo-Relay-Key` ヘッダーに付けて機器の REST へ転送します。ペアリング済みの機器は、
  この鍵のない要求を断り、ブラウザ向けの CORS も返しません。
- `--no-auth`（または `SMABO_NO_AUTH=1`）で従来の動き（誰でも接続でき、web は機器へ直接つなぐ）に戻せます。開発用です。

HTTP API（smabo-web 用。`Authorization: Bearer <ブラウザの token>` が必要）:

| メソッドとパス | 内容 |
|---------------|------|
| `GET /pairings` | ペアリング済みの一覧（`online` 付き）と、自分の `id` |
| `POST /invites` `{"role": "web" \| "app" \| "esp32", "name"?}` | 招待コード |
| `PATCH /pairings/{id}` `{"name"}` | 名前の変更 |
| `DELETE /pairings/{id}` | ペアリングを外す |
| `* /devices/{esp32 \| app}/{path}` | 機器の REST へ転送（機器が未接続なら 504、鍵を断られたら 502） |

## 操作権（1 つのブラウザだけが動かす）

smabo-web のタブが複数開いていても、走行（`/cmd_vel`）とアーム（`/servo/command`）を送れるのは 1 つのタブだけです（`brain/control.py`）。

- 誰も操作していないとき、最初に走行・アームの指令を送ったタブが操作権を取る
- ほかのタブの指令は捨てる。そのタブには「○○が操作中」と「操作を引き継ぐ」（`{"op": "control", "action": "claim"}`）を出す
- 操作中のタブは `{"op": "control", "action": "release"}` で手放す。タブを閉じる・切れたときも手放す
- 操作権が別のタブへ移る・手放されると、brain は ESP32 に止まる指令（速度 0 の `/cmd_vel`）を送る
- 誰が操作中かは `{"op": "notice", "event": "control", "holder": 名前 or null, "you": bool}` で全タブに知らせる（接続時にも 1 回）。名前はペアリング名（`--no-auth` では `browser 1` など）
- `/brain/info` の `control: true` で web はこの仕組みがあると分かる（smabo-brain-ros にはないので、web は従来どおり動く）

## 起動

```bash
./run.sh                 # 初回は venv 作成＋依存導入、以降はそのまま起動
./run.sh --port 9091     # 引数は python -m brain に渡る（--host / --port）
```

手動で行う場合:

```bash
pip3 install -r requirements.txt
python3 -m brain                 # --host / --port で変更可
```

### 画像処理の起動時初期値

`--vision-config <path.json>`（または環境変数 `SMABO_VISION_CONFIG`）で画像処理
設定の初期値を渡せます。JSON は `/vision/config` の `data` と同形（部分指定可）。

```bash
python3 -m brain --vision-config vision.json
# vision.json 例: {"enabled": true, "mode": "color", "color": "green",
#                  "behaviors": {"look_at": true, "servo": false}}
```

未指定なら検出無効（`mode=off`）で起動し、実行中は `/vision/config` で部分上書き
できます（現在値に deep-merge）。

## web UI

操作用 web フロントエンド（`smabo-web`）は別リポジトリです。
ブラウザから `ws://<brain-host>:9090/` に接続して使用します。

## ファイル構成

| ファイル | 内容 |
|----------|------|
| `brain/__main__.py` | 起動オプション |
| `brain/server.py` | WebSocket の受信と振り分け |
| `brain/clients.py` | 接続中のクライアントと、その種類の判別 |
| `brain/pairing.py` | ペアリングの控え・招待コード・token の照合 |
| `brain/gateway.py` | smabo-web 用の HTTP API（ペアリングの管理、機器への設定の転送） |
| `brain/frames.py` | rosbridge JSON の組み立て（publish・`/odom`・接続通知） |
| `brain/vision_pipeline.py` | 画像処理の実行と状態（設定・PID） |
| `brain/speech.py` | 録音の受け付けと音声認識 |
| `brain/topics.py`・`odometry.py`・`vision.py`・`stt.py`・`webrtc_hub.py` | 共通ロジック。smabo-brain-ros も import する |

テスト: `python -m pytest tests`
