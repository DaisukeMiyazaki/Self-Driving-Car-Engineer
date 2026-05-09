# Tailscale 経由のリモートアクセス構成

Linux ホスト（Ubuntu 24.04 + AMD RX 7800 XT, **常時 SSH 運用のヘッドレスサーバ**）で動かしている CARLA + ros-bridge + capstone の Docker スタックに、Mac など別端末から Tailscale 経由でアクセスする構成。

`docs/simulator-landscape.md` がシム選定とハードウェア要件の話なのに対し、本ドキュメントは**「立てたシムを別端末から見る／操作する」レイヤ**を扱う。

---

## 1. 運用前提（このサーバ固有の制約）

ハードウェア／OS の事前確認結果：

| 項目 | 確認結果 |
|---|---|
| OS | Ubuntu 24.04.3 LTS（Noble） |
| CPU | AMD Ryzen 5 9600X（Zen 5、6C/12T） |
| RAM | 30 GiB |
| GPU 1（discrete） | AMD Radeon RX 7800 XT (Navi 32, RDNA 3) — `card0` / `renderD128` |
| GPU 2（iGPU） | Ryzen 9600X 内蔵 Radeon — `card1` / `renderD129` |
| カーネルドライバ | 両 GPU とも `amdgpu`（kernel 6.8.0-110-generic） |
| Mesa | 25.2.8 |
| VAAPI エンコーダ（RX 7800 XT） | H.264 / HEVC Main / HEVC Main10 / AV1 Profile 0 すべて有効 |
| セッション | **`tty` のみ**、X11/Wayland セッションなし |
| アクセス | **常時 SSH（別ロケーション）** |

**重要な含意**: 物理コンソールにログインできないため「ログイン中の GUI セッション」が存在しない。これがリモートアクセス方式の選定を左右する。

### 1.1 GPU デバイス権限の前提

`/dev/dri/` 配下のデバイスは `root:video` および `root:render` 所有で `0660`。**ユーザを `video` と `render` グループに追加**する必要がある：

```bash
sudo usermod -aG video,render $USER
# SSH を切って入り直す（usermod は新セッションでないと反映されない）
```

---

## 2. 採用する 2 系統と、見送る 1 系統

### 2.1 全体図

```
                        Tailscale tailnet
                                   │
   ┌───────────────────────────────┴───────────────────────────────┐
   │                                                               │
   ▼                                                               ▼
┌──────────────────────┐                          ┌──────────────────────┐
│ B: CARLA Python      │                          │ C: Foxglove + ros-   │
│    クライアント       │                          │    bridge            │
│ Mac で UE4 の画像を   │                          │ ROS トピックを       │
│ 直接受信＆描画        │                          │ ブラウザで可視化     │
└──────────────────────┘                          └──────────────────────┘
        ▲                                                      ▲
        │ TCP :2000                                            │ WebSocket :9090
        │                                                      │
   ┌────┴──────────────────────────────────────────────────────┴────┐
   │       Linux サーバ（ヘッドレス、Ubuntu 24.04 + RX 7800 XT）     │
   │  ┌──────────────────────┐                                       │
   │  │ CARLA -RenderOffScreen │←─→carla_ros_bridge ←─→ capstone     │
   │  │ Vulkan, GPU 計算のみ   │                                     │
   │  └──────────────────────┘                                       │
   └─────────────────────────────────────────────────────────────────┘
```

### 2.2 採用ルート

| ルート | 役割 |
|---|---|
| **B: CARLA Python クライアント (Mac 側)** | TCP :2000 経由で CARLA と直接対話。spectator camera で第三人称ビューを Mac の Pygame に描画、運転、シナリオ生成 |
| **C: Foxglove + rosbridge** | rosbridge_server を Docker 内に立て、Mac の Chrome から `https://app.foxglove.dev/` で接続。ROS トピック（pose、plan、画像、TF）をブラウザで可視化 |

### 2.3 見送るルート

**A: Sunshine + Moonlight** ＝ **このサーバ構成では非採用**。

見送り理由（記録）:
- Sunshine は「ログイン中の GUI セッション」をキャプチャする方式。物理コンソールが無く SSH のみのヘッドレス運用では、`Xvfb` か `Xorg dummy driver` で**仮想 X サーバを自前で立てる必要**がある
- そこまで構築コストを払って画面をキャプチャ→エンコード→ストリームするより、**CARLA を `-RenderOffScreen` で動かしてセンサデータだけ流す**ほうが GPU/CPU/帯域すべて軽い
- 本来の目的（ROS スタック開発）にはルート B + C で十分

VAAPI エンコーダ（H.264/HEVC/AV1）は動作確認済みなので、**将来サーバを GUI 運用に切り替えた場合や、別マシンで Sunshine を試す場合**にはそのまま流用できる。本リポジトリの範囲外。

---

## 3. ルート B: Mac 側 CARLA Python クライアント

### 3.1 役割
Mac 側で `carla` Python パッケージを動かして、Linux 上の CARLA サーバに **TCP で直接接続**。`manual_control.py` 等の公式サンプルで運転、または特定カメラの画像だけを Mac の Pygame ウィンドウで見る。spectator camera を spawn すれば第三人称俯瞰ビューも作れる。

### 3.2 構成要素
- Mac 側に Python 3.x + `pip install carla==0.9.15`
- Linux 側はサーバが動いていれば追加インストール不要
- 接続先: `carla.Client('linux-host.tailnet:2000')`

### 3.3 必要なポート
| ポート | プロトコル | 用途 |
|---|---|---|
| 2000 | TCP | CARLA RPC（クライアント API） |
| 2001 | TCP | ストリーミング（センサデータ） |
| 2002 | TCP | セカンダリ |

### 3.4 帯域とレイテンシ
- カメラ 1 系統で数 Mbps（Tailscale の暗号化込みで余裕）
- API レイテンシは Tailscale RTT 次第（同一サブネットなら 1〜5 ms）
- 画面ピクセルではなくセンサデータだけ流れるため軽量

### 3.5 制約
- **Apple Silicon Mac**: クライアントは動くが CARLA サーバ本体は Intel 専用なので Mac 側でサーバを立てる用途には使えない
- `manual_control.py` は Pygame ウィンドウを Mac 側で開く必要があるので、Mac の Python 環境に Pygame が要る

---

## 4. ルート C: Foxglove + rosbridge

### 4.1 役割
このリポジトリの ROS スタック（waypoint_updater、tl_detector、twist_controller 等）を開発する**本来の検証経路**。トピックを Foxglove で開いて、画像・点群・TF・プラン軌跡をブラウザ上でリアルタイム表示する。

### 4.2 構成要素
- **Linux 側**: `rosbridge_server`（ROS Noetic パッケージ `ros-noetic-rosbridge-server`）を新サービスとして `docker-compose.yml` に追加
- **Mac 側**: Chrome で [https://app.foxglove.dev/](https://app.foxglove.dev/) を開き、接続先に `ws://linux-host.tailnet:9090` を指定

### 4.3 必要なポート
| ポート | プロトコル | 用途 |
|---|---|---|
| 9090 | WebSocket (TCP) | rosbridge JSON over WS |
| 11311 | TCP | ROS master（Mac 側からは直接叩かない、内部用） |

### 4.4 docker-compose.yml への追加サービス（参考イメージ）
```yaml
  rosbridge:
    image: ros:noetic
    network_mode: host
    depends_on: [ros-bridge]
    command: bash -c "apt update && apt install -y ros-noetic-rosbridge-server &&
                      source /opt/ros/noetic/setup.bash &&
                      roslaunch rosbridge_server rosbridge_websocket.launch port:=9090"
```

### 4.5 強み
- **ブラウザだけで完結**（Foxglove の Web 版は SaaS で配信されている）
- `foxglove-bridge`（ProtoBuf ベースの新ブリッジ）に切り替えると帯域効率が rosbridge の数倍
- ROS 1 / ROS 2 両対応のため、ROS 2 移行後もそのまま使える

### 4.6 制約
- **CARLA UE4 のフル 3D は見えない**（あくまで ROS トピックの可視化）
- 画像トピックは base64/JPEG エンコードして送る形式なので、生 RGB を 30 fps で流すと負荷高め

---

## 5. CARLA を `-RenderOffScreen` で動かす意義

ヘッドレス運用に切り替える主たる構成変更：

```yaml
# docker-compose.yml 内 carla サービスの command
command: >
  /bin/bash -c "./CarlaUE4.sh -vulkan -RenderOffScreen -quality-level=Low"
```

| 含意 | 詳細 |
|---|---|
| GPU は計算のみ使う | UE4 のラスタライズ／センサ用 ray cast は GPU で走るが、**画面表示は省略** |
| X11 / Wayland 不要 | サーバが GUI セッションを持っていなくても起動できる |
| 帯域・CPU 削減 | キャプチャ→エンコード→ストリームの一連処理が要らない |
| Mac から見たい時 | ルート B（Python API + spectator camera）で**必要な画面だけ Mac 側で描画** |

ROS bridge はサーバ側の描画有無と無関係にトピックを publish するので、ルート C はそのまま機能する。

---

## 6. Tailscale 側の共通設定

### 6.1 必須
- Linux ホスト・Mac 双方に Tailscale クライアントを導入し、同一 tailnet にログイン
- `network_mode: host` の Docker サービスは自動的に Tailscale IP からも見える（Linux 限定の挙動）

### 6.2 推奨
- **MagicDNS を有効化**: `linux-host.tailnet.ts.net` のようなホスト名で接続できる
- **Tailscale ACL** で `Mac → Linux:{2000-2002, 9090}` のみに絞る
- **Tailscale SSH** を有効化すれば、リモート shell も同一 tailnet で完結

### 6.3 やってはいけない
- **Tailscale Funnel（公開エンドポイント化）**: CARLA も rosbridge も認証機構を持たないので公開すると即座に乗っ取られる。tailnet 内部限定で運用する

---

## 7. 既知のリスクと注意点

| リスク | 影響 | 対処 |
|---|---|---|
| `usermod -aG video,render` が現セッションに反映されない | `vainfo` や docker での GPU アクセス失敗 | SSH を切って入り直す |
| Apple Silicon Mac で `carla` パッケージが入らない | B が動かない | Rosetta 経由で x86_64 Python を立てる、または C のみで運用 |
| Tailscale の MTU 1280 で大画像トピックが切れる | C で画像が来ない | `foxglove-bridge`（圧縮効率が高い）に切り替える |
| CARLA Python API バージョン不一致 | B で接続即切断 | サーバ・クライアント双方 `0.9.15` で固定 |
| rosbridge が認証なしで起動する | tailnet 外に漏れたら危険 | Funnel を絶対に使わない、ACL で必ず Mac IP のみ許可 |
| iGPU と discrete GPU の取り違え | エンコード／描画性能が低下 | `--device /dev/dri/renderD128`（discrete）を明示 |

---

## 8. このロードマップとリポジトリの関係

- 本ドキュメントが対象とするのは **このキャップストーンの開発・検証ワークフロー**
- 実装が乗るのは主に `docker-compose.yml`（C のサービス追加、CARLA を OffScreen 化）と、Mac 側の Python 環境セットアップ
- アダプタ（CARLA → 本リポの ROS トピック名／型変換）は本ドキュメントの範囲外。`docs/simulator-landscape.md` 7.3 節および `docker/README.md` の「既知の未完成部分」を参照
- 将来 GUI 運用に切り替えた場合の Sunshine ルート再採用は、別途設計判断が要る

---

## 9. 出典

- [CARLA Python API Reference](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA Quickstart / `-RenderOffScreen` フラグ](https://carla.readthedocs.io/en/latest/adv_rendering_options/)
- [carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)
- [rosbridge_suite — ROS Wiki](http://wiki.ros.org/rosbridge_suite)
- [Foxglove Studio](https://foxglove.dev/)
- [foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge)
- [Tailscale ACL Reference](https://tailscale.com/kb/1018/acls)
- [Tailscale MagicDNS](https://tailscale.com/kb/1081/magicdns)
- [Arch Wiki — Hardware video acceleration](https://wiki.archlinux.org/title/Hardware_video_acceleration)（VAAPI 一般情報）
- [LizardByte/Sunshine](https://github.com/LizardByte/Sunshine)（将来採用検討時の参照先）
