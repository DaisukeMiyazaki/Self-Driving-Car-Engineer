# Docker Compose 構成（CARLA 接続用、ヘッドレス前提）

`docker-compose.yml` を 4 サービス構成で立ち上げる。常時 SSH で運用する Linux サーバ向け。詳しい背景は [`docs/remote-access.md`](../docs/remote-access.md) を参照。

```
┌───────────────┐  ┌──────────────────┐  ┌─────────────────┐  ┌────────────────┐
│ CARLA Server  │←→│ carla_ros_bridge │←→│ Capstone (ROS)  │  │ rosbridge      │
│ -RenderOff-   │  │ ROS Noetic       │  │ ROS Noetic      │  │ WebSocket :9090│
│ Screen, AMD   │  │ /carla/* 配信    │  │ catkin_make 済  │  │ Foxglove 接続  │
└───────────────┘  └──────────────────┘  └─────────────────┘  └────────────────┘
   Vulkan(GPU 計算)        roscore @ :11311          ←─Mac browser via Tailscale
   :2000 (CARLA RPC)
   ←─Mac の CARLA Python クライアント via Tailscale
```

## 前提

- Ubuntu Linux ホスト前提（`network_mode: host` を使うため Mac/Windows では動かない）
- AMD GPU 用ドライバ:
  ```bash
  sudo apt install mesa-va-drivers vainfo vulkan-tools mesa-vulkan-drivers
  vainfo --display drm --device /dev/dri/renderD128 | grep EncSlice
  ```
- ユーザを GPU グループに追加（**SSH を入り直すまで反映されない**）:
  ```bash
  sudo usermod -aG video,render $USER
  exit   # その後 SSH しなおす
  ```

## 起動

```bash
docker compose up -d                # 全サービス起動
docker compose ps                   # 状態確認
docker compose logs carla -f        # CARLA のログ追跡
docker compose exec capstone bash   # capstone コンテナにログイン
docker compose down                 # 停止
```

## Mac から接続する

Tailscale で同一 tailnet に入った状態で：

| 用途 | 接続方法 |
|---|---|
| **CARLA Python API**（運転、spectator camera で UE4 風ビュー） | `pip install carla==0.9.15` → `carla.Client('linux-host.tailnet:2000')` |
| **ROS トピック可視化** | Chrome で [https://app.foxglove.dev/](https://app.foxglove.dev/) → `ws://linux-host.tailnet:9090` |

CARLA は `-RenderOffScreen` で動いているのでサーバ側に画面は出ない。Mac の Pygame で受け取って描画するか、Foxglove で ROS データを見る運用。

## 既知の未完成部分

**CARLA → capstone のトピック変換アダプタは未実装。** 現状は配管だけ通っている：

- `carla_ros_bridge` が出すトピック例: `/carla/ego_vehicle/odometry`（`nav_msgs/Odometry`）、`/carla/ego_vehicle/rgb_front/image`（`sensor_msgs/Image`）
- 本リポの `waypoint_updater` 等が期待するトピック: `/current_pose`（`geometry_msgs/PoseStamped`）、`/current_velocity`（`geometry_msgs/TwistStamped`）、`/image_color`、`/base_waypoints`

メッセージ型が違うので単純な remap では済まず、**~200 行程度の Python アダプタノード**が必要。`docs/simulator-landscape.md` 7.3 節も参照。

## トラブルシュート

| 症状 | 対処 |
|---|---|
| CARLA が `Vulkan failed to find GPU` | `vainfo --display drm --device /dev/dri/renderD128` で discrete GPU が見えるか確認、`group_add: [video, render]` がコンテナに効いているか確認 |
| capstone コンテナがビルド失敗 | ホストの `__pycache__` を `rm -rf` してから再 build |
| ROS マスタに繋がらない | 各コンテナで `network_mode: host` が効いているか確認（Linux ホスト必須） |
| Foxglove が `ws://...:9090` に接続できない | `docker compose logs rosbridge` でロード状況確認、Tailscale ACL で 9090 が許可されているか確認 |
| Mac の `carla` パッケージが入らない | Apple Silicon の場合は Rosetta 経由で x86_64 Python を立てる |
