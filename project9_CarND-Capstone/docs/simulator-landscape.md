# シミュレータ全体像と学習ロードマップ

PORTABILITY.md は「このキャップストーンのコードを別のシムに移すには何が必要か」だけを扱っている。本ドキュメントはそこから派生して調べた内容を網羅的に整理する：

- このリポジトリが想定していた `dbw_mkz_simulator`（Lincoln MKZ）の現状
- 一般的な車種向けの代替オープンソースシム
- 業界各社がそれらを実際にどこまで使っているか（出資・提携関係を含む）
- 中国 OEM が使っているシムと「ワールドモデル」シフト
- 産業用ロボット（アーム）系のシム
- 現用 PC（AMD Ryzen 5 9600X + RX 7700 XT/7800 XT, NVIDIA なし）での動作可否
- 上記を踏まえた現実的な学習ロードマップ

PORTABILITY.md と重複する部分は要点のみ参照する形にとどめ、本ドキュメントは**周辺事情と判断材料の蓄積**を目的とする。

立てたシムを Mac など別端末から触るためのリモートアクセス構成は [`docs/remote-access.md`](./remote-access.md) に分離している（**ヘッドレス SSH 運用前提でルート B + C を採用、ルート A の Sunshine は見送り**）。

---

## 1. 起点：`dbw_mkz_simulator` の現状

このキャップストーンは Udacity Term 3 シムを起点に書かれているが、Dockerfile が `dbw_mkz`（Dataspeed の Lincoln MKZ ドライバ）を入れているため、移植先として最初に検討されたのが Dataspeed の Gazebo シム `dbw_mkz_simulator` だった。

### 1.1 概要
- **Lincoln MKZ 専用の Gazebo + ROS シミュレータ**
- 実車 ADAS Kit と**同じ CAN メッセージインタフェース**をエミュレート（`can_bus_dbw/can_tx` 等）
- 本リポジトリの `twist_controller/dbw_node.py` が出している `/vehicle/throttle_cmd` `/brake_cmd` `/steering_cmd` を**そのまま受け取れる**（`dbw_mkz_msgs` が共通）
- レーンキープのサンプル（`dbw_mkz_gazebo`）も同梱

### 1.2 入手先
- ソース: Bitbucket の `DataspeedInc/dbw_mkz_simulation`（GitHub ではない）
- バイナリ: `sudo apt-get install ros-$ROS_DISTRO-dbw-mkz-simulator`（Noetic にも配布）
- `dbw_mkz_ros` 本体は ROS Noetic distro に正式リリース済み

### 1.3 現実的な制約
- **Lincoln MKZ は 2020 年で生産終了**。シムの存在意義は維持されているが、対象車両は実質的にディスコン
- 公式に動作確認されている OS／ROS の組み合わせは Indigo (14.04) と Kinetic (16.04) が中心。Noetic 用 deb は配布されているが、Gazebo のバージョン差で多少の調整が要る可能性あり
- 「半オープンソース」と評される領域（内部に一部クローズドな部品）

### 1.4 このキャップストーンへの当てはめ
PORTABILITY.md で「移植コスト最小」と書いた構成。`styx/` を捨てて `roslaunch` を差し替えるだけで `waypoint_updater` 以下の制御スタックは無改造で動く。一方、対象車両がディスコンであることを踏まえると、**学習目的では他のシムを優先したほうが投資効率が良い**というのが本ドキュメントの結論。

---

## 2. 一般車種向けの代替オープンソースシム

### 2.1 CARLA（業界の de facto 共通言語）
- **Unreal Engine ベース、MIT（コード）/ CC-BY（アセット）**、商用利用可
- **車両カタログが豊富**: Tesla（Model 3、Cybertruck）、Audi（A2、E-Tron、TT）、BMW Gran Tourer、Chevrolet、Ford Mustang、Mercedes、Volkswagen、Mini など 30 台以上
- ROS 連携は `carla_ros_bridge` 経由
- **Apollo（百度）との公式ブリッジ** `CARLA-Apollo bridge` が存在、Apollo スタックを CARLA で動かせる
- **SUMO** と co-simulation 可能

### 2.2 AWSIM（Autoware Foundation / TIER IV）
- **Unity ベース**、Apache 2.0（コード）、CC BY-NC（アセット）
- **TIER IV の自動運転車（Lexus RX 改造ベース）**を中心に複数車両モデル＋シーンを同梱
- **ROS 2 ネイティブ**（このリポジトリは ROS 1 Noetic なので `ros1_bridge` 必須）
- Autoware と組み合わせる前提のデジタルツイン
- アセットが CC BY-NC（非商用）なので商用利用は要交渉

### 2.3 OSRF `car_demo`（Toyota Prius）
- ROS Kinetic + Gazebo Classic、**EOL 警告あり**
- Prius モデル＋ 16 ビーム LiDAR、超音波 ×8、カメラ ×4、planar LiDAR ×2
- 素直な ROS インタフェース（`/throttle` `/brake` `/steering` を直接 publish）
- Noetic に上げるならフォークして自前修正の覚悟が要る

### 2.4 CAT Vehicle (`catvehicle`)
- **Ford Escape Hybrid** ベース、ROS Melodic、Gazebo 9
- アリゾナ大学の研究用テストベッド、Ackermann 操舵を物理シムで真面目に解いている
- 本キャップストーンの `dbw_node` の Twist→ステア/スロットルとは相性良好

### 2.5 AutoCarROS2
- ROS 2 + Gazebo 11、汎用四輪モデル
- 個人プロジェクトだがメンテされており、最新スタックで「とりあえず四輪を回す」用途向け
- 特定メーカーモデルではなく簡略化された一般車

### 2.6 SVL / LGSVL（**使用不可**）
- LG が **2022 年 1 月に開発中止**を宣言、リポジトリはアーカイブ済み
- Tesla / BMW などのモデル付きで人気だったが、新規環境構築は現実的でない

### 2.7 比較サマリ

| 用途 | 推奨 |
|---|---|
| 「とりあえず違う車で回す」最短ルート | OSRF `car_demo`（Prius）— ただし Noetic 化は自分で |
| 多様な車種・本格的研究 | CARLA — `carla_ros_bridge` 経由で本リポの制御スタックと接続可能 |
| Autoware 系へ移行 | AWSIM — ROS 2 移行が前提 |

---

## 3. 業界の採用実態と提携関係

### 3.1 結論
**研究・初期開発フェーズでは CARLA / AWSIM が広く使われ、量産バリデーションでは商用クローズドツールに切り替わる**、という二段構成が現状の実態。

### 3.2 CARLA の出資・提携関係

| 関係者 | 内容 |
|---|---|
| Intel | 創設時のキースポンサー（Intel Labs / Computer Vision Center / バルセロナ自治大学） |
| Toyota Research Institute | Computer Vision Center に **$100,000 寄付**（2023） |
| NVIDIA | 公式スポンサー。**Omniverse NuRec** と **Cosmos Transfer** を CARLA 最新版に統合 |
| Synkrotron | スポンサー兼コンソーシアム会員、**中国地域の独占代理店** |
| 運営母体 | Embodied AI Foundation（非営利）＋ Computer Vision Center |
| エコシステム提携 | Ansys、MathWorks、AVL、PTV Group |

NVIDIA は DRIVE Sim を捨てて CARLA に移行したわけではなく、**自社プロプライエタリと CARLA を補完関係として統合**してくる戦略。量産 OEM の最終バリデーションは依然 NVIDIA / Applied Intuition / Foretellix / IPG CarMaker / rFpro / Cognata 等の商用クローズド SIM が主戦場。

### 3.3 AWSIM の関係者

| 関係者 | 立場 |
|---|---|
| TIER IV（東大発、名古屋） | AWSIM の主開発元、Autoware も同社発 |
| Robotec.ai（ポーランド） | AWSIM 共同開発、Autoware Foundation **プレミアムメンバー**、Simulation WG リード |
| Autoware Foundation 創設メンバー（2018） | Apex.AI / Linaro (96Boards) / TIER IV |
| Arm | プレミアムメンバー、Open AD Kit の中核貢献者 |
| メンバー総数 | **31 社以上**（OEM・Tier1・半導体・クラウド系混在） |
| 採用例 | Indy Autonomous Challenge のシム予選で 2024 から AWSIM 採用 |

### 3.4 量産現場で実際に使われているもの（公開情報ベース）

| フェーズ | 主流ツール |
|---|---|
| 早期 R&D / 認識アルゴ訓練 | CARLA・AWSIM・SVL（過去）の合成データ |
| シナリオ網羅・回帰テスト | Foretellix Foretify、Applied Intuition |
| 車両ダイナミクス・ECU SIL/HIL | IPG CarMaker、dSPACE、rFpro、ANSYS AVxcelerate |
| センサ・フォトリアル合成 | NVIDIA DRIVE Sim / Omniverse、Cognata |
| 公開ベンチマーク・論文 | CARLA（実質的な共通言語） |

---

## 4. 中国市場：自社製ワールドモデル＋商用ツールの 4 層構造

中国は他地域と事情が異なり、**OEM が自前のシム／ワールドモデルを持っている度合いが極めて高い**。

### 4.1 第 1 層：OEM 自社製クラウドシム／ワールドモデル（2024〜2026 で本格化）

| メーカー | 自社シム / モデル | 公表スケール |
|---|---|---|
| Huawei (Qiankun ADS 4) | **WEWA**（World Engine + World Behavior Model）。クラウドで diffusion ベースのシナリオ生成 | 高速 L3 で **6 億 km シミュレーション**、レアケースを実走の **1,000 倍密度**で生成 |
| XPeng | **X-World**（VLA 2.0 を支える） | クローズドループシナリオが 3 万 → 50 万（約 1 年）、**1 日あたり 3,000 万 km 相当** |
| NIO | **NWM (NIO World Model)** | 100 ms 以内に **216 シナリオ**同時シミュレーション |
| Li Auto | 世界モデル系（VLA 寄り）に転換 | 北京で L3 公道試験許可取得 |
| Pony.ai | **PonyWorld** | Robotaxi/Robotruck の検証用 |
| BYD（BAS / God's Eye） | 公開情報少。**DeepSeek の AI モデルを統合** | 700 万台超の販売スケールで実走データ収集 |

**重要トレンド**: 従来の「シナリオ書く → CARLA で再生」型から、**ワールドモデルが将来フレームを生成する**型に移行している（Tesla、Huawei 等の "Global World Model War"）。

### 4.2 第 2 層：中国国内の商用シム

| プラットフォーム | 立ち位置 |
|---|---|
| Baidu Apollo Simulator | Apollo の公式シム。**70 以上の OEM/Tier1 と提携**、HD マップ＋実走データ込み |
| Synkrotron OASIS Sim | **CARLA をコアに据えた商用版**。Synkrotron は CARLA governing board の中国唯一メンバー、**OpenSCENARIO 2.0** 完全対応 |
| 51WORLD / 51Sim-One | 北京 51WORLD（旧 51VR）。デジタルツイン路線、ASAM 加盟、SimOne / DataOne / TIM の三本柱 |
| Tencent TAD Sim | WeChat / クラウド資産を背景にしたシム（業界では知られた話） |

実態としては「**Apollo Sim or 51Sim-One or OASIS のどれかを下回りに置きつつ、自社ワールドモデルを上に乗せる**」構成が一般的。

### 4.3 第 3 層：CARLA ／オープンソース系
- 中国での流通は Synkrotron 経由が主
- 学術研究や Tier1 のコンポーネント開発で使用
- Apollo は CARLA-Apollo bridge を公式維持
- OEM 量産プロセスのコアではなく、論文・ベンチマーク・初期 R&D のレイヤー

### 4.4 第 4 層：海外勢
- **NVIDIA Thor チップ**: NIO / XPeng / Li Auto / BYD 全社が 2025 量産で採用予定
- **DRIVE Sim / Cosmos**: 上記の延長で選択肢に入っている
- **Foretellix / Applied Intuition / IPG CarMaker**: 国際 OEM と協業する場合に使用

### 4.5 中国らしさ
1. **垂直統合志向が強い**: OEM が自社チップ（NIO Shenji NX9031、XPeng Turing 等）＋ 自社ワールドモデル＋ 自社運転スタックを揃える
2. **ワールドモデル＝シムの主戦場**: 「物理 + 3D アセット」型から「実走データから将来フレームを生成する生成 AI 型シム」へ
3. **DeepSeek の波及**: BYD の例のように、国産 LLM／世界モデル基盤がシム側にも流れ込む構造

---

## 5. 産業用ロボット（アーム）系のシム

「アーム系」と一口に言っても就職市場は大きく 2 系統に割れ、学習用シムの選び方が変わる。

### 5.1 系統 A：AI／RL／Physical AI 系
2025〜2026 で **NVIDIA Isaac エコシステムが事実上の標準**化。

| ツール | 役割 | 学習価値 |
|---|---|---|
| **NVIDIA Isaac Sim**（Omniverse / OpenUSD） | 物理シム＋合成データ生成 | ★★★ 業界の現在地 |
| **Isaac Lab**（Isaac Sim 上の RL/IL/MP フレームワーク） | GPU 加速の RL／模倣学習／motion planning | ★★★ 求人で名指し急増 |
| **Newton 1.0**（GTC 2026 GA、Warp + OpenUSD） | dexterous manipulation 用物理エンジン | ★★ 今後の本命 |
| **MuJoCo + MuJoCo Playground**（DeepMind） | 接触リッチな RL の研究標準 | ★★ 論文を読むなら必須 |

実績の例: **UR10e（Universal Robots 実機）でギア組み立てを Isaac Lab で RL 訓練 → ゼロショットで実機に転送成功**（NVIDIA 公式事例）。

### 5.2 系統 B：工場自動化／SI 系
**実機メーカーの純正ツールが使えること**が直接の採用条件。

| シム / ツール | メーカー | 立ち位置 |
|---|---|---|
| **RobotStudio** | ABB | RAPID 言語編集環境込み、**個人利用無料** |
| **ROBOGUIDE** | FANUC | 工場で最強の存在感、**有償（〜$2,500/年）**、日本では FANUC 案件が圧倒的 |
| **KUKA.Sim** | KUKA | 欧州系自動車工場で強い |
| **MotoSim** | Yaskawa | 国内・東南アジアで強い |
| **RoboDK**（横断的） | 独立系 | 複数メーカーの実機をまたいでオフラインプログラミング、**Linux 対応** |

求人実態: 「FANUC 認定ロボットプログラマ」「ROBOGUIDE 経験」のように**ベンダー名で書かれる**。

### 5.3 中間レイヤー：ROS 2 + MoveIt 2 + Gazebo（両系統の共通言語）
- **MoveIt 2** は manipulation のオープンソース de facto 標準
- 採用企業: Fetch、Franka Emika、PAL、Kinova、Realtime Robotics 等
- **Pilz industrial motion planner**（直線・円弧の決定論的軌道）が組み込まれている
- 産業向け用途にも実用される

### 5.4 系統別の推奨
| 目的 | 推奨 |
|---|---|
| 学習用「1 つだけ」選ぶなら | **Isaac Sim + Isaac Lab**（2026 時点で求人で最も具体的に名指し） |
| 就職市場の幅を広げたい | **Isaac Lab + MoveIt 2（ROS 2）の二刀流** |
| 国内 FA・工場 SI 路線 | **RobotStudio（無料）→ 案件に応じて ROBOGUIDE/KUKA.Sim** |

---

## 6. 現用 PC での動作可否

### 6.1 現スペック
```
CPU : AMD Ryzen 5 9600X (6コア/12スレッド, Zen 5, ブースト 5.48GHz)
RAM : 30 GiB
GPU : AMD Radeon RX 7700 XT / 7800 XT (Navi 32, RDNA 3)
SSD : 空き 126 GB / 215 GB
OS  : Ubuntu 24.04.3 LTS (Noble)
NVIDIA : なし
```

CPU・RAM・GPU 性能はモダンで強い構成だが、**「NVIDIA GPU が無い」が最大の制約**。

### 6.2 動作可否マトリクス

#### ❌ そのままでは動かない（NVIDIA 必須）
| シム | 理由 |
|---|---|
| NVIDIA Isaac Sim / Isaac Lab | Omniverse RTX レンダラ＋CUDA。**RTX 30/40/50 系＋VRAM 8GB 以上必須**。AMD では起動不可 |
| NVIDIA DRIVE Sim / Cosmos / Newton | Warp（CUDA）と OptiX 前提 |
| MuJoCo Playground の Madrona renderer (GPU 経路) | CUDA 必須。MuJoCo 本体は動くが GPU 加速学習は不可 |

→ Physical AI 路線（Isaac Lab）を本気で目指す場合、**NVIDIA RTX GPU の追加 or クラウド GPU（Lambda Labs / RunPod 等）**が事実上の前提。

#### ⚠️ 動くが工夫が必要
| シム | 状況 |
|---|---|
| CARLA | Unreal Engine 4/5 ベースで Vulkan 経由で **AMD GPU で動く**。RX 7800 XT は推奨スペック超え。**ただし公式サポート OS は Ubuntu 22.04**、24.04 は要動作確認 → **22.04 Docker 経由が定石** |
| AWSIM | Unity (Linux) で **AMD でも動く**。公式推奨 RTX 2080+ だが RX 7800 XT は性能上問題なし。Ubuntu 24.04 は公式外 |
| Apollo + CARLA-Apollo bridge | CARLA が動けば OK |

#### ✅ そのまま快適に動く
| シム | 状況 |
|---|---|
| ROS 2 Jazzy（Ubuntu 24.04 ネイティブ）+ MoveIt 2 + Gazebo Harmonic | スペック過剰なほど余裕、OS との相性最良 |
| MuJoCo（CPU パス）+ MuJoCo Playground（CPU 環境） | オープンソースの MuJoCo 自体は **GPU 不要** |
| OSRF car_demo / catvehicle / AutoCarROS2 | Gazebo Classic 系、CPU バウンド |
| 本キャップストーン（ROS Noetic Docker） | 動作確認済み |

#### ❌ OS が違う（Linux で動かない）
| ツール | 必要 OS |
|---|---|
| ABB RobotStudio | Windows のみ |
| FANUC ROBOGUIDE | Windows のみ |
| KUKA.Sim | Windows のみ |
| Yaskawa MotoSim | Windows のみ |
| **RoboDK** | **Windows / Linux / macOS 全対応**（学習効率◎） |

→ ベンダー系シムは **Windows デュアルブート or VirtualBox/VMware** が要る。学習目的なら **RoboDK 一本で複数メーカーの実機をまたいで触れる**。

### 6.3 ストレージの目安
空き 126 GB に対する各シムのインストールサイズ:

- Isaac Sim（アセット込み）: **~100 GB**
- CARLA: ~30 GB（マップ込み）／ Unreal ビルド付き ~80 GB
- AWSIM: ~5 GB
- ROS 2 Jazzy + MoveIt 2 + Gazebo: ~5 GB
- MuJoCo: ~500 MB
- 本キャップストーン（Docker イメージ）: ~5 GB

→ Isaac 系本格運用前に SSD 増設の検討を推奨。CARLA + AWSIM + ROS 2 + MoveIt なら 50 GB 以内で全部入る。

---

## 7. 学習ロードマップ（現スペック前提）

### 7.1 自動運転寄りの場合
| 順序 | 何を学ぶ | 動作可否 |
|---|---|---|
| 1 | このキャップストーンを最後まで Docker で動かす | ✅ |
| 2 | CARLA（Ubuntu 22.04 Docker）+ `carla_ros_bridge` | ⚠️ Docker 経由で確実 |
| 3 | AWSIM（ROS 2 Jazzy ネイティブで） | ⚠️ 24.04 動作要確認 |

### 7.2 産業用ロボット寄りの場合（推奨優先順）
| 優先度 | 何を学ぶ | 動作可否 |
|---|---|---|
| ★★★ | ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic | ✅ ネイティブ最速 |
| ★★★ | MuJoCo + MuJoCo Playground (CPU) | ✅ GPU 不要 |
| ★★ | RoboDK（FA 寄り入門） | ✅ Linux 対応 |
| 保留 | Isaac Sim / Isaac Lab | ❌ NVIDIA 必須。クラウド GPU で代替するか、本格化のタイミングで GPU 追加 |

### 7.3 自動運転で身につけた知識の転用範囲
このキャップストーンを通じて得られる以下のスキルは、産業用ロボット側でもそのまま活きる:

- ROS／Python／Docker
- センサ統合、PID、状態推定
- catkin / colcon ビルドシステム
- launch ファイル、tf 座標変換

新規学習負荷:
- **接触ダイナミクス**（剛体衝突、摩擦） → MuJoCo のチュートリアルで補完
- **運動学**（IK／DH パラメータ） → MoveIt 2 のチュートリアルで補完

---

## 8. 出典

### 8.1 dbw_mkz_simulator
- [dbw_mkz - ROS Wiki](http://wiki.ros.org/dbw_mkz)
- [dbw_mkz_can - ROS Wiki](http://wiki.ros.org/dbw_mkz_can)
- [DataspeedInc / dbw_mkz_ros (Bitbucket)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/)
- [ROS Package index: dbw_mkz](https://index.ros.org/p/dbw_mkz/)

### 8.2 オープンソース AV シム
- [CARLA Simulator (公式)](https://carla.org/)
- [CARLA Vehicle Catalogue](https://carla.readthedocs.io/en/latest/catalogue_vehicles/)
- [tier4/AWSIM (GitHub)](https://github.com/tier4/AWSIM)
- [autowarefoundation/AWSIM-Labs (GitHub)](https://github.com/autowarefoundation/AWSIM-Labs)
- [osrf/car_demo (GitHub)](https://github.com/osrf/car_demo)
- [Catvehicle ドキュメント](https://jmscslgroup.github.io/catvehicle/)

### 8.3 業界採用と提携
- [Toyota Research Institute Supports Development of Open-Source Automated Driving Simulator](https://global.toyota/en/newsroom/corporate/23017368.html)
- [Toyota Invests in CARLA Open Source AV Simulator Project — TU-Automotive](https://www.tu-auto.com/toyota-invests-in-carla-open-source-av-simulator-project/)
- [CARLA democratizes autonomous vehicle R&D — Unreal Engine Spotlight](https://www.unrealengine.com/en-US/spotlights/carla-democratizes-autonomous-vehicle-r-d-with-free-open-source-simulator)
- [CARLA Ecosystem](https://ecosystem.carla.org/)
- [NVIDIA — CARLA Simulator (公式統合ドキュメント)](https://carla.readthedocs.io/en/0.9.16/nvidia/)
- [AWSIM: End-to-End Digital Twin Simulation Platform — Autoware](https://autoware.org/awsim-end-to-end-digital-twin-simulation-platform/)
- [The Autoware Foundation bolsters partnership with Arm via the Open AD Kit](https://autoware.org/arm-awf-partnership-on-open-ad-kit/)
- [Indy Autonomous Challenge to Relaunch SIM Races with Autoware Foundation, Autonoma, TIER IV](https://www.robotics247.com/article/indy_autonomous_challenge_relaunching_autonomous_challenge_sim_races_autoware_foundation_autonoma_tier_iv)

### 8.4 中国市場
- [Huawei ADS 4 Lands Big — World Model gives vehicles a "predictive brain"](https://chinaevinsights.com/2025/09/01/huawei-ads-4-lands-big-world-model-gives-vehicles-a-predictive-brain/)
- [Huawei's Qiankun ADS autopilot system closes in on 10 billion km](https://www.globalchinaev.com/post/huaweis-qiankun-ads-autopilot-system-closes-in-on-10-billion-km-in-china)
- [Tesla, Huawei, and New Entrants Engage in a Decisive Battle: The Global World Model War — 36kr](https://eu.36kr.com/en/p/3463098292770179)
- [XPeng's X-World powers VLA 2.0 driving simulation rollout — Automotive World](https://www.automotiveworld.com/news/xpengs-x-world-powers-vla-2-0-driving-simulation-rollout/)
- [How do Nio, Xpeng, Li Auto differ in their AI approaches? — CnEVPost](https://cnevpost.com/2026/01/19/how-do-nio-xpeng-li-auto-differ-in-ai-approaches/)
- [BYD shares hit record after rolling out driver assistance with DeepSeek's AI — CNBC](https://www.cnbc.com/2025/02/11/byd-rolls-out-driver-assist-tech-for-evs-with-deepseeks-ai-help.html)
- [Baidu Apollo Open Source Autonomous Driving Platform — Neousys](https://www.neousys-tech.com/en/core-technologies/fanless-in-vehicle-pc/baidu-apollo-open-source-autonomous-driving-platform)
- [Synkrotron simulation solutions — CARLA documentation](https://carla.readthedocs.io/en/latest/ecosys_synkrotron/)
- [About 51Sim](https://www.51sim.com/about/index)
- [Pony AI Update: Robotaxis and Robotruck Services — EE Times](https://www.eetimes.com/pony-ai-update-robotaxis-and-robotruck-services/)

### 8.5 産業用ロボット
- [Isaac Sim — Robotics Simulation and Synthetic Data Generation (NVIDIA)](https://developer.nvidia.com/isaac/sim)
- [NVIDIA Isaac Lab Open-Source Modular Framework](https://developer.nvidia.com/isaac/lab)
- [Bridging the Sim-to-Real Gap for Industrial Robotic Assembly Using NVIDIA Isaac Lab](https://developer.nvidia.com/blog/bridging-the-sim-to-real-gap-for-industrial-robotic-assembly-applications-using-nvidia-isaac-lab/)
- [Newton — Contact-Rich Manipulation and Locomotion Capabilities for Industrial Robotics](https://developer.nvidia.com/blog/newton-adds-contact-rich-manipulation-and-locomotion-capabilities-for-industrial-robotics/)
- [moveit/moveit2 (GitHub)](https://github.com/moveit/moveit2)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [google-deepmind/mujoco (GitHub)](https://github.com/google-deepmind/mujoco)
- [google-deepmind/mujoco_playground (GitHub)](https://github.com/google-deepmind/mujoco_playground)
- [Off-line Programming — RoboDK Blog](https://robodk.com/blog/off-line-programming/)
- [KUKA vs. FANUC: Which industrial robot brand is right for you in 2026? — Standard Bots](https://standardbots.com/blog/kuka-vs-fanuc)

### 8.6 PC 動作確認関連
- [Isaac Sim System Requirements — NVIDIA Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html)
- [CARLA Quickstart / System Requirements](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [ROS 2 Jazzy — Supported Platforms](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [RoboDK — Download / Supported Platforms](https://robodk.com/download)
