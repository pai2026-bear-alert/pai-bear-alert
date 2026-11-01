# bear_alert：ヒグマを検知すると行動するロボットアーム

Physical AI 応用1 最終課題。

動画からYOLOでクマを検出し、ROS 2のトピック `/bear_alert` を通じて、Gazebo上のロボットアーム CRANE+ V2 に警告の動作（グリッパを開き、アームを左右に振る）をさせます。

- デモ動画：（YouTubeのURL）

## システム構成

```
動画ファイル ──▶ bear_detector ──/bear_alert (std_msgs/Bool)──▶ bear_warning ──▶ CRANE+ V2（Gazebo）
                 YOLOv8n（COCO学習済み）                         アーム：crane_plus_arm_controller/joint_trajectory
                 クマ（class 21）だけを検出                       グリッパ：crane_plus_gripper_controller/joint_trajectory
```

| ノード | 役割 |
|---|---|
| `bear_detector` | 動画を実時間で再生しながら、YOLOでクマを検出する。検出結果を `/bear_alert` に True/False で送り続け、枠付きの映像をウィンドウに表示する |
| `bear_warning` | `/bear_alert` を受け取る。True になったらグリッパを開いてアームを左右に振り、False になったら元の姿勢に戻ってグリッパを閉じる |

検出側と動作側をトピックで分けているので、動作側をスピーカーや実機などに差し替えられます。

## 工夫した点

- **クマにだけ反応する**：YOLOの対象をCOCOの `bear` クラスに絞り、キツネなど他の動物には反応しないことをデモで示した。
- **誤検出とちらつきへの対策**：2フレーム連続で検出したら警告を開始し、動画の時刻で1秒間クマが見えなければ解除する。CPU推論が遅くても判定が崩れないよう、フレーム数ではなく動画の時刻で判定している。
- **安全装置**：`bear_warning` は、`/bear_alert` が3秒届かなければ警告を解除する。検出側が異常終了しても、アームが振り続けない。`bear_detector` も終了時に False を送る。
- **Gazeboの動作速度に合わせた動き**：グリッパは警告中は開いたままにし、アームは1.5秒ごとに左右へ振る。命令の間隔が短すぎると、シミュレーション上でほとんど動かないため。

## 動作環境

- Docker イメージ：`airobotbook/ros2-desktop-ai-robot-book-humble`（ROS 2 Humble、noVNCでデスクトップを表示）
- CRANE+ V2 の Gazebo 環境：`crane_plus_gazebo`（講義で使用したもの）
- Python 3.10、ultralytics 8.3.74（YOLOv8n）

## セットアップ

コンテナ内で実行します。

```bash
# ultralytics（numpy 2系になるとROS側が壊れるおそれがあるため固定する）
pip install --user ultralytics "numpy<2" "opencv-python<4.12"

# パッケージを配置してビルド
cp -r bear_alert ~/pai_ws/src/
cd ~/pai_ws
colcon build --symlink-install --packages-select bear_alert
source install/setup.bash
```

## 実行

3つのターミナルで、この順番に起動します。

```bash
# 1. Gazebo と CRANE+ V2
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py

# 2. 警告ノード（起動したままにする）
ros2 run bear_alert bear_warning

# 3. 検出ノード（動画のパスを指定）
ros2 run bear_alert bear_detector --ros-args -p video_path:=/home/ubuntu/videos/bear_demo_640.mp4
```

検出ウィンドウを閉じると `bear_detector` は終了し、アームは元の姿勢に戻ります。

### 主なパラメータ

| ノード | パラメータ | 既定値 | 内容 |
|---|---|---|---|
| bear_detector | `video_path` | `/home/ubuntu/videos/bear.mp4` | 入力する動画 |
| | `model` | `yolov8n.pt` | YOLOのモデル |
| | `conf` | 0.4 | 検出の信頼度のしきい値 |
| | `imgsz` | 416 | 推論の画像サイズ（小さいほどCPUで速い） |
| | `on_frames` | 2 | 警告を開始する連続検出フレーム数 |
| | `off_seconds` | 1.0 | 警告を解除するまでの、クマが見えない時間（動画時刻） |
| | `loop` | False | 動画を繰り返し再生するか |
| bear_warning | `step` | 1.5 | アームを1回振る時間 [s] |
| | `timeout` | 3.0 | 信号が途絶えたら警告を解除するまでの時間 [s] |
| | `gripper_open` / `gripper_close` | 0.0 / 0.6 | グリッパの開／閉（約0.67で閉じきる。これを超える値は送らない） |

## デモ動画の入力素材

動画ファイルはリポジトリに含めていません。以下のフリー素材を使用し、キツネ → クマ → キツネの順に連結して、640×360に縮小しました。

- クマ：（pixabay　https://pixabay.com/videos/animal-nature-wildlife-mammal-pet-319733/）
- キツネ：（pixabay　https://pixabay.com/videos/fox-animal-nature-wildlife-winter-319743/）

連結と縮小の例：

```bash
ffmpeg -i fox.mp4 -i bear.mp4 -filter_complex \
  "[0:v]trim=0:6,setpts=PTS-STARTPTS,scale=640:360,fps=30,setsar=1,split[a][c]; \
   [1:v]trim=0:15,setpts=PTS-STARTPTS,scale=640:360,fps=30,setsar=1[b]; \
   [a][b][c]concat=n=3:v=1:a=0" -an bear_demo_640.mp4
```

## ライセンス

[ultralytics](https://github.com/ultralytics/ultralytics)（AGPL-3.0）を使用しているため、このリポジトリも AGPL-3.0 とします。詳しくは [LICENSE](LICENSE) を参照してください。
