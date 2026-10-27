# 過去のキネマティック配送の記録

現在の提出デモは[コンベヤ色選別](docs/07_CONVEYOR_SORTING.md)です。この文書の旧方式や実験件数は選別デモの結果ではありません。

この文書の起動例は旧デモ用です。`./run_demo.sh`は現在、物理配送を起動します。以下の旧デモを再現する場合は`./run_legacy_demo.sh`または`DELIVERY_MODE=legacy ./run_demo.sh`を使ってください。

# 学習型色認識を用いたCRANE+配送タスク

ROS 2 Humble / Gazebo Classicのカメラから青い物体を検出し、CRANE+で発送箱へ配送します。認識には教師あり学習済みのOpenCV決定木を使用します。既存のACT・ブラウザ版プロジェクトは変更していません。

## 物理把持の学習実験（追加）

MuJoCo上で、CRANE+の指との接触・摩擦だけで青い箱を把持する実験を追加しました。学習した方策は未学習条件30件で30件成功し、指を開いた比較条件は0件でした。これは単体把持の専用環境での結果です。実行方法・動画・条件・制限は[physics_grasp/README.md](physics_grasp/README.md)を参照してください。標準Gazebo起動は引き続き以下のキネマティック方式です。

## 検証済みの範囲

- `simple_delivery.world`：青1個を箱へ配送し、赤い誤選別物体を元の位置に残す動作を録画・検証済み。
- 関節の到達確認後に状態を進め、未到達では`error`で停止します。
- `two_target_delivery.world`：青2個の連続配送を試験しましたが、箱への移送時に関節未到達で停止しました。実験用です。
- `multi_color_delivery.world`：10色20個の旧実験シーン。配置の重なり・到達性が未解決であり、提出用デモには使用しません。

把持は**キネマティック方式**です。静的な対象モデルを手先に追従させ、解放時に指定スロットへ配置します。接触力による物理把持、衝突回避経路計画、実機動作は検証していません。対象の識別は画像を使い、動作に用いる3次元位置はGazeboのモデル座標を使います。

## 起動

必要環境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic、MoveIt 2、ros2_control、CRANE+ description/control/moveit_config、OpenCV、cv_bridge、NumPy、PyYAML、colcon。Gazeboのカメラには描画可能なX環境が必要です（GUI非表示でもカメラのレンダリングは必要）。講義環境の`/home/ubuntu/airobot_ws`を既定の依存ワークスペースにしています。

```bash
cd final_project_ros
./run_demo.sh
# Gazeboの操作画面を表示しない場合
./run_demo.sh gui:=false
# CRANE+環境の場所が異なる場合
CRANE_WS=/path/to/airobot_ws ./run_demo.sh gui:=false
```

前回のlaunchをCtrl+Cで終了してから再起動してください。ワールドの変更は再起動後に反映されます。`target_color`は10色を指定できますが、検証済み単体ワールドの対象はblueだけです。存在しない色では配送を実行しません。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /task_state
ros2 topic echo /target_point
ros2 control list_controllers
```

正常時：`initializing → waiting_for_object → target_detected → approaching_target → grasping → closing_gripper → lifting → placing_in_box → releasing → done`。

## 把持までの動作の最適化

標準起動では深く折り畳むホーム姿勢を経由せず、現在の関節位置から対象上方へ接近します。接近・下降は速度と加速度を制限した5次補間で、各区間の始点・終点の速度と加速度をゼロにします。移動量から区間時間を計算し、下降には低い上限を使います。軌道終了時刻を過ぎてから、関節誤差0.10 rad以内を3サンプル確認して次へ進みます。搬送・配置には従来の軌道方式を使用します。

```bash
# ゆっくり接近・下降する例（関節速度rad/s、加速度rad/s²）
./run_demo.sh approach_max_velocity:=0.6 approach_max_acceleration:=1.0 \
  descent_max_velocity:=0.25 descent_max_acceleration:=0.5
# 起動時のホーム動作が必要な場合
./run_demo.sh startup_homing:=true
```

標準値は接近0.8 rad/s・1.4 rad/s²、下降0.35 rad/s・0.7 rad/s²です。これらは指令軌道の上限で、接触やGazeboの振動を含む実測値の保証ではありません。今回の単体シーンの初期姿勢で検証しており、任意配置の衝突回避を最適化したものではありません。

同じ単体ワールドで各1回計測し、初期化から把持完了（liftingへ移行）までが**9.43秒 → 6.03秒（シミュレーション時間、約36%短縮）**になりました。両方とも配送完了を確認しています。フィードバックの累積関節変動には高周波の振動が含まれるため、経路長の改善率としては使用しません。

- `artifacts/motion_optimization/comparison.mp4`：変更前後の比較。開始を揃え、速度は変更せず、先に完了した側の末尾を静止表示します。
- `artifacts/motion_optimization/after.mp4`：最適化後の配送全体。
- `artifacts/motion_optimization/comparison.json`：時間の比較と計測条件。
- `tools/compare_motion.py`：比較動画と集計の再生成。

## AIと座標の扱い

`learned_color.py`はRGBを入力として10色＋背景を分類する決定木です。合成色データ8,000件で学習し、異なる乱数シードの3,200件で99.84%を得ました。これは合成色データ上の結果であり、実画像全般の精度ではありません。モデルと評価結果は`ros2_ws/src/final_project_ros/models/`に格納しています。

推論マスクの輪郭から複数候補を検出します。`/target_point`は**画像のピクセル座標**であり、メートル単位ではありません。制御側はCameraInfoとワールドに固定された下向きカメラ（位置0.55, 0.15, 1.4 m）で画像とモデルを対応付け、18ピクセル以内の未配送対象のみ選択します。カメラ姿勢を変更する場合はこの投影式も更新してください。

MoveIt `/compute_ik`へ対象の方位に応じたyawを指定します。軌道は各JointTrajectoryControllerへ直接送信します。学習した方策によるアーム制御やMoveItの衝突回避経路計画は実装していません。既存5次元ACTは本タスクに使用していません。

HSV認識の関数は比較用に残していますが、標準launchの認識ノードは学習済み決定木を使用します。旧`color_pick_demo`は過去の独立デモで、標準launchから起動しません。`/task_state`は標準launchでは配送制御ノードだけが発行します。

## 再学習とテスト

```bash
python3 tools/train_color_model.py
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" python3 -m unittest discover -s tests -v
```

10件のテストで複数の青と赤の分離、空画像、微小ノイズ、関節未到達時の停止、実モデル位置による配置判定、補間軌道の速度・加速度・端点条件、移動量に応じた時間調整、軌道終了前の遷移抑制を確認します。

## 録画と提出物の再生成

```bash
./run_demo.sh gui:=false record_demo:=true record_output:="$PWD/artifacts/demo_raw.mp4"
# doneになり録画ノードが終了したらlaunchをCtrl+Cで終了
python3 tools/prepare_submission.py
```

録画ノードは実際のGazeboカメラ画像と関節値・状態を記録し、終了時に同名のJSONへ状態履歴と最終モデル位置を保存します。最大170秒、done/error後4秒で終了します。録画時間の都合上、極端に低速な環境では録画を拡張してください。

提出物生成にはFFmpeg、Pillow、ReportLab、日本語Noto CJKフォントが必要です。録画素材は実時間で再生し、冒頭12秒・末尾10秒の説明画面を追加します。成功した最適化後の録画を優先し、なければ青2個の成功録画、従来の単体録画の順に選びます。成功JSONのない録画を完成品として生成しません。

生成先`submission/`：

- `PAI最終課題_アカウントID.mp4`：説明付き提出用動画（1〜3分に収まることを確認）。
- `説明資料_PAI最終課題_アカウントID.pdf`：日本語レポート。
- `source_code.zip`：ソース・学習済みモデル・再現ツール。依存するCRANE+等は上記環境へ別途導入。
- `verification.json`：録画に対応する実測結果と合成分類評価。

`アカウントID`を自身のOmniCampusアカウント名またはIDに置き換えてください。公開GitHubへソースを配置し、YouTubeへ動画をアップロードして、両URLとPDFをOmniCampusに提出します。このプロジェクトからの外部公開・提出は実施していません。詳細は元資料`doc/最終課題（10_19 改訂版）.pdf`を参照してください。

## ファイル

- `ros2_ws/src/final_project_ros/`：ROSパッケージ、ワールド、学習済みモデル。
- `tools/`：再学習と提出物生成。
- `tests/`：認識・制御の検証。
- `artifacts/`：実測録画・JSON・検証ログ。
- `FINAL_REPORT.md`：実測記録から生成するレポート原稿。
