# 学習型色認識と接触把持によるCRANE+コンベヤ選別

GitHubでの管理対象と別PCでの環境再現は[GITHUB.md](GITHUB.md)を参照してください。

現在の提出デモは、青→赤→緑→青の4箱を1個ずつ供給し、停止後にカメラ画像で色を判断する連続選別です。青は黄色い配送箱、それ以外は反対側の灰色の回収箱へ投入します。4箱で自動終了します。

```bash
cd /home/ubuntu/final_pj/final_project_ros
DELIVERY_MODE=sorting ./run_demo.sh gui:=true
# GUIなし（カメラ描画用の有効なDISPLAYは必要）
DELIVERY_MODE=sorting ./run_demo.sh gui:=false
```

必要環境はROS 2 Humble、Gazebo Classic、既存CRANE+ワークスペース、SciPy等です。既定のCRANE+環境は`/home/ubuntu/airobot_ws`で、異なる場合は`CRANE_WS`を設定します。[起動・環境の説明](PHYSICAL_DELIVERY.md)を参照してください。

コンベヤは供給中だけGazeboの位置設定サービスで箱を少しずつ動かす模擬搬送です。停止後は位置設定を使わず、学習した把持姿勢と指の接触・摩擦によって持ち上げ・運搬・投入します。色分類は検査位置の画像を使い、モデル名や正解ラベルから行き先を決めません。3次元位置と検証にはGazebo座標を使います。

最終版の通し検証1試行で4箱の判定・投入と終了時の保持を確認しました（約181秒）。これは独立4試行の成功率ではありません。無限供給・任意個数・ベルト摩擦駆動・未知環境・実機は未検証です。失敗時は記録を保存し、非ゼロで終了します。

- [実行動画](artifacts/conveyor_sorting/episode.mp4) ／ [終了画像](artifacts/conveyor_sorting/episode.png) ／ [検証JSON](artifacts/conveyor_sorting/episode.json)
- [提出物一覧](submission/README.md) ／ [説明資料の原稿](FINAL_REPORT.md)
- [選別デモの設計・パラメータ・失敗と再現](docs/07_CONVEYOR_SORTING.md)
- [開発・学習ドキュメント](docs/README.md) ／ [詳細ガイドPDF](docs/開発・学習ガイド.pdf)
- [把持学習と再学習](physics_grasp/README.md)

## テストと提出物の再生成

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" python3 -m unittest discover -s tests -v
python3 tools/document_experiments.py
python3 tools/build_development_docs.py
python3 tools/prepare_submission.py
```

テスト16件が通過しています。提出生成は`artifacts/conveyor_sorting/episode.json`と同名の録画・画像を使い、色・行き先・物理検証・終了時保持を検証します。失敗した選別結果を旧デモの成功で置き換えません。`submission/`に説明付き動画・日本語PDF・Markdown・ソースZIP・検証JSONを生成します。`アカウントID`は提出時に自身のIDへ変更してください。外部公開・提出は行っていません。

## 単一箱配送と過去の方式

`./run_demo.sh`の既定は従来の単一箱物理配送です。青い箱1個を配送し、赤い物体を残します。記録は`artifacts/physical_delivery/`にあり、標準と別条件の計4試行・開指対照はこの単一箱モードの検証です。選別デモの検証件数には含めません。

```bash
DELIVERY_MODE=physical ./run_demo.sh gui:=false
DELIVERY_MODE=legacy ./run_demo.sh gui:=false
```

キネマティック方式の旧デモは[LEGACY_DEMO.md](LEGACY_DEMO.md)を参照してください。過去の比較値や参照スナップショットは当時の記録として残しています。
