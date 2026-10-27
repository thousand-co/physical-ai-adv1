# 学習した把持を使うCRANE+物理配送

ROS 2 Humble / Gazebo Classic上で、青い箱を指の接触・摩擦で掴み、黄色い配送箱へ運び、指を開いて投入します。学習した把持パラメータを配送制御へ組み込み、配送後の物体位置と接触まで確認します。

```bash
cd /home/ubuntu/final_pj/final_project_ros
./run_demo.sh gui:=false
# Gazebo画面も表示する場合
./run_demo.sh gui:=true
```

実行は自動終了し、`artifacts/physical_delivery/episode.json`と同名の動画・画像を保存します。失敗時は非ゼロで終了します。必要環境・設定・再現方法は[PHYSICAL_DELIVERY.md](PHYSICAL_DELIVERY.md)を参照してください。カメラ描画のため、有効なX環境と`DISPLAY`が必要です。

- [開発・学習ドキュメント（設計・調整・失敗・再学習・改善）](docs/README.md)
- [詳細ガイドPDF](docs/開発・学習ガイド.pdf)
- [完成した配送の動画](artifacts/physical_delivery/delivery.mp4)
- [配送完了の画像](artifacts/physical_delivery/delivery.png)
- [配送の検証結果](artifacts/physical_delivery/verification.json)
- [学習の内容・再学習手順](physics_grasp/README.md)
- [説明資料の原稿](FINAL_REPORT.md)

標準配置と位置・質量・摩擦を変えた3条件、計4試行で配送完了を検証します。指を開いた対照条件では、把持に失敗して停止することを確認します。詳細な実測結果は検証JSONに保存します。この少数の専用シーンでの試験は、任意配置での成功率を保証するものではありません。

把持の位置補正・手首角度・閉じ角度・動作時間はMuJoCo上でCEMにより学習しました。色の検出には学習済み決定木を使います。配送の状態遷移・逆運動学・補間は制御コードで、3次元位置はGazeboから読み取ります。物体の位置を強制変更する把持方式は使っていません。

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" \
  python3 -m unittest discover -s tests -v
# 保存済みの配送試験を集計し、動画・PDF・ソース一式を生成
python3 tools/prepare_submission.py
```

`submission/`に説明付き動画・日本語PDF・ソースZIP・検証JSONを生成します。`アカウントID`は提出時に自身のIDへ変更してください。外部公開や提出は実施していません。

従来のキネマティック配送と過去の検証記録は[LEGACY_DEMO.md](LEGACY_DEMO.md)に残しています。旧デモは`DELIVERY_MODE=legacy ./run_demo.sh ...`で起動します。旧比較動画や旧計測値は、今回の物理配送の性能値とは別です。
