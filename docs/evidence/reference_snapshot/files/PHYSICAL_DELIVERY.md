# 学習した物理把持を使うGazebo配送デモ

標準の`run_demo.sh`は、青い箱を指の接触・摩擦で掴み、黄色の配送箱へ運び、指を開いて投入する物理配送を起動します。ROS 2 Humble / Gazebo Classic ODE上で実際に実行して検証します。対象モデルの位置を強制変更する処理や、手先への固定拘束は使っていません。

```bash
cd /home/ubuntu/final_pj/final_project_ros
./run_demo.sh gui:=false
# GUIも表示する場合（有効なDISPLAYが必要）
./run_demo.sh gui:=true
```

認識・関節制御の起動後に自動で実行し、成功／失敗の記録を保存してGazeboを終了します。成功の確認まで含めてコマンドが終了し、失敗時は非ゼロの終了コードを返します。`gui:=false`でもカメラのレンダリング用X環境は必要です。

保存先：`artifacts/physical_delivery/episode.json`、同名の`.mp4`と`.png`。保存先を変える場合：

```bash
./run_demo.sh gui:=false output:="$PWD/artifacts/physical_delivery/my_run.json"
```

詳細な設計理由・失敗と調整・再学習と比較実験は[開発・学習ドキュメント](docs/README.md)を参照してください。

## 配送を完了とする条件

1. 学習済み色分類器がカメラ画像内の青い物体を検出し、その画像位置とモデル位置が一致すること。
2. 両指の接触をGazeboの接触センサーで確認し、箱を持ち上げて1秒保持すること。
3. 配送箱へ運搬中も、箱の高さと両指の接触を維持すること。
4. 箱を離せる幅まで指を開き、実際の指メッシュの下端から必要な上昇量を計算して姿勢を保って真上へ退避し、配送箱の壁より上で指を全開にすること。その後、箱の8頂点すべてが配送箱の内側に収まること。
5. 箱の底面が配送箱の床の高さにあり、床との接触が検出され、並進速度が0.02m/s未満、角速度が0.1rad/s未満であること。
6. 赤い誤選別物体の移動が5mm以内であること。

座標や関節値だけで掴めたとみなさず、両指との接触を用います。把持で指が物体に阻まれて目標角へ到達しなくても、それ自体はエラーにしません。運搬へ移る際も閉じ指令を保持し、握力の目標を途切れさせないようにしています。アームの区間始点も直前の目標を引き継ぎ、指令の切り替えで保持力が弱まることを避けます。関節が追従できない場合、把持・運搬に失敗した場合、配置を確認できない場合は停止します。

## 学習と制御

MuJoCoのCRANE+モデルでCEMにより学習した8パラメータを、`models/learned_grasp_policy.json`としてROSパッケージへ取り込みます。学習の詳細は[physics_grasp/README.md](physics_grasp/README.md)を参照してください。

Gazeboでは、同じCRANE+ URDFの形状・関節構造を使い、トルク上限1.5NmのeffortインターフェースとPID制御を使います。接近・把持・持ち上げ・配置・解放・退避の指令は、速度1rad/s、加速度2rad/s²を上限とする5次補間です。運搬は速度0.5rad/s・加速度0.8rad/s²に制限し、開始前に箱の下端が配送箱の縁より1.5cm以上高いことを確認します。Gazeboの関節減衰は陰的計算にして安定化しました。Gazebo用の追加学習は実施していません。

学習対象は把持動作の補正値と時間です。状態遷移・運搬先・逆運動学・補間は制御コードであり、配送全体を画像から一貫して学習するニューラル方策ではありません。3次元位置にはGazeboのモデル座標を使い、画像検出を動作開始条件にしています。

## 別条件と対照試験

```bash
PHYSICAL_SEED=1000 PHYSICAL_MASS=.02 PHYSICAL_FRICTION=.7 \
  ./run_demo.sh gui:=false output:="$PWD/artifacts/physical_delivery/seed1000.json"
# 指を開いたままの負の対照。把持できず停止するのが期待結果です。
./run_demo.sh gui:=false disable_grip:=true \
  output:="$PWD/artifacts/physical_delivery/open_gripper.json"
```

`PHYSICAL_SEED`で初期配置をx±8mm・y±15mmの範囲で変えます。`PHYSICAL_MASS`はkg、`PHYSICAL_FRICTION`は接触摩擦係数です。これらは起動時に物理シーンを生成するための設定で、運転中の物体位置は変更しません。

全条件をGazeboで再実行して集計する場合：

```bash
python3 tools/verify_physical_delivery.py
```

各試験の動画・JSON・ログを保存し、`artifacts/physical_delivery/verification.json`へ集計します。途中の開発試験で発生した数値不安定・干渉・落下の記録も同フォルダに残しています。最終版の集計は、高さ確保と退避後の全開の両工程がある試行だけを使用します。

## 環境とテスト

既存のROS 2 Humble / Gazebo Classic / CRANE+ワークスペースに加え、PythonのSciPyが必要です。MoveItはこの物理モードの運動学計算には使用しません。既定のCRANE+環境は`/home/ubuntu/airobot_ws`です。異なる場合は`CRANE_WS`を設定してください。起動時に`tools/prepare_physical_delivery.py`がURDF、コントローラ設定、ワールド、方策を準備してビルドします。

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" \
  python3 -m unittest discover -s tests -v
```

## 適用範囲

完成したデモは、青い4cm箱1個と赤い物体1個の専用配置です。元のコンベヤ配置よりロボットに近い位置へ対象を置き、開いた指が配送箱に干渉しない配置にしています。任意配置・複数配送・障害物回避・実機への適用は未検証です。自己衝突は無効で、物体・テーブル・配送箱との接触は有効です。

従来のキネマティック配送は`DELIVERY_MODE=legacy ./run_demo.sh ...`または`./run_legacy_demo.sh ...`で実行できます。旧比較動画や旧検証値は、この物理配送の性能値とは別です。
