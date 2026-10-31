# CRANE+の接触把持と方策探索（実験版）

青い4cm立方体を、指との接触と摩擦で持ち上げる実験です。把持学習用のMuJoCo環境で、箱を手先へ追従させる処理はありません。学習した方策はGazebo配送にも組み込み済みです。配送の実行・検証結果は[../PHYSICAL_DELIVERY.md](../PHYSICAL_DELIVERY.md)を参照してください。

CEMの計算式、報酬、探索範囲、再学習の留意点は[学習の詳細解説](../docs/02_GRASP_LEARNING.md)と[実験手順](../docs/05_EXPERIMENT_RUNBOOK.md)にまとめています。

## 実測結果

学習で使用していない30配置・条件で、学習した方策は **30/30成功**、初期パラメータは **0/30**、指を開いたままの比較条件は **0/30** でした。成功条件は、箱の中心高さが6.5cmを超え、両側の指の接触を1秒間の95%以上で維持し、終了時にも高さ6.5cmを超えることです。

- 保持中の平均中心高さ：10.20〜10.44cm（初期中心高さ約2cm）。
- 1試行：4.37〜4.46秒。1秒の保持を含みます。
- 指令速度上限：1.5rad/s、指令加速度上限：4rad/s²。5次補間で端点の速度・加速度をゼロにします。
- 評価で計測したアーム関節速度の最大値：1.17rad/s未満。実測加速度の上限は保証しません。
- 把持後に指を開くと箱が重力で落下することを追加テストで確認しました。

動画：[`results/learned_grasp.mp4`](results/learned_grasp.mp4) ／ 最終画像：[`results/hold.png`](results/hold.png)

詳細：`results/evaluation.json`、`results/video_episode.json`。学習結果：`results/policy.json`、`results/training.json`。

この30件はCEMの学習seedとは異なりますが、開発中にも参照した条件であり、全開発工程から隔離した未知テストセットではありません。任意配置での成功率を保証する値ではありません。

## 学習しているもの

CEM（交差エントロピー法）による、8パラメータの方策探索です。接近位置の補正2軸・手首高さ・手首角度・指の閉じ角度・接近／閉じ／持ち上げの時間を、物理シミュレーションの報酬で最適化します。把持保持を優先し、動作時間と速度変動を小さくする評価も含みます。速度・加速度の制限に必要な場合は区間時間を延長します。

逆運動学、接近→閉じる→持ち上げ→保持の順序、5次補間は通常の制御コードです。ニューラルネットワークやACTを用いた画像からの一貫した動作学習ではありません。観測にはシミュレータの物体位置を使います。

学習は乱数seed 0,1,2の3配置で実行し、評価は1000〜1029の30配置を使いました。配置はx=0.105±0.008m、y=0.025±0.015m、質量は20/30/40g、接触の摩擦係数は0.7/1.0/1.3です。評価条件の全組み合わせを網羅したものではありません。箱のサイズ・初期向き・初期アーム姿勢には広いランダム化を加えていません。

## 再実行

この作業環境では依存ライブラリを`../.simdeps`へ導入済みです。

```bash
cd /home/ubuntu/final_pj/final_project_ros/physics_grasp
# 保存済み方策を評価し、動画を再生成
./run.sh --evaluate --eval-episodes 30
# 初期状態から再学習
./run.sh --generations 15 --population 24 --eval-episodes 30
# 保存済み方策を中心に追加探索
./run.sh --resume --generations 8 --population 24 --eval-episodes 30
# 接触把持・解放・対照条件の検証
PYTHONPATH="../.simdeps:$PWD" python3 test_physics.py
```

評価／学習は`results/`を上書きします。別環境ではPython 3.10以降と描画用EGLまたはOSMesaを用意し、仮想環境などへ`pip install -r requirements.txt`を実行してください。`run.sh`はシステム側のインストールも参照します。EGLが使えない場合は`MUJOCO_GL=osmesa ./run.sh ...`でOSMesaを選択できます（OSMesa自体の導入が必要です）。

## モデルの再生成

`scene.xml`と`assets/`を同梱しており、通常の実行にROSは不要です。元モデルから再生成する場合：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
xacro /home/ubuntu/airobot_ws/src/crane_plus/crane_plus_description/urdf/crane_plus.urdf.xacro > /tmp/crane_physics.urdf
export PYTHONPATH="../.simdeps${PYTHONPATH:+:$PYTHONPATH}"
python3 prepare_model.py --urdf /tmp/crane_physics.urdf \
  --description /home/ubuntu/airobot_ws/src/crane_plus/crane_plus_description
python3 build_scene.py
```

既存のCRANE+ V2の関節位置・可動範囲・質量・慣性・表示メッシュを引き継いでいます。指と手首の凹形状はCoACDで凸形状へ分割し、指の隙間を保持しています。モデルの出典とライセンスは`assets/NOTICE`と`assets/CRANE_PLUS_LICENSE`を参照してください。

物理モデルでは正の手指角で閉じ、負の角で開きます。既存デモの開閉指定と逆であることを、形状と実際の接触で確認しました。

## 実験の限界

- 本ディレクトリの評価はMuJoCo上の単体把持・持ち上げ・保持です。学習済みパラメータをGazebo配送へ転用し、配送先への配置も別途検証しました。`../run_demo.sh`はGazeboの物理配送を起動します。実機は未検証です。
- 上から掴むため、箱をCRANE+の近くに置いた専用シーンです。元のコンベヤ・配送箱の配置を再現した環境ではありません。
- ロボット同士の自己衝突は無効です。ロボットと床・物体の衝突は有効です。障害物回避の学習は行っていません。
- 関節のトルク上限は元の1.5Nmを保ち、位置制御ゲインと減衰・関節摩擦を実験用に設定しています（`build_scene.py`）。実機のアクチュエータ同定は行っていません。
- 方策と学習履歴は保存済みです。`attempt1_results/`、`attempt2_results/`は失敗試行、`attempt3_results/`は指令速度制限を追加する前の成功試行です。最終版は`results/`です。
