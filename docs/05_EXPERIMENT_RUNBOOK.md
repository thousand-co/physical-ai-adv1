# 05 再現・再学習・比較実験の手順

現在の提出デモの起動は`DELIVERY_MODE=sorting ./run_demo.sh gui:=false`です。[07](07_CONVEYOR_SORTING.md)に選別の証跡と再現手順があります。以下の無指定起動や4条件の集計は単一箱モードです。

以下のコマンドの作業ディレクトリは`/home/ubuntu/final_pj/final_project_ros`です。新しい結果を作る前に、基準のコード・設定・結果を保管します。標準の学習・検証スクリプトは上書きを行うため、実験ごとのコピーを推奨します。

## 1. 最初に読む証拠

- `artifacts/physical_delivery/verification.json`：最終4試行と負の対照の集計。
- `artifacts/physical_delivery/delivery.json`：標準試行の状態遷移、サンプル、最終位置・姿勢・速度。
- `physics_grasp/results/policy.json`：学習値。
- `physics_grasp/results/evaluation.json`：学習方策・初期方策・開いた指の90評価。
- `docs/evidence/reference_snapshot/manifest.json`：ドキュメント作成時の主要ファイルのハッシュと保存先。

動画は形状・順序の理解に便利ですが、最後のJSONと同じ試行の動画かを確認します。同名の拡張子違いを組として扱います。古い版の成功映像を、変更後の成功証拠に再利用しません。

## 2. 参照スナップショットを作る

```bash
cd /home/ubuntu/final_pj/final_project_ros
python3 tools/document_experiments.py --snapshot experiments/baseline_01
```

存在しない保存先を指定します。既存フォルダは上書きしません。主要ソース・設定・学習JSON・配送JSON・ログをコピーし、各ファイルのSHA-256を記録します。ライブラリ本体、ROSのinstall/build、動画、MuJoCoの全メッシュを含む完全な実行環境のバックアップではありません。メッシュの再配布物は現在のsource ZIP、元ライセンスは`physics_grasp/assets/`にあります。

`tools/document_experiments.py`を引数なしで実行すると、現在の結果から証拠索引と機械可読catalogを更新します。索引自体は現在値から再生成されるため、論文・提出・比較で使った版はスナップショットやZIPでも保管します。

## 3. 学習を壊さず試す

元の`physics_grasp/results/`で直接学習すると、基準方策・履歴・評価が上書きされます。学習用フォルダごとコピーすると、コード中の`ROOT`もコピー先を指すため結果を分けられます。

```bash
cd /home/ubuntu/final_pj/final_project_ros
mkdir -p experiments/grasp_trial_01
cp -a physics_grasp experiments/grasp_trial_01/physics_grasp
# コピー先のコードを編集してから、コピー先で実行する
PYTHONPATH="$PWD/.simdeps${PYTHONPATH:+:$PYTHONPATH}" MUJOCO_GL=egl \
  python3 experiments/grasp_trial_01/physics_grasp/experiment.py \
  --generations 15 --population 24 --eval-episodes 30 --seed 42 \
  > experiments/grasp_trial_01/train.log 2>&1
```

コピー先の`run.sh`は相対位置`../.simdeps`を参照するので、そのままだと元の依存フォルダを見つけられない場合があります。上記のように元プロジェクトの`.simdeps`をPYTHONPATHへ明示すると避けられます。`.simdeps`をシステム全体へ混ぜず、ROS実行時のPython環境とは分けます。

成功方策の周りを探索する場合は同じコマンドに`--resume`を付けます。ただし完全なチェックポイント復元ではありません。比較のため、resume前のpolicy.jsonも保管します。

## 4. 再学習でまず変える順番

1. **実装が正しいか。** 開閉方向、動的な対象、指の隙間、単位を確認。
2. **到達できるか。** IK失敗の割合を確認。姿勢・配置・高さを修正。
3. **接触するか。** 近づいているだけか、押しているだけか、両指に触れるかを見る。
4. **持ち上がるか。** 閉じ角と手首補正、接触の向きと保持を確認。
5. **保持できるか。** 保持1秒の成功割合と解放テストを確認。
6. **時間・滑らかさ。** 把持が成立した後で時間ペナルティ等を調整。
7. **範囲を広げる。** 位置・質量・摩擦・向き・サイズ・照明を段階的に変える。

世代数だけを増やしても、床との衝突や逆方向の開閉は解決できません。まず動画と失敗コードを見ます。

## 5. 改善した方策をGazeboへ採用する

コピー先のMuJoCoで評価と接触テストを行い、基準より良いか確認します。次に元の方策を控えて、新しい方策を明示的に採用します。

```bash
cd /home/ubuntu/final_pj/final_project_ros
mkdir -p experiments/adoption_01
cp physics_grasp/results/policy.json experiments/adoption_01/previous_policy.json
cp experiments/grasp_trial_01/physics_grasp/results/policy.json \
   physics_grasp/results/policy.json
./run_demo.sh gui:=false output:="$PWD/artifacts/physical_delivery/adoption_01.json"
```

起動時に生成スクリプトがROS側の`models/learned_grasp_policy.json`へコピーします。ROS側のコピーだけを編集すると次回起動で戻ります。

この操作は学習方策を切り替えるもので、学習時の物理環境をGazeboへ完全に同一化する操作ではありません。まず1件を試し、次に複数条件へ広げます。新方策の評価JSON・学習コード・物理設定も一緒に保存してください。元へ戻す場合は控えたpolicy.jsonを同じ元パスへ戻して起動します。

## 6. Gazeboの比較実験

標準結果を壊さない出力名を使います。

```bash
./run_demo.sh gui:=false output:="$PWD/artifacts/physical_delivery/trial_new_01.json"
PHYSICAL_SEED=2000 PHYSICAL_MASS=.03 PHYSICAL_FRICTION=1.0 \
  ./run_demo.sh gui:=false output:="$PWD/artifacts/physical_delivery/seed2000_new.json"
```

環境変数の設定は先頭に付けたコマンド1回に適用します。シェルへexportしている場合、以降の試行にも残る点に注意します。`physical_scene.json`と試行JSONの`scene`で実際に使った値を確認します。

全ての標準検証を実行する場合：

```bash
python3 tools/verify_physical_delivery.py
```

このコマンドは`delivery`、`seed1000`、`seed1001`、`seed1002`、`open_gripper`の固定名を上書きし、途中の正例が失敗するとその時点で終了します。新旧比較用の基準を残すには先にスナップショットやコピーを作ります。`--skip-default`は既存の標準結果を再利用するため、全件を新しいコードで実行する検証には使いません。

未知条件で性能を測るときは、検証スクリプトを実験用コピーにし、seed・物理条件・出力フォルダを別にします。標準の集計器は4つの固定名専用で、任意の実験結果を自動集計する汎用ベンチマークではありません。

## 7. 比較は一度に1つずつ

例：運搬速度.5→.4rad/sを試すなら、他の配置・方策・摩擦・ゲインは変えず、同じseed集合と繰り返し数で比較します。

|記録する指標|意味|
|---|---|
|成功数/全試行数|失敗も分母へ入れる。成功だけを選ばない|
|失敗段階・理由|学習、把持、運搬、解放のどこか|
|動作開始→完了sim秒|ソルバの実行速度と分離|
|wall秒|描画やCPU性能を含む。ビルド時間の包含も明記|
|保持中・運搬中の最低高さ|箱中心と8頂点最低点を区別|
|両指接触の切れた時間|現在のboolだけでなく力・片指接触も将来追加|
|関節追従誤差・飽和|目標と実測、制御トルクを区別|
|最終位置・速度・角速度|配送箱内で落ち着いたか|
|赤い物体の変位|副作用|

平均だけでなく最悪値や分布も見ます。成功と引き換えに所要時間が増える場合は、その両方を記載します。ゲイン・摩擦・速度をまとめて変えた場合は、組み合わせとしての結果を記録し、個々の効果を断定しません。

## 8. JSONの読み方

```python
import json
from pathlib import Path
r = json.loads(Path('artifacts/physical_delivery/trial_new_01.json').read_text())
print(r['final_state'], r['events'][-1])
for event in r['events']:
    print(event['state'], event['time'], event['reason'])
for sample in r['samples']:
    if sample['phase'] == 'transporting':
        print(sample['time'], sample['position'][2], sample['bilateral_contact'])
```

`events.time`はシミュレーションの絶対時刻です。現在の集計は`events[-1].time-events[1].time`（moving_to_startから完了）です。JSONの`wall_seconds`は配送ノード生成から記録時までで、ビルド時間は含みません。

サンプルの記録は約.05秒間隔を狙いますが、厳密な等間隔ではありません。速度や加速度を後計算するときは実際のtime差を使ってください。現在のsamplesに目標角・接触力・物体姿勢の全時系列は含まれず、最終姿勢と最終速度は別フィールドです。必要な解析に応じて記録項目を追加します。

## 9. テスト

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" \
  python3 -m unittest discover -s tests -v
PYTHONPATH="$PWD/.simdeps:$PWD/physics_grasp${PYTHONPATH:+:$PYTHONPATH}" \
  MUJOCO_GL=egl python3 physics_grasp/test_physics.py
```

ROSのPYTHONPATHを上書きして捨てるとrclpyを読み込めません。元の`$PYTHONPATH`を末尾に残してください。MuJoCoテストの保存評価チェックは30件を前提とするので、`--eval-episodes`を変えたコピーではその前提も見直します。

ユニットテストはロジックの退行を検出し、Gazebo通し試験は実際の接触と配送を確認します。どちらか一方だけでは十分ではありません。ただし文書だけの変更で長い物理試験をやり直す必要はなく、変更した範囲に合わせて検証します。

## 10. 不具合の調べ方

|現象|最初に確認するもの|
|---|---|
|rclpyがない|ROSをsourceしたか、PYTHONPATHを消していないか|
|MUJOCO_GL / EGLエラー|学習環境のEGL/OSMesa。GazeboのX環境とは別|
|Gazebo画像が来ない|DISPLAY、Xサーバー、カメラplugin、トピック|
|起動直後のLCPエラー|ODE設定、減衰の陰的計算、初期衝突、ゲイン|
|visual_detection_timeout|画像マスク、投影式、対象の遮蔽、18pixelの対応|
|No reachable solution|BASE相対か、pitchと位置が同時に届くか、残差|
|joint_tracking_timeout|時刻・最大関節誤差・接触。ゲインを上げる前に干渉を見る|
|no_bilateral_contact_at_grasp|開閉方向、両指接触の名前、指の隙間、箱の位置|
|object_dropped_in_transport|高さ、縁との接触、指令連続性、速度、接触履歴|
|fingers_not_clear_of_tray|指の点群変換、退避高さ、配送箱の寸法|
|placement_not_settled_in_tray|8頂点、床接触、速度、片指接触、落ち着くまでの時間|
|成功なのに再生しにくい|ffmpegの有無、H.264、同名の動画とJSONか|

ROSログにKeyboardInterruptや終了時のエラーがあっても、配送結果は同じ試行のJSONとコマンド終了コードで確認します。逆に、ros2 launchの終了だけを成功の根拠にしません。外側のcheckスクリプトでfreshな成功JSONを確認します。

## 11. モデル再生成と依存関係

Gazebo側は`run_demo.sh`が生成・ビルドします。CRANE_WSはinstallだけでなく、現在の生成スクリプトが読む`src/crane_plus/crane_plus_description`も含むワークスペースを指す必要があります。

MuJoCoの再生成は`physics_grasp/README.md`にあるxacro→prepare_model→build_sceneの順です。CoACDのキャッシュを変えると物理形状が変わるため、古い学習方策と新しい形状を同条件と扱わないでください。元メッシュのライセンス・NOTICEも一緒に保存します。

## 12. 記録・提出物の更新

```bash
python3 tools/document_experiments.py
python3 tools/build_development_docs.py
python3 tools/prepare_submission.py
```

最初は証拠索引、次は詳細資料PDF、最後は短い提出用PDF・説明付き動画・ソースZIPの生成です。`prepare_submission.py`はFINAL_REPORT.mdを作り直しますが、手書きの詳細資料は別に残ります。物理配送の`delivery.json`が失敗なら、古いキネマティックの成功を代用せず停止します。書きかけや古い出力が残る可能性があるため、コマンドが正常終了したか確認してください。
