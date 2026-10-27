# 証拠ファイル索引

既存ファイルから自動生成。表の順序はファイルパス順で、実行時刻順ではありません。上書きされた試行は復元しません。

現在の提出デモは[4箱の色選別](../../artifacts/conveyor_sorting/episode.json)です。最終版の通し検証1試行内で4箱を処理しました。旧単一箱配送の[4条件の集計](../../artifacts/physical_delivery/verification.json)とは合算しません。

## 配送の保存記録

|JSON|最終状態|最後の理由|最終工程マーカー|画像・動画|
|---|---|---|---|---|
|[artifacts/conveyor_sorting/episode.json](../../artifacts/conveyor_sorting/episode.json)|done|—|両工程あり|[mp4](../../artifacts/conveyor_sorting/episode.mp4) / [png](../../artifacts/conveyor_sorting/episode.png)|
|[artifacts/conveyor_sorting/lift_slip.json](../../artifacts/conveyor_sorting/lift_slip.json)|error|lift_not_confirmed|旧工程／把持前停止||
|[artifacts/conveyor_sorting/release_interference.json](../../artifacts/conveyor_sorting/release_interference.json)|error|joint_tracking_timeout: 0.1274382050446654|両工程あり|[png](../../artifacts/conveyor_sorting/release_interference.png)|
|[artifacts/physical_delivery/before_release_clearance_fix/delivery.json](../../artifacts/physical_delivery/before_release_clearance_fix/delivery.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/before_release_clearance_fix/delivery.mp4) / [png](../../artifacts/physical_delivery/before_release_clearance_fix/delivery.png)|
|[artifacts/physical_delivery/before_release_clearance_fix/seed1000.json](../../artifacts/physical_delivery/before_release_clearance_fix/seed1000.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/before_release_clearance_fix/seed1000.mp4) / [png](../../artifacts/physical_delivery/before_release_clearance_fix/seed1000.png)|
|[artifacts/physical_delivery/before_release_clearance_fix/seed1001.json](../../artifacts/physical_delivery/before_release_clearance_fix/seed1001.json)|error|joint_tracking_timeout: 0.28573426362410204|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/before_release_clearance_fix/seed1001.mp4) / [png](../../artifacts/physical_delivery/before_release_clearance_fix/seed1001.png)|
|[artifacts/physical_delivery/delivery.json](../../artifacts/physical_delivery/delivery.json)|done|—|両工程あり|[mp4](../../artifacts/physical_delivery/delivery.mp4) / [png](../../artifacts/physical_delivery/delivery.png)|
|[artifacts/physical_delivery/failed_repeat_before_carry_fix.json](../../artifacts/physical_delivery/failed_repeat_before_carry_fix.json)|error|object_dropped_in_transport|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/failed_repeat_before_carry_fix.mp4) / [png](../../artifacts/physical_delivery/failed_repeat_before_carry_fix.png)|
|[artifacts/physical_delivery/open_gripper.json](../../artifacts/physical_delivery/open_gripper.json)|error|no_bilateral_contact_at_grasp|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/open_gripper.mp4) / [png](../../artifacts/physical_delivery/open_gripper.png)|
|[artifacts/physical_delivery/pre_clearance_results/open_gripper.json](../../artifacts/physical_delivery/pre_clearance_results/open_gripper.json)|error|no_bilateral_contact_at_grasp|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/pre_clearance_results/open_gripper.mp4) / [png](../../artifacts/physical_delivery/pre_clearance_results/open_gripper.png)|
|[artifacts/physical_delivery/pre_clearance_results/seed1000.json](../../artifacts/physical_delivery/pre_clearance_results/seed1000.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/pre_clearance_results/seed1000.mp4) / [png](../../artifacts/physical_delivery/pre_clearance_results/seed1000.png)|
|[artifacts/physical_delivery/pre_clearance_results/seed1001.json](../../artifacts/physical_delivery/pre_clearance_results/seed1001.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/pre_clearance_results/seed1001.mp4) / [png](../../artifacts/physical_delivery/pre_clearance_results/seed1001.png)|
|[artifacts/physical_delivery/pre_clearance_results/seed1002.json](../../artifacts/physical_delivery/pre_clearance_results/seed1002.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/pre_clearance_results/seed1002.mp4) / [png](../../artifacts/physical_delivery/pre_clearance_results/seed1002.png)|
|[artifacts/physical_delivery/seed1000.json](../../artifacts/physical_delivery/seed1000.json)|done|—|両工程あり|[mp4](../../artifacts/physical_delivery/seed1000.mp4) / [png](../../artifacts/physical_delivery/seed1000.png)|
|[artifacts/physical_delivery/seed1001.json](../../artifacts/physical_delivery/seed1001.json)|done|—|両工程あり|[mp4](../../artifacts/physical_delivery/seed1001.mp4) / [png](../../artifacts/physical_delivery/seed1001.png)|
|[artifacts/physical_delivery/seed1002.json](../../artifacts/physical_delivery/seed1002.json)|done|—|両工程あり|[mp4](../../artifacts/physical_delivery/seed1002.mp4) / [png](../../artifacts/physical_delivery/seed1002.png)|
|[artifacts/physical_delivery/trial1.json](../../artifacts/physical_delivery/trial1.json)|error|joint_tracking_timeout: 1.7892321432495368|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/trial1.mp4) / [png](../../artifacts/physical_delivery/trial1.png)|
|[artifacts/physical_delivery/trial2.json](../../artifacts/physical_delivery/trial2.json)|error|joint_tracking_timeout: 0.24080764685949474|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/trial2.mp4) / [png](../../artifacts/physical_delivery/trial2.png)|
|[artifacts/physical_delivery/trial3.json](../../artifacts/physical_delivery/trial3.json)|error|object_dropped_in_transport|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/trial3.mp4) / [png](../../artifacts/physical_delivery/trial3.png)|
|[artifacts/physical_delivery/trial4.json](../../artifacts/physical_delivery/trial4.json)|done|—|旧工程／把持前停止|[mp4](../../artifacts/physical_delivery/trial4.mp4) / [png](../../artifacts/physical_delivery/trial4.png)|

マーカーは版を識別する補助で、ソースハッシュの代替ではありません。対照条件は把持前で停止するため、最終版でも後段のマーカーがありません。

## 学習の保存記録

|保存先|世代数|best reward|学習方策の成功|評価JSON|
|---|---:|---:|---|---|
|physics_grasp/attempt1_results|15|0.124636|0/30|[評価](../../physics_grasp/attempt1_results/evaluation.json)|
|physics_grasp/attempt2_results|16|-0.116270|0/30|[評価](../../physics_grasp/attempt2_results/evaluation.json)|
|physics_grasp/attempt3_results|15|13.712930|30/30|[評価](../../physics_grasp/attempt3_results/evaluation.json)|
|physics_grasp/results|8|13.546499|30/30|[評価](../../physics_grasp/results/evaluation.json)|

物理モデル・報酬・軌道制限が変更されているため、異なる版のrewardを直接比較しません。

## 実行ログ

- [artifacts/conveyor_sorting/episode.log](../../artifacts/conveyor_sorting/episode.log)
- [artifacts/conveyor_sorting/release_interference.log](../../artifacts/conveyor_sorting/release_interference.log)
- [artifacts/physical_delivery/delivery.log](../../artifacts/physical_delivery/delivery.log)
- [artifacts/physical_delivery/development_logs/grasp_training.log](../../artifacts/physical_delivery/development_logs/grasp_training.log)
- [artifacts/physical_delivery/development_logs/grasp_training2.log](../../artifacts/physical_delivery/development_logs/grasp_training2.log)
- [artifacts/physical_delivery/development_logs/grasp_training3.log](../../artifacts/physical_delivery/development_logs/grasp_training3.log)
- [artifacts/physical_delivery/development_logs/grasp_training4.log](../../artifacts/physical_delivery/development_logs/grasp_training4.log)
- [artifacts/physical_delivery/development_logs/physical_delivery1.log](../../artifacts/physical_delivery/development_logs/physical_delivery1.log)
- [artifacts/physical_delivery/development_logs/physical_delivery2.log](../../artifacts/physical_delivery/development_logs/physical_delivery2.log)
- [artifacts/physical_delivery/development_logs/physical_delivery3.log](../../artifacts/physical_delivery/development_logs/physical_delivery3.log)
- [artifacts/physical_delivery/development_logs/physical_delivery4.log](../../artifacts/physical_delivery/development_logs/physical_delivery4.log)
- [artifacts/physical_delivery/development_logs/physical_delivery_clearance.log](../../artifacts/physical_delivery/development_logs/physical_delivery_clearance.log)
- [artifacts/physical_delivery/development_logs/physical_delivery_complete.log](../../artifacts/physical_delivery/development_logs/physical_delivery_complete.log)
- [artifacts/physical_delivery/open_gripper.log](../../artifacts/physical_delivery/open_gripper.log)
- [artifacts/physical_delivery/seed1000.log](../../artifacts/physical_delivery/seed1000.log)
- [artifacts/physical_delivery/seed1001.log](../../artifacts/physical_delivery/seed1001.log)
- [artifacts/physical_delivery/seed1002.log](../../artifacts/physical_delivery/seed1002.log)
- [artifacts/physical_delivery/tests.log](../../artifacts/physical_delivery/tests.log)

development_logsは、文書作成時点で残っていた一時ログを退避したものです。一時ログ名はソース版を保証せず、同じ名前で過去に上書きされた可能性があります。JSONとコードに照合して読みます。

## 参照スナップショット

[manifest.json](reference_snapshot/manifest.json)は旧単一箱配送の参照用控えです。現在の選別ソースのスナップショットではありません。現在版は提出ソースZIPとsubmission/verification.jsonを参照します。

機械可読の索引：[catalog.json](catalog.json)。
