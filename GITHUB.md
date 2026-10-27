# GitHubでの管理と環境の再現

リポジトリの単位は`final_project_ros/`です。隣接するブラウザ版、データセット、ACTチェックポイント、別の学習用Gitリポジトリは含めません。

## 管理するもの

- ROSソース・launch・ワールド生成処理・学習済み色モデルと把持方策。
- MuJoCo学習コード・評価記録・ライセンス付きメッシュ。
- テスト、設計資料、レポート原稿。
- 成功した選別録画・画像・検証JSONと開発中の失敗記録。
- environment/observed.json：選択した依存パッケージと外部CRANE+の観測バージョン。

ROSのbuild/install/log、Python環境、キャッシュ、再生成できる提出動画・PDF・ZIPは除外します。原本録画はartifacts/conveyor_sorting/episode.mp4として管理し、説明付き提出動画はそこから生成します。現時点の録画は通常のGitで管理し、今後大容量のデータを追加するときはGit LFSやRelease添付の利用を検討します。

## 別PCでの再現

このリポジトリはOSやROS本体を内包しません。元環境はUbuntu 22.04、Python 3.10、ROS 2 Humble、Gazebo Classicです。

1. リポジトリをcloneします。
2. ROS 2 Humble、Gazebo Classic、ros2_control、cv_bridge、OpenCV、NumPy、SciPy、PyYAML、xacro、colconを用意します。依存宣言はros2_ws/src/final_project_ros/package.xmlを参照してください。
3. CRANE+ワークスペースを別途用意し、ビルドします。src/crane_plus/crane_plus_descriptionとinstall/setup.bashが必要です。外部CRANE+の観測コミットはenvironment/observed.jsonを参照します。
4. 描画できるX環境とDISPLAYを用意し、次を実行します。

```bash
export CRANE_WS=/path/to/airobot_ws
DELIVERY_MODE=sorting ./run_demo.sh gui:=true
```

起動処理がCRANE+からURDFと制御設定を再生成してcolconでビルドします。生成済み絶対パスはGitに含めません。詳細は[PHYSICAL_DELIVERY.md](PHYSICAL_DELIVERY.md)と[選別ガイド](docs/07_CONVEYOR_SORTING.md)を参照してください。

## テストと提出物

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" python3 -m unittest discover -s tests -v
python3 tools/document_experiments.py
python3 tools/build_development_docs.py
python3 tools/prepare_submission.py
```

提出生成にはPillow、ReportLab、ffmpeg、日本語フォントNoto Sans CJKも必要です。把持の再学習をする場合だけphysics_grasp/requirements.txtの依存も導入します。環境観測の更新はpython3 tools/capture_environment.pyです。これは完全な依存ロックやインストール処理ではありません。

## 通常の更新

```bash
git status
git add README.md docs/ tools/ tests/ ros2_ws/src/
git diff --cached
git commit -m "Update conveyor sorting demo"
git push
```

生成した提出用動画・PDF・ZIPを配布する場合は、GitHubのReleaseへ添付する方法もあります。ローカル生成だけでGitHubへの公開・提出は行われません。
