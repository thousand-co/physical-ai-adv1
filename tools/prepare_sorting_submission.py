"""Build the submission from the verified four-parcel sorting recording."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from PIL import Image, ImageDraw, ImageFont

from check_physical_delivery import check
from prepare_physical_submission import package_sources

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'submission'
ART = ROOT/'artifacts/conveyor_sorting'
TITLE = '学習型色認識と接触把持によるCRANE+コンベヤ選別'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_report(evidence, metrics):
    rows = '\n'.join(f"|{r['parcel']}|{r['expected_color']}|{r['predicted_color']}|"
        f"{'黄色い配送箱' if r['destination']=='delivery_tray' else '反対側の回収箱'}|確認済み|"
        for r in evidence['parcels'])
    return f'''# {TITLE}

## 目的と完成した動作
青い箱1個だけを配送する課題では、色を識別して行動を選んだことが分かりにくいため、青・赤・緑・青の4箱を1個ずつコンベヤでロボット前へ供給し、停止してから画像で色を判断し、青は黄色い配送箱、それ以外は反対側の灰色の回収箱へ投入する課題を実装した。供給・停止・判定・把持・運搬・解放・退避・投入確認を繰り返し、4箱で自動終了する。動画には処理中の箱番号・判定色・投入先・動作状態を表示する。

## AIと通常制御の役割
色認識はOpenCVの教師ありRGB決定木で行う。10色と背景を合成色データ{metrics['training_samples']}件で学習し、別の乱数seedによる合成評価{metrics['evaluation_samples']}件で正解率{metrics['synthetic_accuracy']*100:.2f}%だった。この数値は合成パレットの画素分類評価であり、本課題の箱の認識成功率や実画像一般の精度ではない。
検査位置のカメラ画像9×9画素を分類し、75%以上が同じ色となったフレームを候補にする。背景・古い画像を除外し、異なるフレームの候補が5回連続で一致したら行き先を決める。モデル名はparcel_00等で色を含めず、正解ラベルは事後の検証にのみ用いる。途中に無効フレームがある場合は候補に追加しないため、必ずしも隣接する5枚ではない。
把持姿勢の位置補正・手首角度・閉じ角度等の8パラメータはMuJoCoでCEMによって学習した方策を使う。Gazebo向けの追加学習は行っていない。配送先の選択規則、状態遷移、逆運動学、5次補間は通常の制御コードである。3次元位置にはGazeboのモデル座標を使い、画像だけから位置を推定するシステムではない。既存のACTモデルは使用していない。

## コンベヤと把持の実装範囲
コンベヤは供給中だけGazeboのSetEntityStateサービスを呼び、現在の箱を検査位置x=0.205m、y=-0.025mまで少量ずつ動かす模擬搬送である。更新ごとの指定速度は0.08m/sを上限とし、応答待ちがあるため実際の平均搬送速度を保証する値ではない。ベルトと箱の摩擦による物理駆動ではない。
停止後は箱の位置設定を呼ばない。指の接触と摩擦で把持・持ち上げ・運搬し、解放後は重力で箱内に落とす。手先への固定拘束や、把持中に箱を強制追従させる処理は使わない。ROS 2 Humble、Gazebo Classic ODE、effort方式のros2_controlを使い、関節トルク上限は1.5Nm、通常の指令速度・加速度上限は1rad/s・2rad/s²、運搬は0.5rad/s・0.8rad/s²とする。これらは指示値の上限であり、実測値の絶対上限の保証ではない。

## 連続選別での工夫
物理配送制御を継承するconveyor_sorting.pyを追加した。判定色に応じて配送先を切り替え、各箱内の2つの投入位置を決定。投入後は同じ検査位置に戻り、次の箱を供給する。
解放時の接触で腕が目標姿勢からずれる問題に対し、箱の床接触、両指保持の解除、指の部分開放角を確認してから、実測した腕の姿勢を始点として上方へ退避する。指メッシュの最低点から上昇量を計算し、箱の壁を越えてから全開にする。箱幅の拡大や閉じ角の増加も試したが、把持に悪影響があったため最終版には採用しなかった。保存した失敗はrelease_interference.json、lift_slip.jsonに記録した。

## 最終版の実測結果
最終版の通し実行1試行で、4箱すべての色判定・投入が成功した。実行時間は{evidence['wall_seconds']:.1f}秒、録画のシミュレーション時間は{evidence['video_sim_end']-evidence['video_sim_start']:.1f}秒だった。

|箱|正解色|判定色|投入先|終了時の保持|
|---|---|---|---|---|
{rows}

箱ごとに両指接触、持ち上げ、運搬を確認した。解放・退避後は箱の8頂点が投入先の内側にあり、底面が床の高さで、床との接触が最近0.2秒以内にあり、並進速度0.02m/s未満、角速度0.1rad/s以下であることを確認した。全箱の処理後、先に投入した箱も含めて箱内の位置・床高さ・並進速度を再確認した。終了時の保持確認は各箱の過去の床接触確認と最終姿勢を組み合わせたもので、全箱の同時の床接触や角速度を再測定した判定ではない。
(注)この4箱は1試行内の4処理であり、独立した4試行の成功率ではない。旧単一箱配送の4条件の結果や、MuJoCo把持単体の評価とは合算しない。最終版の多数回反復による成功率は未評価である。

## テストと成果物
認識、古い画像の除外、同一フレームの二重投票防止、床接触を条件とする実測姿勢からの退避、誤選別・未完了の証跡拒否、既存の軌道・物理配送判定を含むユニットテスト16件が通過した。成功したGazebo実行のJSON・動画・画像・ログをartifacts/conveyor_sorting/に保存した。
成果物は説明付き動画、本文と実際の録画画像を含むPDF、このMarkdown、検証JSON、ソースZIPである。映像本編は実際のGazeboカメラの記録で、シミュレーション時間に合わせて再生される。動作の倍速化は行わなかった。

## 再現方法
UbuntuのROS 2 Humble / Gazebo Classic / CRANE+環境を用意し、README.mdの手順に従う。有効なDISPLAYはGUIを表示しない場合もカメラ描画に必要である。

```bash
cd final_project_ros
DELIVERY_MODE=sorting ./run_demo.sh gui:=false
# GUI表示はgui:=true
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros2_ws/src/final_project_ros:$PYTHONPATH" python3 -m unittest discover -s tests -v
python3 tools/prepare_submission.py
```

run_demo.shの既定は従来の単一箱物理配送であるため、選別にはDELIVERY_MODE=sortingを指定する。失敗時は記録を保存し、非ゼロで終了する。成果物生成は成功済みの選別記録を検証して行い、失敗時に旧デモの成功で置き換えない。

## 制約と今後の課題
既知の検査位置、既知の色・照明、4cm立方体4個、各箱2個のタスク専用である。無限供給、任意個数、満杯の検出、物理駆動ベルト、未知背景・照明、障害物回避の経路計画、実機への適用は未検証である。自己衝突は無効で、物体・テーブル・投入先との接触は有効である。シミュレーション内の停止処理は実機の非常停止ではない。

## 使用技術・出典
ROS 2 Humble、Gazebo Classic、ros2_control、OpenCV、MuJoCo、SciPyを使用した。CRANE+の形状・関節構造・慣性は既存description/controlに基づく。メッシュの権利はphysics_grasp/assets/CRANE_PLUS_LICENSEとNOTICEを参照する。独自実装は色分類、把持方策探索、選別シーン、供給と分岐のステートマシン、接触検証、録画・成果物生成である。外部公開・提出は実施していない。
'''


def make_pdf(report, pictures):
    from reportlab.platypus import Paragraph, Spacer, PageBreak, HRFlowable
    from pdf_layout import ProjectDocument, styles, markdown, figure, ACCENT

    style = styles()
    story = [Paragraph('PAI 最終課題  /  実装・検証レポート', style['caption']),
             Paragraph('学習型色認識と接触把持による<br/>CRANE+ コンベヤ選別', style['title']),
             HRFlowable(width='100%', thickness=2, color=ACCENT), Spacer(1, 12)]
    sections = report.split('\n## ')
    for section in sections[1:]:
        if section.startswith(('連続選別での工夫', '再現方法')):
            story.append(PageBreak())
        story += markdown('## '+section, style)

    story += [PageBreak(), Paragraph('録画で見る選別の動作', style['h1']),
              Paragraph('保存済みのGazebo録画から抽出した画像。画像内の表示で判定色と投入先を確認できます。', style['body'])]
    for index, (name, label) in enumerate(pictures):
        if index == 2:
            story += [PageBreak(), Paragraph('4箱の処理後の最終状態', style['h1'])]
        story.append(figure(ART/name, f'図 {index+1}  {label}', style, width=350 if index < 2 else 470))
    document = ProjectDocument(OUT/'説明資料_PAI最終課題_アカウントID.pdf', TITLE, 'コンベヤ色選別  /  提出資料')
    document.render(story)


def main():
    OUT.mkdir(exist_ok=True)
    evidence = check(ART/'episode.json')
    if evidence.get('mode') != 'conveyor_sorting': raise ValueError('Expected sorting evidence')
    for ext in ['.mp4', '.png']:
        if not (ART/('episode'+ext)).is_file(): raise FileNotFoundError('Missing sorting media '+ext)
    metrics = json.loads((ROOT/'ros2_ws/src/final_project_ros/models/color_tree.json').read_text())
    pictures = []
    for color, name, label in [('blue', 'blue_decision.png', '画像から青と判定し、配送箱へ'),
                               ('red', 'red_decision.png', '画像から赤と判定し、反対側の回収箱へ')]:
        event = next(e for e in evidence['events'] if e['state']=='color_decision' and e['predicted_color']==color)
        subprocess.run(['ffmpeg','-y','-loglevel','error','-ss',str(max(0,event['time']+.4-evidence['video_sim_start'])),
            '-i',str(ART/'episode.mp4'),'-frames:v','1',str(ART/name)],check=True)
        pictures.append((name,label))
    pictures.append(('episode.png','最終状態：黄色に青2個、反対側に赤と緑'))
    report = make_report(evidence, metrics)
    (ROOT/'FINAL_REPORT.md').write_text(report)
    (OUT/'report.md').write_text(report)
    make_pdf(report, pictures)
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    def card(name, title, lines):
        image = Image.new('RGB',(1280,720),(18,23,28)); draw = ImageDraw.Draw(image)
        draw.text((55,80),title,font=ImageFont.truetype(font,36),fill=(102,203,255))
        for i,line in enumerate(lines): draw.text((55,190+i*68),line,font=ImageFont.truetype(font,25),fill='white')
        image.save(ART/name)
    card('intro.png','CRANE+ コンベヤからの色選別デモ',[
        '青 → 赤 → 緑 → 青を1箱ずつ供給し、停止後に画像で色判定。',
        '青は黄色い配送箱へ。それ以外は反対側の回収箱へ。',
        'AI：RGB決定木の色分類 ＋ CEMで学習した把持姿勢。',
        '供給は位置設定による模擬搬送。停止後は接触・摩擦で把持。',
        '本編は実際のGazebo録画。シミュレーション時間で再生。'])
    card('outro.png','4箱の選別・投入と終了時の保持を確認',[
        '青2個を配送箱、赤と緑を反対側の回収箱へ投入。',
        '各箱の接触把持・持ち上げ・運搬・床接触を確認。',
        '最終版の通し検証は1試行。成功率の評価ではない。',
        '既知の色・配置の4箱で終了。無限供給・実機は未検証。',
        'コード・検証JSON・失敗記録・再現手順を添付。'])
    subprocess.run(['ffmpeg','-y','-loglevel','error','-loop','1','-t','15','-i',str(ART/'intro.png'),
        '-i',str(ART/'episode.mp4'),'-loop','1','-t','12','-i',str(ART/'outro.png'),
        '-filter_complex','[0:v]fps=10,format=yuv420p,setsar=1[a];[1:v]scale=960:720,pad=1280:720:160:0,fps=10,format=yuv420p,setsar=1[b];[2:v]fps=10,format=yuv420p,setsar=1[c];[a][b][c]concat=n=3:v=1:a=0[v]',
        '-map','[v]','-c:v','libx264','-preset','fast','-crf','20','-movflags','+faststart',str(OUT/'PAI最終課題_アカウントID.mp4')],check=True)
    shutil.copyfile(ART/'episode.mp4',OUT/'delivery.mp4')
    verification = dict(mode='conveyor_sorting', final_version_trials=1,
        parcels_in_trial=len(evidence['parcels']), recorded_evidence=evidence,
        synthetic_classifier_evaluation=metrics,
        inputs={str(p.relative_to(ROOT)):sha256(p) for p in [ART/'episode.json',ART/'episode.mp4',ART/'episode.png']})
    (OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2))
    (OUT/'README.md').write_text('''# 提出物：CRANE+コンベヤ色選別

- PAI最終課題_アカウントID.mp4：説明付き動画（本編はシミュレーション時間と等速）。
- 説明資料_PAI最終課題_アカウントID.pdf：日本語レポートと録画画像。
- report.md：PDF本文のMarkdown原稿。
- delivery.mp4：説明画面なしの選別録画。
- verification.json：同じ録画の検証結果、入力ファイルのSHA-256。
- source_code.zip：再現用ソース、詳細資料、学習と実験の記録。

アカウントIDは提出者のIDへ変更してください。生成のみを実施し、外部への提出は行っていません。
再生成はプロジェクトルートで python3 tools/prepare_submission.py。
''')
    package_sources()
    print('Generated conveyor sorting submission:',OUT)


if __name__ == '__main__': main()
