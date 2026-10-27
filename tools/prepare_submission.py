#!/usr/bin/env python3
"""Build reviewable report, video and source archive from recorded evidence."""
import json
from pathlib import Path
import subprocess
import zipfile
from PIL import Image, ImageDraw, ImageFont
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
def wrap_japanese(text, font, size, width):
    lines, current = [], ''
    for character in text:
        if current and pdfmetrics.stringWidth(current + character, font, size) > width:
            lines.append(current)
            current = character
        else:
            current += character
    if current:
        lines.append(current)
    return lines

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'submission'
OUT.mkdir(exist_ok=True)
ART = ROOT / 'artifacts'
sorting = ART / 'conveyor_sorting/episode.json'
if sorting.exists():
    from prepare_sorting_submission import main
    main()
    raise SystemExit(0)
physical = ART / 'physical_delivery/delivery.json'
if physical.exists():
    if not json.loads(physical.read_text()).get('success'):
        raise RuntimeError('Physical delivery evidence records a failure; refusing to substitute a legacy success')
    from prepare_physical_submission import main
    main()
    raise SystemExit(0)
candidates = [ART / 'motion_optimization/after.mp4', ART / 'two_targets_raw.mp4', ART / 'demo_raw.mp4']
source = next((path for path in candidates if path.exists() and path.with_suffix('.json').exists()
               and json.loads(path.with_suffix('.json').read_text())['final_state'] == 'done'), None)
if source is None:
    raise RuntimeError('No successful recorded run')
evidence = json.loads(source.with_suffix('.json').read_text())
if evidence['final_state'] != 'done':
    raise RuntimeError('No successful recorded run; refusing to label submission complete')
metrics = json.loads((ROOT/'ros2_ws/src/final_project_ros/models/color_tree.json').read_text())
targets = {k:v for k,v in evidence['final_model_positions'].items() if k.startswith('target_blue_')}
count = len(targets)
report = f'''# 学習型色認識を用いたCRANE+物体仕分け

## 概要と目的
ROS 2 HumbleとGazebo Classic上で、カメラ画像から指定色を検出し、CRANE+アームで発送箱へ配送するタスクを実装した。青い対象と赤い誤選別物体を配置し、認識・選択・接近・把持・搬送・配置確認をROSノードで接続した。

## AIの役割
AIはOpenCVの教師あり決定木によるRGB画素の色分類である。Tab10の10色と背景をラベルとし、明るさ・ノイズを変えた合成データ{metrics['training_samples']}件で学習した。独立乱数シードの合成評価データ{metrics['evaluation_samples']}件で正解率{metrics['synthetic_accuracy']*100:.2f}%だった。この値は合成パレット上の評価であり、実画像一般の認識精度ではない。推論時はカメラ画素を入力し、選択色のマスクから輪郭・中心座標を求める。HSVのみのベースラインを残しつつ、デモ起動時は学習済み決定木を使う。

## システムと工夫
Gazeboのカメラ画像を認識ノードが購読し、検出中心を/target_pointに発行する。制御ノードはCameraInfoと既知のカメラ外部パラメータで画像中心と物体を対応付ける。動作座標はGazeboのモデル位置を利用する。画像のみから三次元位置を推定したシステムではない。
MoveItの/compute_ikで関節目標を求め、ROS 2 controlのアーム・グリッパー制御器へ軌道を送る。4自由度アームに合わせて対象の方位から手先yawを決定した。箱とアームの干渉を避ける配置、シミュレーション時間に基づく進行、関節到達確認、複数物体の配送済み管理を追加した。物体ごとに箱内スロットを割り当て、配置後にGazebo座標を検証する。

## 検証結果
実際に記録したシーンでは青い対象{count}個の配送を完了した。最終状態はdone、録画に対応する実時間は{evidence['wall_seconds']:.1f}秒。最終モデル座標は以下の通りである。
'''
for name,position in evidence['final_model_positions'].items():
    report += f'- {name}: '+', '.join(f'{v:.3f}' for v in position)+' m\n'
report += '''
単体の青1個・赤1個のシーンでも完了を確認した。単体検証では関節未到達を無視する継続処理は使用していない。ユニットテスト10件（認識、配置判定、未到達時の停止、補間軌道の速度・加速度制限と端点条件、軌道終了前の遷移抑制など）を実行し、すべて成功した。録画と状態履歴・最終座標JSONを成果物に添付する。青2個の追加シーンは箱への移送時に関節未到達で停止した。20個の実験ワールドも完走していない。提出には完走を記録した単体シーンを使用する。

## 把持動作の改善
起動時に深く折り畳むホーム姿勢を経由せず、現在の関節位置から対象の上方へ接近する構成に変更した。接近と下降に速度・加速度制限付きの5次補間を適用し、始点・終点の速度と加速度をゼロにした。移動量に応じて動作時間を計算し、下降には低い上限を使う。軌道終了後の関節到達を確認してから把持へ進む。
同じ単体シーンで変更前後各1回を記録した結果、初期化から把持完了までのシミュレーション時間は9.43秒から6.03秒へ約36%短縮した。両試行とも配送を完了した。これは当該シーンの比較であり、大域的な最適解や一般的な成功率を示すものではない。比較動画と計測JSONを別途保存した。

## 制約と考察
把持はキネマティック方式で、対象モデルを手先へ追従させ、解放時に箱内へ配置する。物体は静的モデルであり、摩擦・接触力による物理把持や衝突回避経路の検証ではない。IKは手先姿勢を求めるだけで、MoveItによる経路計画は実行していない。関節の到達確認と対象モデルの位置検証を行う範囲の実験である。
分類器は既知のシーンの色を学習した小規模モデルで、実物体・未知照明・背景一般への汎化は示していない。特に灰色の背景との分離には課題が残る。既存の5次元ACTチェックポイントは画像や色を入力できないため本タスクには使用していない。今後は深度カメラによる物体座標推定、実際の接触把持、実画像での評価へ発展させたい。

## 再現方法
README.mdの手順でROS/CRANE+環境をsourceし、run_demo.shを実行する。学習モデルの再生成はpython3 tools/train_color_model.py、テストと録画コマンドはREADME.mdを参照。動画は実際のGazeboカメラと関節値を記録したもので、アニメーションによる再現ではない。録画時の実時間で再生し、前後に説明画面を付加した。

## 使用技術・参考実装
講義環境のCRANE+ description/control/MoveIt設定を使用。ROS 2 Humble、Gazebo Classic、MoveIt 2、ros2_control、OpenCVを利用した。独自実装はシーン設計、学習型色認識、検出とモデルの対応付け、配送状態機械、検証・録画・提出物生成である。依存パッケージの権利は各配布元に帰属する。課題要件はdocの10/19改訂版PDFを参照した。
'''
(ROOT/'FINAL_REPORT.md').write_text(report)
(OUT/'report.md').write_text(report)
pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
pdf = canvas.Canvas(str(OUT/'説明資料_PAI最終課題_アカウントID.pdf'), pagesize=(595,842))
pdf.setTitle('学習型色認識を用いたCRANE+物体仕分け')
y=795
for paragraph in report.splitlines():
    heading = paragraph.startswith('#')
    text=paragraph.lstrip('# ').strip()
    size=15 if paragraph.startswith('# ') else 12 if heading else 10
    if not text:
        y-=9;continue
    for line in wrap_japanese(text,'HeiseiKakuGo-W5',size,495):
        if y<55:
            pdf.showPage();y=795
        pdf.setFont('HeiseiKakuGo-W5',size);pdf.drawString(50,y,line);y-=size+7
pdf.save()
fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def card(path,title,lines):
    img=Image.new('RGB',(1280,720),(18,23,28));draw=ImageDraw.Draw(img)
    draw.text((65,90),title,font=ImageFont.truetype(fontpath,42),fill=(102,203,255))
    for i,line in enumerate(lines):
        draw.text((65,200+i*58),line,font=ImageFont.truetype(fontpath,27),fill='white')
    img.save(path)
card(ART/'intro.png','学習型色認識 × CRANE+ 配送タスク',[
    'ROS 2 + Gazebo + OpenCV 決定木（教師あり学習）',
    f'青い対象{count}個を選別して箱へ配送。赤い物体は動かさない。',
    'AI：カメラ画素の色分類 ／ 制御：MoveIt IK + ros2_control',
    '物体位置にはGazebo座標を使用。把持はキネマティック方式。',
    '次の映像は実際のカメラ・関節値の録画（実時間再生）。'])
card(ART/'outro.png','検証結果と実装の範囲',[
    f'青い対象 {count}個の配送完了 ／ 最終状態：done',
    '対象の箱内座標と、赤い物体が元の位置に残ることを確認。',
    '関節到達を確認し、未到達ならエラーで停止。',
    '合成色データ上の分類評価・検証ログ・コードを添付。',
    '接触力学による物理把持・実機動作は今後の課題。'])
# No speed change; concatenate title, recorded camera and end card.
subprocess.run(['ffmpeg','-y','-loglevel','error','-loop','1','-t','12','-i',str(ART/'intro.png'),
 '-i',str(source),'-loop','1','-t','10','-i',str(ART/'outro.png'),
 '-filter_complex','[0:v]fps=10,format=yuv420p,setsar=1[a];[1:v]fps=10,format=yuv420p,setsar=1[b];[2:v]fps=10,format=yuv420p,setsar=1[c];[a][b][c]concat=n=3:v=1:a=0[v]',
 '-map','[v]','-c:v','libx264','-preset','fast','-crf','20','-movflags','+faststart',str(OUT/'PAI最終課題_アカウントID.mp4')],check=True)
(OUT/'verification.json').write_text(json.dumps({'recording':source.name,'recorded_evidence':evidence,'synthetic_classifier_evaluation':metrics},ensure_ascii=False,indent=2))
with zipfile.ZipFile(OUT/'source_code.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for folder in ('ros2_ws/src','tools','tests'):
        for path in (ROOT/folder).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                archive.write(path,Path('final_project_ros')/path.relative_to(ROOT))
    for name in ('README.md','FINAL_REPORT.md','run_demo.sh','.gitignore'):
        archive.write(ROOT/name,Path('final_project_ros')/name)
print('Generated submission:',count,'targets; source:',source.name)
