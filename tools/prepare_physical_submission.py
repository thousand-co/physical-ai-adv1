"""Package the contact-physics delivery evidence, current report and sources."""
import json
from pathlib import Path
import subprocess
import shutil
import zipfile
from PIL import Image,ImageDraw,ImageFont
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from summarize_physical_delivery import summarize

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'submission';OUT.mkdir(exist_ok=True)
ART=ROOT/'artifacts/physical_delivery'

def package_sources():
    with zipfile.ZipFile(OUT/'source_code.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for folder in ['ros2_ws/src','tools','tests','physics_grasp','docs','environment','artifacts/physical_delivery','artifacts/conveyor_sorting']:
            for path in (ROOT/folder).rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts:
                    archive.write(path,Path('final_project_ros')/path.relative_to(ROOT))
        for name in ['README.md','GITHUB.md','PHYSICAL_DELIVERY.md','LEGACY_DEMO.md','FINAL_REPORT.md','run_demo.sh','run_physical_demo.sh','run_legacy_demo.sh','.gitignore','.gitattributes']:
            archive.write(ROOT/name,Path('final_project_ros')/name)

def main():
    verification=summarize()
    evidence=json.loads((ART/'delivery.json').read_text())
    for phase,name in [('holding','grasp_hold.png'),('transporting','transport.png'),('done','delivered.png')]:
        if phase=='done':
            shutil.copyfile(ART/'delivery.png',ART/name)
            continue
        event=next(e for e in evidence['events'] if e['state']==phase)
        stamp=event['time']+.4
        if phase=='transporting':stamp=next(e['time'] for e in evidence['events'] if e['state']=='lowering_into_box')-.5
        offset=max(0.,stamp-evidence['video_sim_start'])
        subprocess.run(['ffmpeg','-y','-loglevel','error','-ss',str(offset),'-i',str(ART/'delivery.mp4'),'-frames:v','1',str(ART/name)],check=True)
    grasp=json.loads((ROOT/'physics_grasp/results/evaluation.json').read_text())
    metrics=json.loads((ROOT/'ros2_ws/src/final_project_ros/models/color_tree.json').read_text())
    report=f'''# 学習した接触把持を用いたCRANE+物理配送

## 完成した動作
ROS 2 Humble / Gazebo Classic ODEで、カメラ画像の青い箱を選び、指の接触と摩擦で掴み、持ち上げて1秒保持し、黄色の配送箱へ運搬する。指を開いて箱を解放し、指を退避してから配送完了を確認する。赤い物体は配送しない。箱モデルを手先へ追従させる処理、物体の位置を強制更新する処理、固定拘束は使わない。

## 学習の役割
色分類には教師あり学習済みのOpenCV決定木を使う。合成色データ上の評価は{metrics['synthetic_accuracy']*100:.2f}%であり、実画像一般の精度ではない。把持動作はMuJoCoでCEMによる8パラメータの方策探索を行った。接近位置の補正・手首高さ・手首角度・閉じ角度・3区間の時間を学習している。把持単体は未学習条件{grasp['summary']['learned']['episodes']}件中{grasp['summary']['learned']['successes']}件成功した。
学習したパラメータをそのままGazebo配送制御へ取り込んだ。Gazebo用の追加学習は行っていない。配送先・状態遷移・逆運動学・補間は制御コードであり、配送全体を画像入力から一貫して学習したニューラル方策ではない。動作位置にはGazebo座標を使い、画像検出を開始条件にしている。

## 物理制御と改善
CRANE+ URDFの形状・関節構造・質量・慣性を使う。Gazeboではeffort制御、関節トルク上限1.5Nm、指令速度上限1rad/s、指令加速度上限2rad/s²を適用し、5次補間で滑らかに動かす。実際の速度・加速度の絶対上限を保証するものではない。ODEでは関節減衰を陰的に計算することで初期の数値不安定を解消した。開いた指と配送箱の干渉を避ける配置を採用した。試行の繰り返しで運搬時の箱と縁の干渉が判明したため、運搬前に箱の下端が縁より1.5cm以上高いことを確認する工程を追加し、運搬を0.5rad/s・0.8rad/s²に制限した。アームと手指の区間始点は直前の指令を引き継ぐ。解放後に開いた指が壁へ押し付けられる問題は、箱を離す幅まで開き、実際の指メッシュの下端から必要な上昇量を計算して、姿勢を保ち真上へ退避し、壁より上で全開にする構成にした。把持後に物体で阻まれた手指の実測角を次区間の指令始点へ戻すと握力が弱まるため、閉じ指令を連続して維持するようにした。

## 配送の検証
Gazeboの接触センサーで両指との接触を確認する。持ち上げと運搬中は箱の高さと接触を監視し、落下時は停止する。解放・退避後は箱の8頂点が配送箱内にあり、底面が床の高さにあり、床との接触が検出され、並進速度0.02m/s未満・角速度0.1rad/s未満であることを確認する。赤い物体の変位は5mm以内を要求する。
標準配置と、位置・重さ・摩擦を変えた3条件で、最終版では計{verification['positive_trials']}試行中{verification['positive_successes']}試行の配送に成功した。これは専用シーンでの少数の試行であり、任意配置での成功率ではない。指を開いた対照試験は把持に失敗して安全に停止し、配送完了と判定しなかった。
'''
    for run in verification['positive_runs']:
        report+=f"- {run['evidence']}: {run['elapsed_sim_s']:.2f}秒、赤い物体の変位{run['distractor_displacement_m']*1000:.3f}mm。\n"
    report+='''
## テストと成果物
認識・従来制御・補間・配送判定・握力維持を確認する13件のテストを実行した。物理配送の録画、状態履歴、物体位置、関節値、接触判定、最終姿勢をJSONで保存した。負の対照の失敗記録も残した。動画の実験部分は実際のGazeboカメラ画像で、シミュレーション時間に合わせて再生する。実時間での計算速度は環境に依存する。

## 再現
README.mdとPHYSICAL_DELIVERY.mdの環境を用意し、./run_demo.sh gui:=falseを実行する。成功／失敗の証拠を保存して自動終了する。失敗時は非ゼロの終了コードを返す。tools/prepare_physical_delivery.pyが既存CRANE+からモデルと設定を生成する。学習の再現はphysics_grasp/README.mdを参照する。

## 制約
完成したデモは青い4cm箱1個・赤い物体1個の専用配置である。元のコンベヤ配置から到達可能な位置へ変更した。物体と床・配送箱の衝突は有効だが、ロボットの自己衝突は無効である。障害物回避の経路計画、複数物体の連続配送、未知照明や背景、実機への転用は未検証である。既存ACTモデルは使用していない。

## 出典
ROS 2 Humble、Gazebo Classic、ros2_control、OpenCV、MuJoCo、SciPyを使用した。CRANE+ V2 descriptionの形状・運動学・慣性はRT Corporation等の元実装に基づく。メッシュのライセンスはphysics_grasp/assets/CRANE_PLUS_LICENSEとNOTICEを参照する。外部公開・提出は実施していない。
'''
    (ROOT/'FINAL_REPORT.md').write_text(report);(OUT/'report.md').write_text(report)
    pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
    pdf=canvas.Canvas(str(OUT/'説明資料_PAI最終課題_アカウントID.pdf'),pagesize=(595,842));pdf.setTitle('学習した接触把持を用いたCRANE+物理配送')
    y=795
    for paragraph in report.splitlines():
        size=15 if paragraph.startswith('# ') else 12 if paragraph.startswith('##') else 10
        text=paragraph.lstrip('# ').strip()
        if not text:y-=9;continue
        lines=[];line=''
        for c in text:
            if line and pdfmetrics.stringWidth(line+c,'HeiseiKakuGo-W5',size)>495:lines.append(line);line=c
            else:line+=c
        if line:lines.append(line)
        for line in lines:
            if y<55:pdf.showPage();y=795
            pdf.setFont('HeiseiKakuGo-W5',size);pdf.drawString(50,y,line);y-=size+7
    pdf.showPage();pdf.setFont('HeiseiKakuGo-W5',15);pdf.drawString(50,795,'実際のGazeboカメラ：把持・運搬・配送完了')
    for index,(name,label) in enumerate([('grasp_hold.png','両指で把持・保持'),('transport.png','配送箱の上方へ運搬'),('delivered.png','解放・退避後に配送完了')]):
        top=750-index*225
        pdf.setFont('HeiseiKakuGo-W5',11);pdf.drawString(50,top,label)
        pdf.drawImage(str(ART/name),125,top-215,width=280,height=210,preserveAspectRatio=True,anchor='c')
    pdf.save()
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    def card(name,title,lines):
        image=Image.new('RGB',(1280,720),(18,23,28));draw=ImageDraw.Draw(image)
        draw.text((55,85),title,font=ImageFont.truetype(font,39),fill=(102,203,255))
        for i,line in enumerate(lines):draw.text((55,200+i*65),line,font=ImageFont.truetype(font,25),fill='white')
        image.save(ART/name)
    card('intro.png','学習した把持を使う CRANE+ 物理配送',[
        'AI：学習型色認識 ＋ MuJoCoで学習した把持パラメータ',
        'Gazeboで、指の接触と摩擦だけで青い箱を掴む。',
        '持ち上げ → 運搬 → 配送箱へ解放 → 指を退避して確認',
        '物体の強制追従・位置の書き換えは使わない。',
        '次の実験映像はシミュレーション時間と同じ速度で再生。'])
    card('outro.png','物理配送の検証結果',[
        f"標準配置＋別配置3条件：{verification['positive_successes']}/{verification['positive_trials']}試行で配送完了",
        '両指の接触・持ち上げ・運搬・解放・床への接触を確認。',
        '箱全体が配送箱内で静止し、赤い物体が残ることを確認。',
        '指を開いた対照条件は、把持できず停止。',
        '専用配置の単体デモ。任意配置・複数配送・実機は未検証。'])
    subprocess.run(['ffmpeg','-y','-loglevel','error','-loop','1','-t','22','-i',str(ART/'intro.png'),'-i',str(ART/'delivery.mp4'),'-loop','1','-t','18','-i',str(ART/'outro.png'),'-filter_complex','[0:v]fps=10,format=yuv420p,setsar=1[a];[1:v]scale=960:720,pad=1280:720:160:0,fps=10,format=yuv420p,setsar=1[b];[2:v]fps=10,format=yuv420p,setsar=1[c];[a][b][c]concat=n=3:v=1:a=0[v]','-map','[v]','-c:v','libx264','-preset','fast','-crf','20','-movflags','+faststart',str(OUT/'PAI最終課題_アカウントID.mp4')],check=True)
    verification['recorded_evidence']=evidence
    (OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2))
    package_sources()
    print('Generated contact-physics submission package:',OUT)

if __name__=='__main__':main()
