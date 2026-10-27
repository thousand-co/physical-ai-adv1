"""Index existing evidence and optionally preserve an immutable reference snapshot.

Never runs training/simulation and never changes policy or recorded evidence.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
EVIDENCE = DOCS / 'evidence'


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def link(path):
    return os.path.relpath(path, EVIDENCE).replace(os.sep, '/')


def index_evidence():
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    records = []
    for path in sorted([*(ROOT / 'artifacts/physical_delivery').rglob('*.json'),
                        *(ROOT / 'artifacts/conveyor_sorting').rglob('*.json')]):
        data = json.loads(path.read_text())
        if 'events' not in data:
            continue
        stages = [event['state'] for event in data['events']]
        last = data['events'][-1] if data['events'] else {}
        records.append({
            'path': str(path.relative_to(ROOT)), 'sha256': digest(path),
            'final_state': data.get('final_state'), 'reason': last.get('reason', ''),
            'final_sequence_markers_present': all(s in stages for s in ['clearing_box_rim', 'opening_after_retreat']),
            'scene': data.get('scene'),
            'media': [str(path.with_suffix(ext).relative_to(ROOT)) for ext in ['.mp4', '.png'] if path.with_suffix(ext).exists()],
        })
    learning = []
    for folder in ['attempt1_results', 'attempt2_results', 'attempt3_results', 'results']:
        base = ROOT / 'physics_grasp' / folder
        evaluation = json.loads((base / 'evaluation.json').read_text())
        history = json.loads((base / 'training.json').read_text())
        learning.append({'folder': str(base.relative_to(ROOT)), 'generations': len(history),
                         'best_reward': history[-1]['best_reward'], 'summary': evaluation['summary']})
    catalog = {'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
               'interpretation': 'Observed files, not a complete chronological history of all executed trials. Old versions are not pooled into the final-version success count.',
               'delivery_records': records, 'learning_records': learning}
    (EVIDENCE / 'catalog.json').write_text(json.dumps(catalog, ensure_ascii=False, indent=2))
    lines = ['# 証拠ファイル索引', '',
             '既存ファイルから自動生成。表の順序はファイルパス順で、実行時刻順ではありません。上書きされた試行は復元しません。', '',
             '現在の提出デモは[4箱の色選別](../../artifacts/conveyor_sorting/episode.json)です。最終版の通し検証1試行内で4箱を処理しました。旧単一箱配送の[4条件の集計](../../artifacts/physical_delivery/verification.json)とは合算しません。', '',
             '## 配送の保存記録', '', '|JSON|最終状態|最後の理由|最終工程マーカー|画像・動画|', '|---|---|---|---|---|']
    for record in records:
        path = ROOT / record['path']
        media = ' / '.join(f'[{Path(m).suffix[1:]}]({link(ROOT / m)})' for m in record['media'])
        reason = record['reason'].replace('|', '\\|') or '—'
        marker = '両工程あり' if record['final_sequence_markers_present'] else '旧工程／把持前停止'
        lines.append(f"|[{record['path']}]({link(path)})|{record['final_state']}|{reason}|{marker}|{media}|")
    lines += ['', 'マーカーは版を識別する補助で、ソースハッシュの代替ではありません。対照条件は把持前で停止するため、最終版でも後段のマーカーがありません。', '',
              '## 学習の保存記録', '', '|保存先|世代数|best reward|学習方策の成功|評価JSON|', '|---|---:|---:|---|---|']
    for item in learning:
        stats = item['summary']['learned']
        target = ROOT / item['folder'] / 'evaluation.json'
        lines.append(f"|{item['folder']}|{item['generations']}|{item['best_reward']:.6f}|{stats['successes']}/{stats['episodes']}|[評価]({link(target)})|")
    lines += ['', '物理モデル・報酬・軌道制限が変更されているため、異なる版のrewardを直接比較しません。', '', '## 実行ログ', '']
    logs = sorted([*(ROOT / 'artifacts/physical_delivery').rglob('*.log'),
                   *(ROOT / 'artifacts/conveyor_sorting').rglob('*.log')])
    for path in logs:
        lines.append(f'- [{path.relative_to(ROOT)}]({link(path)})')
    lines += ['', 'development_logsは、文書作成時点で残っていた一時ログを退避したものです。一時ログ名はソース版を保証せず、同じ名前で過去に上書きされた可能性があります。JSONとコードに照合して読みます。', '',
              '## 参照スナップショット', '']
    snapshot = EVIDENCE / 'reference_snapshot/manifest.json'
    if snapshot.exists():
        lines.append('[manifest.json](reference_snapshot/manifest.json)は旧単一箱配送の参照用控えです。現在の選別ソースのスナップショットではありません。現在版は提出ソースZIPとsubmission/verification.jsonを参照します。')
    else:
        lines.append('まだありません。`python3 tools/document_experiments.py --snapshot docs/evidence/reference_snapshot`で作成します。')
    lines += ['', '機械可読の索引：[catalog.json](catalog.json)。', '']
    (EVIDENCE / 'INDEX.md').write_text('\n'.join(lines))
    return catalog


def selected_files():
    allowed = {'.py', '.md', '.json', '.yaml', '.yml', '.xml', '.urdf', '.world', '.sh', '.txt', '.log', '.npz'}
    paths = set()
    for directory in ['ros2_ws/src', 'tools', 'tests', 'physics_grasp', 'artifacts/physical_delivery', 'artifacts/conveyor_sorting']:
        for path in (ROOT / directory).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix in allowed:
                paths.add(path)
    paths.update(path for path in ROOT.glob('*.md'))
    paths.update(path for path in ROOT.glob('*.sh'))
    paths.update(DOCS.glob('*.md'))
    # This function intentionally never traverses docs/evidence or experiments.
    paths.discard(DOCS / 'DEVELOPMENT_GUIDE.md')
    return sorted(paths)


def snapshot(destination):
    destination = Path(destination)
    if not destination.is_absolute():
        destination = ROOT / destination
    if destination.exists():
        raise FileExistsError('Snapshot already exists; choose a new directory: ' + str(destination))
    paths = selected_files()
    destination.mkdir(parents=True)
    entries = []
    for path in paths:
        relative = path.relative_to(ROOT)
        copied = destination / 'files' / relative
        copied.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, copied)
        entries.append({'source': str(relative), 'copy': str(copied.relative_to(destination)),
                        'bytes': path.stat().st_size, 'sha256': digest(copied)})
    geometry = {str(p.relative_to(ROOT)): digest(p) for p in sorted((ROOT / 'physics_grasp/assets').rglob('*'))
                if p.is_file() and p.suffix in {'.stl', '.obj'}}
    manifest = {'captured_at_utc': datetime.now(timezone.utc).isoformat(),
                'python_at_capture': sys.version, 'platform_at_capture': platform.platform(),
                'scope': 'Selected code/configuration/JSON/logs. Geometry is hashed but not copied. Dependencies and videos are not included. Capture environment is not proof of the historical execution environment.',
                'files': entries, 'geometry_sha256': geometry}
    (destination / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(f'Snapshot: {destination} ({len(entries)} files; {len(geometry)} geometry hashes)')
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', help='New snapshot directory, relative to project root or absolute')
    args = parser.parse_args()
    index_evidence()
    if args.snapshot:
        snapshot(args.snapshot)
        index_evidence()
    print('Evidence index:', EVIDENCE / 'INDEX.md')


if __name__ == '__main__':
    main()
