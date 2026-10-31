"""Build a standalone Japanese development guide from maintained Markdown chapters."""
from pathlib import Path
import re
import os

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
CHAPTERS = ['README.md', '07_CONVEYOR_SORTING.md', '01_SYSTEM_DESIGN.md', '02_GRASP_LEARNING.md',
            '03_DEVELOPMENT_HISTORY.md', '04_PARAMETER_REFERENCE.md',
            '05_EXPERIMENT_RUNBOOK.md', '06_IMPROVEMENT_ROADMAP.md',
            'EXPERIMENT_TEMPLATE.md', 'evidence/INDEX.md']


def main():
    from reportlab.platypus import Paragraph, Spacer, PageBreak, HRFlowable
    from pdf_layout import ProjectDocument, styles, contents, markdown, ACCENT

    style = styles()
    story = [Paragraph('DEVELOPMENT & LEARNING', style['caption']),
             Paragraph('CRANE+<br/>接触把持・コンベヤ色選別', style['title']),
             Paragraph('開発・学習ガイド', style['subtitle']),
             HRFlowable(width='100%', thickness=2, color=ACCENT), Spacer(1, 10),
             Paragraph('設計・学習・失敗の分析・調整値・再現実験・改善課題', style['subtitle']),
             Paragraph('更新日：2026-10-31（日本時間）。現在の提出デモはコンベヤ色選別です。単一箱配送は基礎と過去の検証として区別します。編集にはdocs/README.mdから始まるMarkdown版を使ってください。', style['body']),
             Spacer(1, 8), Paragraph('目次', style['h2']), contents(style)]
    combined = ['# CRANE+ 接触把持・コンベヤ色選別 開発・学習ガイド', '', 'このファイルは自動生成です。修正は各章のMarkdownへ行ってください。', '']
    for name in CHAPTERS:
        path = DOCS / name
        source = path.read_text()
        def combined_link(match):
            target = match[2]
            if re.match(r'[a-zA-Z][a-zA-Z0-9+.-]*:', target) or target.startswith(('#', '/')):
                return match[0]
            relative, separator, anchor = target.partition('#')
            rebased = os.path.relpath(path.parent / relative, DOCS)
            return f'[{match[1]}]({rebased}{separator}{anchor})'
        combined_source = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', combined_link, source)
        combined += [f'\n<!-- source: {name} -->\n', combined_source]
        story += [PageBreak(), Paragraph(name, style['caption'])]
        story += markdown(source, style)
    output = DOCS / '開発・学習ガイド.pdf'
    document = ProjectDocument(output, 'CRANE+ 接触把持・コンベヤ色選別 開発・学習ガイド', 'コンベヤ色選別  /  開発・学習ガイド')
    document.render(story, toc=True)
    (DOCS / 'DEVELOPMENT_GUIDE.md').write_text('\n'.join(combined))
    print('Generated:', output)


if __name__ == '__main__':
    main()
