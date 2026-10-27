"""Build a standalone Japanese development guide from maintained Markdown chapters."""
from html import escape
from pathlib import Path
import re
import textwrap
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, KeepTogether, Preformatted

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
CHAPTERS = ['README.md', '07_CONVEYOR_SORTING.md', '01_SYSTEM_DESIGN.md', '02_GRASP_LEARNING.md',
            '03_DEVELOPMENT_HISTORY.md', '04_PARAMETER_REFERENCE.md',
            '05_EXPERIMENT_RUNBOOK.md', '06_IMPROVEMENT_ROADMAP.md',
            'EXPERIMENT_TEMPLATE.md', 'evidence/INDEX.md']


def inline(text):
    # PDF includes readable source paths; working Markdown is the navigable edition.
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', lambda m: f'{m[1]} ({m[2]})', text)
    text = escape(text)
    text = re.sub(r'`([^`]+)`', r'<font color="#244b65">\1</font>', text)
    return re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', text)


def main():
    pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
    font = 'HeiseiKakuGo-W5'
    styles = {
        'body': ParagraphStyle('body', fontName=font, fontSize=8.7, leading=13.2, wordWrap='CJK', spaceAfter=6),
        'h1': ParagraphStyle('h1', fontName=font, fontSize=17, leading=24, spaceAfter=14, wordWrap='CJK'),
        'h2': ParagraphStyle('h2', fontName=font, fontSize=12, leading=17, spaceBefore=10, spaceAfter=7, keepWithNext=True, wordWrap='CJK'),
        'h3': ParagraphStyle('h3', fontName=font, fontSize=10, leading=15, spaceBefore=8, spaceAfter=5, keepWithNext=True, wordWrap='CJK'),
        'code': ParagraphStyle('code', fontName=font, fontSize=7.3, leading=10.6, wordWrap='CJK', backColor=colors.HexColor('#f1f4f6'), borderPadding=5, spaceAfter=2),
        'code_ascii': ParagraphStyle('code_ascii', fontName='Courier', fontSize=7.0, leading=10.6, spaceAfter=2),
        'cell': ParagraphStyle('cell', fontName=font, fontSize=7.2, leading=10.5, wordWrap='CJK'),
    }
    story = [Paragraph('CRANE+ 接触把持・コンベヤ色選別<br/>開発・学習ガイド', styles['h1']),
             Paragraph('設計・学習・失敗の分析・調整値・再現実験・改善課題', styles['h2']),
             Paragraph('更新日：2026-10-27（日本時間）。現在の提出デモはコンベヤ色選別です。単一箱配送は基礎と過去の検証として区別します。編集にはdocs/README.mdから始まるMarkdown版を使ってください。', styles['body'])]
    for index, name in enumerate(CHAPTERS, 1):
        story.append(Paragraph(f'{index}. {name}', styles['body']))
    combined = ['# CRANE+ 接触把持・コンベヤ色選別 開発・学習ガイド', '', 'このファイルは自動生成です。修正は各章のMarkdownへ行ってください。', '']
    for name in CHAPTERS:
        path = DOCS / name
        source = path.read_text()
        combined += [f'\n<!-- source: {name} -->\n', source]
        story.append(PageBreak())
        lines = source.splitlines()
        i = 0
        while i < len(lines):
            line = lines[i]
            if line.startswith('```'):
                language = line[3:].strip()
                i += 1
                if language == 'mermaid':
                    story.append(Paragraph('図はMarkdown版でMermaid表示できます。PDFには図の定義を記載します。', styles['body']))
                while i < len(lines) and not lines[i].startswith('```'):
                    for fragment in textwrap.wrap(lines[i], width=80, replace_whitespace=False, drop_whitespace=False) or [' ']:
                        story.append(Preformatted(fragment, styles['code_ascii'] if fragment.isascii() else styles['code']))
                    i += 1
                story.append(Spacer(1, 6))
            elif line.startswith('|'):
                rows = []
                while i < len(lines) and lines[i].startswith('|'):
                    values = [v.strip() for v in lines[i].strip().strip('|').split('|')]
                    if not all(re.fullmatch(r'[:\- ]+', v or '-') for v in values):
                        rows.append([Paragraph(inline(v), styles['cell']) for v in values])
                    i += 1
                if rows:
                    columns = max(map(len, rows))
                    for row in rows:
                        row.extend([Paragraph('', styles['cell'])] * (columns - len(row)))
                    table = Table(rows, colWidths=[491 / columns] * columns, repeatRows=1, hAlign='LEFT')
                    table.setStyle(TableStyle([('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#e7eef4')),
                                               ('GRID', (0, 0), (-1, -1), .35, colors.HexColor('#bcc8d0')),
                                               ('VALIGN', (0, 0), (-1, -1), 'TOP'),
                                               ('LEFTPADDING', (0, 0), (-1, -1), 4),
                                               ('RIGHTPADDING', (0, 0), (-1, -1), 4),
                                               ('TOPPADDING', (0, 0), (-1, -1), 4),
                                               ('BOTTOMPADDING', (0, 0), (-1, -1), 4)]))
                    story.extend([table, Spacer(1, 8)])
                continue
            elif line.startswith('# '):
                story.append(Paragraph(inline(line[2:]), styles['h1']))
            elif line.startswith('## '):
                story.append(Paragraph(inline(line[3:]), styles['h2']))
            elif line.startswith('### '):
                story.append(Paragraph(inline(line[4:]), styles['h3']))
            elif line.strip():
                story.append(Paragraph(inline(line), styles['body']))
            else:
                story.append(Spacer(1, 3))
            i += 1
    output = DOCS / '開発・学習ガイド.pdf'
    def footer(canvas, doc):
        canvas.setFont(font, 8)
        canvas.setFillColor(colors.HexColor('#5e6c78'))
        canvas.drawString(52, 27, 'CRANE+ 接触把持・コンベヤ色選別 / 開発・学習ガイド')
        canvas.drawRightString(543, 27, str(doc.page))
    document = SimpleDocTemplate(str(output), pagesize=(595, 842), leftMargin=52, rightMargin=52,
                                 topMargin=42, bottomMargin=43, title='CRANE+ 接触把持・コンベヤ色選別 開発・学習ガイド', author='Project development documentation')
    document.build(story, onFirstPage=footer, onLaterPages=footer)
    (DOCS / 'DEVELOPMENT_GUIDE.md').write_text('\n'.join(combined))
    print('Generated:', output)


if __name__ == '__main__':
    main()
