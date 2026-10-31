"""Shared Japanese PDF typography and Markdown layout for project documents."""
from html import escape
from io import BytesIO
from pathlib import Path
import math
import re
import shutil
import subprocess

from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image,
    KeepTogether, CondPageBreak, Flowable,
)
from reportlab.platypus.tableofcontents import TableOfContents

INK = colors.HexColor('#203448')
ACCENT = colors.HexColor('#177E89')
MUTED = colors.HexColor('#647586')
PALE = colors.HexColor('#F0F5F7')
RULE = colors.HexColor('#D8E2E8')
PAGE = (595.28, 841.89)
MARGIN = 48
WIDTH = PAGE[0] - 2 * MARGIN - 12


def setup_fonts():
    font_file = Path('/usr/share/fonts/truetype/takao-gothic/TakaoPGothic.ttf')
    if font_file.exists():
        pdfmetrics.registerFont(TTFont('ProjectJP', str(font_file)))
    else:
        pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
        return 'HeiseiKakuGo-W5'
    symbols = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    if symbols.exists():
        pdfmetrics.registerFont(TTFont('ProjectSymbols', str(symbols)))
    pdfmetrics.registerFontFamily('ProjectJP', normal='ProjectJP', bold='ProjectJP', italic='ProjectJP', boldItalic='ProjectJP')
    return 'ProjectJP'


def styles():
    font = setup_fonts()
    base = dict(fontName=font, textColor=INK, wordWrap='CJK', splitLongWords=True)
    return {
        'title': ParagraphStyle('title', **base, fontSize=23, leading=33, spaceAfter=16, keepWithNext=True),
        'subtitle': ParagraphStyle('subtitle', fontName=font, wordWrap='CJK', fontSize=11, leading=18, textColor=MUTED, spaceAfter=16),
        'body': ParagraphStyle('body', **base, fontSize=9.5, leading=16, spaceAfter=8, allowWidows=0, allowOrphans=0),
        'h1': ParagraphStyle('h1', **base, fontSize=20, leading=29, spaceAfter=16, keepWithNext=True),
        'h2': ParagraphStyle('h2', **base, fontSize=13, leading=20, spaceBefore=15, spaceAfter=9, keepWithNext=True,
                             borderColor=ACCENT, borderWidth=0, borderPadding=0),
        'h3': ParagraphStyle('h3', **base, fontSize=10.5, leading=17, spaceBefore=10, spaceAfter=6, keepWithNext=True),
        'cell': ParagraphStyle('cell', **base, fontSize=8, leading=12, spaceAfter=0),
        'headcell': ParagraphStyle('headcell', fontName=font, wordWrap='CJK', textColor=colors.white, fontSize=8.2, leading=12),
        'code': ParagraphStyle('code', **base, fontSize=8, leading=12, spaceAfter=0),
        'caption': ParagraphStyle('caption', fontName=font, wordWrap='CJK', fontSize=9, leading=14, textColor=MUTED, spaceAfter=9),
        'bullet': ParagraphStyle('bullet', **base, fontSize=9.5, leading=16, spaceAfter=5, leftIndent=12, firstLineIndent=-10),
    }


def escaped(text):
    registered = pdfmetrics.getRegisteredFontNames()
    if 'ProjectJP' not in registered or 'ProjectSymbols' not in registered:
        return escape(text)
    primary = pdfmetrics.getFont('ProjectJP').face.charToGlyph
    symbols = pdfmetrics.getFont('ProjectSymbols').face.charToGlyph
    return ''.join(f'<font name="ProjectSymbols">{escape(c)}</font>'
                   if ord(c) not in primary and ord(c) in symbols else escape(c) for c in text)


def inline(text):
    # Keep link labels readable without repeating long filesystem paths in prose.
    text = escaped(text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<font color="#177E89">\1</font>', text)
    text = re.sub(r'`([^`]+)`', r'<font color="#177E89">\1</font>', text)
    return re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', text)


class ProjectDocument(SimpleDocTemplate):
    def __init__(self, path, title, label):
        super().__init__(str(path), pagesize=PAGE, leftMargin=MARGIN, rightMargin=MARGIN,
                         topMargin=57, bottomMargin=49, title=title, author='CRANE+ Project',
                         pageCompression=1)
        self.label = label
        self.jpfont = setup_fonts()
        self.heading_index = 0

    def beforeDocument(self):
        self.heading_index = 0
        self.has_chapter = False

    def afterFlowable(self, flowable):
        if not isinstance(flowable, Paragraph) or flowable.style.name not in ('h1', 'h2'):
            return
        if flowable.style.name == 'h1': self.has_chapter = True
        level = 1 if flowable.style.name == 'h2' and self.has_chapter else 0
        key = f'heading-{self.heading_index}'
        self.heading_index += 1
        title = flowable.getPlainText()
        self.canv.bookmarkPage(key)
        self.canv.addOutlineEntry(title, key, level=level, closed=False)
        if flowable.style.name == 'h1':
            self.notify('TOCEntry', (0, title, self.page, key))

    def chrome(self, canvas, doc):
        canvas.saveState()
        canvas.setFillColor(ACCENT)
        canvas.rect(MARGIN, PAGE[1]-35, 22, 3, fill=1, stroke=0)
        canvas.setFont(self.jpfont, 8)
        canvas.setFillColor(MUTED)
        canvas.drawString(MARGIN+31, PAGE[1]-35, self.label)
        canvas.setStrokeColor(RULE)
        canvas.line(MARGIN, 36, PAGE[0]-MARGIN, 36)
        canvas.setFont(self.jpfont, 7.5)
        canvas.drawString(MARGIN, 23, 'CRANE+  /  ROS 2 + Gazebo')
        canvas.drawRightString(PAGE[0]-MARGIN, 23, f'{doc.page:02d}')
        canvas.restoreState()

    def render(self, story, toc=False):
        if toc:
            self.multiBuild(story, onFirstPage=self.chrome, onLaterPages=self.chrome)
        else:
            self.build(story, onFirstPage=self.chrome, onLaterPages=self.chrome)


def contents(style):
    toc = TableOfContents()
    toc.levelStyles = [ParagraphStyle('toc', fontName=style['body'].fontName, fontSize=10,
                                      leading=20, spaceBefore=5, textColor=INK, wordWrap='CJK')]
    return toc


def table(rows, style):
    count = max(map(len, rows))
    rows = [r + [''] * (count - len(r)) for r in rows]
    # Allocate widths from robust text lengths so verbose descriptions get room.
    weights = []
    for col in range(count):
        lengths = sorted(pdfmetrics.stringWidth(re.sub(r'[`*]', '', r[col]), style['cell'].fontName, 8)
                         for r in rows)
        representative = lengths[min(len(lengths)-1, math.floor(len(lengths)*.75))]
        weights.append(min(240, max(60, representative + 16)))
    widths = [WIDTH*w/sum(weights) for w in weights]
    if rows[0] == ['トピック', '型', '用途']:
        widths = [WIDTH*.45, WIDTH*.29, WIDTH*.26]
    cells = [[Paragraph(inline(v), style['headcell' if i == 0 else 'cell']) for v in row]
             for i, row in enumerate(rows)]
    result = Table(cells, colWidths=widths, repeatRows=1, hAlign='LEFT',
                   rowSplitRange=(3, -2) if len(rows) > 4 else None)
    result.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), INK),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, PALE]),
        ('LINEBELOW', (0, 0), (-1, 0), .7, ACCENT),
        ('LINEBELOW', (0, 1), (-1, -1), .3, RULE),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 8), ('RIGHTPADDING', (0, 0), (-1, -1), 8),
        ('TOPPADDING', (0, 0), (-1, -1), 7), ('BOTTOMPADDING', (0, 0), (-1, -1), 7),
    ]))
    return result


class StateSequence(Flowable):
    """Fold a linear state sequence into readable rows without shrinking text."""
    def __init__(self, nodes):
        super().__init__()
        self.labels = list(nodes)
        self.width = WIDTH
        self.height = math.ceil(len(self.labels)/3)*57 + 4
        self.font = setup_fonts()

    def draw(self):
        c = self.canv
        box_w, box_h, gap = (WIDTH-36)/3, 36, 18
        positions = []
        for i, name in enumerate(self.labels):
            row, col = divmod(i, 3)
            if row % 2: col = 2-col
            x, y = col*(box_w+gap), self.height-40-row*57
            positions.append((x, y))
            c.setFillColor(PALE); c.setStrokeColor(RULE)
            c.roundRect(x, y, box_w, box_h, 5, fill=1, stroke=1)
            c.setFont(self.font, 8); c.setFillColor(INK)
            label = {'start':'開始', 'end':'終了'}.get(name, name)
            c.drawCentredString(x+box_w/2, y+14, label)
        c.setStrokeColor(ACCENT); c.setFillColor(ACCENT)
        for i in range(len(positions)-1):
            x,y = positions[i]; nx,ny = positions[i+1]
            if ny == y:
                direction = 1 if nx>x else -1
                sx = x+box_w if direction>0 else x
                ex = nx if direction>0 else nx+box_w
                sy = ey = y+box_h/2
                a = (ex-direction*4, ey+2); b = (ex-direction*4,ey-2)
            else:
                sx=ex=x+box_w/2; sy=y; ey=ny+box_h
                a=(ex-2,ey+4); b=(ex+2,ey+4)
            c.line(sx,sy,ex,ey)
            path=c.beginPath(); path.moveTo(ex,ey); path.lineTo(*a); path.lineTo(*b); path.close()
            c.drawPath(path,fill=1,stroke=0)


def diagram(lines):
    """Render the node/edge Mermaid subset used by the maintained chapters."""
    if not shutil.which('dot'):
        raise RuntimeError('PDF diagram generation requires Graphviz (dot).')
    nodes = {}
    edges = []
    state = lines[0].strip() == 'stateDiagram-v2'
    node_pattern = r'(\[\*\]|[A-Za-z_][A-Za-z_0-9]*)(?:\[([^\]]*)\]|\{([^}]*)\})?'
    for line in lines[1:]:
        match = re.fullmatch(r'\s*'+node_pattern+r'\s*-->\s*(?:\|([^|]+)\|\s*)?'+node_pattern+r'\s*', line)
        if not match:
            if line.strip():
                raise ValueError(f'Unsupported Mermaid statement: {line}')
            continue
        left, ll, ld, label, right, rl, rd = match.groups()
        if left == '[*]': left = 'start'
        if right == '[*]': right = 'end'
        nodes.setdefault(left, (ll or ld or left, bool(ld)))
        nodes.setdefault(right, (rl or rd or right, bool(rd)))
        if ll or ld: nodes[left] = (ll or ld, bool(ld))
        if rl or rd: nodes[right] = (rl or rd, bool(rd))
        edges.append((left, right, label))
    if state:
        return StateSequence(nodes)
    def quote(value):
        return '"'+value.replace('\\', '\\\\').replace('"', '\\"')+'"'
    source = ['digraph G {', 'graph [rankdir=TB, bgcolor="transparent", pad=".15", nodesep=".22", ranksep=".23"];',
              'node [shape=box, style="rounded,filled", fillcolor="#f0f5f7", color="#c9d9df", fontname="Noto Sans CJK JP", fontsize=11, margin=".14,.07"];',
              'edge [color="#177e89", arrowsize=.6, fontname="Noto Sans CJK JP", fontsize=9];']
    for key, (label, decision) in nodes.items():
        attrs = f'label={quote(label)}'
        if key in ('start', 'end'): attrs = 'shape=circle, label="", width=.13, height=.13, fillcolor="#177e89"'
        elif decision: attrs += ', shape=diamond'
        source.append(f'{quote(key)} [{attrs}];')
    for left, right, label in edges:
        attr = f' [label={quote(label)}]' if label else ''
        source.append(f'{quote(left)} -> {quote(right)}{attr};')
    source.append('}')
    rendered = subprocess.run(['dot', '-Tpng', '-Gdpi=180'], input='\n'.join(source).encode(), capture_output=True, check=True)
    picture = Image(BytesIO(rendered.stdout))
    factor = min(WIDTH/picture.imageWidth, (510 if state else 465)/picture.imageHeight)
    picture.drawWidth = picture.imageWidth*factor
    picture.drawHeight = picture.imageHeight*factor
    picture.hAlign = 'CENTER'
    return picture


def code_block(lines, style):
    # ReportLab wraps by measured glyph widths, retaining explicit source lines.
    body = '<br/>'.join(escaped(line.replace(' ', '\u00a0')) for line in lines) or ' '
    cell = Paragraph(body, style['code'])
    block = Table([[cell]], colWidths=[WIDTH], hAlign='LEFT')
    block.setStyle(TableStyle([('BACKGROUND', (0,0), (-1,-1), PALE),
        ('BOX', (0,0), (-1,-1), .4, RULE),
        ('LEFTPADDING', (0,0), (-1,-1), 11), ('RIGHTPADDING', (0,0), (-1,-1), 11),
        ('TOPPADDING', (0,0), (-1,-1), 10), ('BOTTOMPADDING', (0,0), (-1,-1), 10)]))
    return block


def markdown(source, style, skip_title=False):
    story = []
    lines = source.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith('```'):
            language = line[3:].strip()
            block = []
            i += 1
            while i < len(lines) and not lines[i].startswith('```'):
                block.append(lines[i]); i += 1
            story += [diagram(block) if language == 'mermaid' else code_block(block, style), Spacer(1, 10)]
        elif line.startswith('|'):
            rows = []
            while i < len(lines) and lines[i].startswith('|'):
                values = [v.strip().replace('\\|', '|') for v in re.split(r'(?<!\\)\|', lines[i].strip().strip('|'))]
                if not all(re.fullmatch(r'[:\- ]+', v or '-') for v in values): rows.append(values)
                i += 1
            if rows: story += [table(rows, style), Spacer(1, 11)]
            continue
        elif line.startswith('# '):
            if not skip_title:
                story.append(Paragraph(inline(line[2:]), style['h1']))
        elif line.startswith('## '):
            story += [CondPageBreak(88), Paragraph(inline(line[3:]), style['h2'])]
        elif line.startswith('### '):
            story += [CondPageBreak(65), Paragraph(inline(line[4:]), style['h3'])]
        elif line.strip():
            bullet = re.match(r'^(?:[-*] |\d+\. )', line)
            if bullet:
                marker = '• ' if line.startswith(('- ', '* ')) else bullet[0]
                story.append(Paragraph(marker+inline(line[bullet.end():]), style['bullet']))
            else:
                story.append(Paragraph(inline(line), style['body']))
        i += 1
    return story


def figure(path, caption, style, width=WIDTH):
    image = Image(str(path))
    ratio = width/image.imageWidth
    image.drawWidth, image.drawHeight = width, image.imageHeight*ratio
    return KeepTogether([image, Spacer(1, 7), Paragraph(inline(caption), style['caption']), Spacer(1, 9)])
