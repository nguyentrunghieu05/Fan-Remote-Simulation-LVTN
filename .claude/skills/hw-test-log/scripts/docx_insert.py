"""Insert test-log figures and text into the thesis .docx, cloning existing paragraphs.

  docx_insert.py insert IN.docx OUT.docx SPEC.json IMGDIR
  docx_insert.py pages  IN.docx OUT.docx CHECK.pdf OFFSET

insert: works on the XML only, so the formatting the author set by hand stays as it is.
Every new paragraph is a copy of an existing one with only its text changed.
SPEC.json:
  {
    "after":   "Việc gắn chung W25Q64 ...",      # prefix of the paragraph to insert after
    "sub":     "Kiểm chứng bộ nhớ Flash W25Q64",  # prefix of a heading to copy for "sub"
    "txt":     "Việc gắn chung W25Q64 ...",      # prefix of a one-run body paragraph for "txt"
    "fig":     "Hình 38",                         # caption whose picture + caption are copied
    "tof":     "Hình 42",                         # list-of-figures entry; new ones go after it
    "items":   [["sub", "..."], ["txt", "..."], ["img", "H43.png"], ["cap", 43, "..."]],
    "renumber": [[43, 47, "Đáp ứng nhiệt độ ..."]], # later figures: caption, list entry, "Hình N" refs
    "replace": [["old text", "new text"]]          # other text-only edits, each must match once
  }
New list-of-figures entries get page "00"; run "pages" afterwards.

pages: sets every TOC / list-of-figures page number from a PDF export of the
document. OFFSET = PDF page index (from 0) minus the printed page number.
"""
import json, random, re, sys, zipfile
from html import escape

PARA = r"<w:p[ >].*?</w:p>"
T = r"<w:t[^>]*>([^<]*)</w:t>"


def txt(p):
    return "".join(re.findall(T, p))


def load(src):
    z = zipfile.ZipFile(src)
    return z, z.read("word/document.xml").decode("utf-8"), z.read("word/_rels/document.xml.rels").decode("utf-8")


def save(z, dst, x, rels, media=()):
    out = zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED)
    for it in z.infolist():
        data = z.read(it.filename)
        if it.filename == "word/document.xml":
            data = x.encode("utf-8")
        elif it.filename == "word/_rels/document.xml.rels":
            data = rels.encode("utf-8")
        out.writestr(it, data)
    for path, name in media:
        out.write(path, "word/media/" + name)
    out.close()


def one(paras, pred, what):
    hit = [i for i, p in enumerate(paras) if pred(p)]
    assert len(hit) == 1, "%s: %d matches" % (what, len(hit))
    return hit[0]


def insert(src, dst, spec, imgdir):
    from PIL import Image
    z, x, rels = load(src)
    paras = re.findall(PARA, x.split("<w:body>")[1], flags=re.S)
    S = json.load(open(spec, encoding="utf-8"))
    is_cap = lambda p: 'w:val="Caption"' in p
    LAST = paras[one(paras, lambda p: txt(p).startswith(S["after"]), "after")]
    SUB = paras[one(paras, lambda p: txt(p).startswith(S["sub"]), "sub")]
    TXT = paras[one(paras, lambda p: txt(p).startswith(S["txt"]), "txt")]
    ci = one(paras, lambda p: is_cap(p) and txt(p).startswith(S["fig"] + ":"), "fig")
    CAP, IMG = paras[ci], paras[ci - 1]
    assert "<w:drawing>" in IMG, "no picture right before " + S["fig"]
    assert x.count(LAST) == 1

    used = set(re.findall(r'w14:paraId="([0-9A-F]{8})"', x))

    def new_id():
        while True:
            v = "%08X" % random.randint(0x10000000, 0x7FFFFFFF)
            if v not in used:
                used.add(v)
                return v

    def fresh(p):
        return re.sub(r'w14:paraId="[0-9A-F]{8}"', lambda m: 'w14:paraId="%s"' % new_id(), p, count=1)

    def set_text(p, text):
        runs = re.findall(r"<w:r>.*?</w:r>|<w:r [^>]*>.*?</w:r>", p, flags=re.S)
        assert len(runs) == 1, "template must have one run, has %d" % len(runs)
        r = re.sub(T, lambda m: '<w:t xml:space="preserve">%s</w:t>' % escape(text, quote=False), runs[0])
        return fresh(p).replace(runs[0], r)

    def set_caption(p, num, text, page=None):
        """Number in the first digit-only run (SEQ result or "Hình N"), text in the run starting with ':'."""
        ts = list(re.finditer(T, p))
        k = next(i for i, m in enumerate(ts) if re.fullmatch(r"(Hình )?\d+", m.group(1)))
        c = next(i for i, m in enumerate(ts) if m.group(1).startswith(":"))
        edits = {k: re.sub(r"\d+", str(num), ts[k].group(1)), c: ": " + text}
        end = len(ts) - 1 if page is not None else len(ts)
        for i in range(c + 1, end):
            edits[i] = ""                                  # rest of the old caption text
        if page is not None:
            edits[len(ts) - 1] = page
        for i in sorted(edits, reverse=True):
            m = ts[i]
            p = p[:m.start()] + '<w:t xml:space="preserve">%s</w:t>' % escape(edits[i], quote=False) + p[m.end():]
        return fresh(p)

    rid = max(int(i) for i in re.findall(r'Id="rId(\d+)"', rels))
    docpr = max(int(i) for i in re.findall(r'<wp:docPr id="(\d+)"', x))
    ex, ey = map(int, re.search(r'<wp:extent cx="(\d+)" cy="(\d+)"/>', IMG).groups())
    media, out, caps = [], [], []
    for item in S["items"]:
        kind = item[0]
        if kind == "sub":
            out.append(set_text(SUB, item[1]))
        elif kind == "txt":
            out.append(set_text(TXT, item[1]))
        elif kind == "cap":
            out.append(set_caption(CAP, item[1], item[2]))
            caps.append((item[1], item[2]))
        elif kind == "img":
            rid += 1; docpr += 1
            name = "log_" + item[1]
            w, h = Image.open("%s/%s" % (imgdir, item[1])).size
            cx, cy = ex, int(ex * h / w)                   # same width as the template figure
            p = fresh(IMG).replace("<w:lastRenderedPageBreak/>", "")
            p = re.sub(r'r:embed="rId\d+"', 'r:embed="rId%d"' % rid, p)
            p = re.sub(r'<wp:extent cx="\d+" cy="\d+"/>', '<wp:extent cx="%d" cy="%d"/>' % (cx, cy), p)
            p = re.sub(r'<a:ext cx="\d+" cy="\d+"/>', '<a:ext cx="%d" cy="%d"/>' % (cx, cy), p)
            p = re.sub(r'<wp:docPr id="\d+" name="[^"]*"', '<wp:docPr id="%d" name="Picture %d"' % (docpr, docpr), p)
            p = re.sub(r'wp14:anchorId="[0-9A-F]+" wp14:editId="[0-9A-F]+"',
                       'wp14:anchorId="%s" wp14:editId="%s"' % (new_id(), new_id()), p)
            rels = rels.replace("</Relationships>", '<Relationship Id="rId%d" Type="http://schemas.openxmlformats.org/'
                                'officeDocument/2006/relationships/image" Target="media/%s"/></Relationships>' % (rid, name))
            media.append(("%s/%s" % (imgdir, item[1]), name))
            out.append(p)
    x = x.replace(LAST, LAST + "".join(out), 1)

    for old, new, cap_text in S.get("renumber", []):
        allp = re.findall(PARA, x, flags=re.S)
        c = allp[one(allp, lambda p: is_cap(p) and cap_text in txt(p), "renumber %d" % old)]
        x = x.replace(c, re.sub(r"(<w:t[^>]*>)%d(</w:t>)" % old, r"\g<1>%d\g<2>" % new, c, count=1))
        x = re.sub(r"(<w:t[^>]*>)Hình %d(</w:t>)" % old, r"\g<1>Hình %d\g<2>" % new, x)
        x = re.sub(r"Hình %d(?=[ .,;)])" % old, "Hình %d" % new, x)

    for a, b in S.get("replace", []):
        assert x.count(a) == 1, a
        x = x.replace(a, b)

    if caps:
        allp = re.findall(PARA, x, flags=re.S)
        tof = allp[one(allp, lambda p: 'TableofFigures' in p and txt(p).startswith(S["tof"] + ":"), "tof")]
        x = x.replace(tof, tof + "".join(set_caption(tof, n, t, "00") for n, t in caps), 1)

    save(z, dst, x, rels, media)
    print("inserted %d paragraphs, %d images, %d captions" % (len(out), len(media), len(caps)))


def pages(src, dst, pdf, offset):
    import pypdf
    norm = lambda s: re.sub(r"\s+", "", s)
    text = [norm(p.extract_text()) for p in pypdf.PdfReader(pdf).pages]
    z, x, rels = load(src)
    for p in re.findall(PARA, x, flags=re.S):
        if not re.search(r'<w:pStyle w:val="(TOC\d|TableofFigures)"', p):
            continue
        ts = list(re.finditer(T, p))
        if not ts or not ts[-1].group(1).strip().isdigit():
            continue
        key = norm("".join(m.group(1) for m in ts[:-1]))[:40]
        hits = [i - offset for i in range(offset + 1, len(text)) if key in text[i]]
        shown = ts[-1].group(1)
        if not hits:
            print("not found:", key)
            continue
        if int(shown) in hits:
            continue
        want = str(hits[0])
        print("%4s -> %4s  %s" % (shown, want, key))
        m = ts[-1]
        q = p[:m.start()] + m.group(0).replace(">%s<" % shown, ">%s<" % want) + p[m.end():]
        assert x.count(p) == 1
        x = x.replace(p, q)
    save(z, dst, x, rels)


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "insert":
        insert(*a[1:5])
    elif a[0] == "pages":
        pages(a[1], a[2], a[3], int(a[4]))
    else:
        sys.exit(__doc__)
