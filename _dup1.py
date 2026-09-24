# -*- coding: utf-8 -*-
import os, re, glob, collections

PAIRS = []
for d in ["Modules", "Modules/video_bridge", "Algorithm", "Devices", "BSP", "Task"]:
    for c in sorted(glob.glob(os.path.join(d, "*.c"))):
        h = c[:-2] + ".h"
        if os.path.exists(h):
            PAIRS.append((c, h))

def txt(p):
    return open(p, "rb").read().decode("gbk", errors="replace").replace("\r\n", "\n")

BLK = re.compile(r"/\*.*?\*/", re.S)
def comments(t):
    out = []
    for m in BLK.finditer(t):
        body = m.group(0)
        body = re.sub(r"^/\*+|\*/$", "", body).strip()
        body = "\n".join(re.sub(r"^\s*\*+\s?", "", l).rstrip() for l in body.split("\n"))
        body = re.sub(r"\s+", " ", body).strip(" ").rstrip("*").strip()
        if len(body) > 8:
            out.append(body)
    for m in re.finditer(r"//[^\n]*", t):
        b = re.sub(r"^//+\s*", "", m.group(0)).strip()
        if len(b) > 12:
            out.append(re.sub(r"\s+", " ", b))
    return out

print("### .c/.h 重复注释（长度>8，逐字相同）\n")
total = 0
for c, h in PAIRS:
    tc, th = txt(c), txt(h)
    cc, ch = collections.Counter(comments(tc)), set(comments(th))
    dup = [(k, v) for k, v in cc.items() if k in ch]
    if dup:
        total += len(dup)
        print("== %-42s 重复 %d 条" % (h.replace("\\","/"), len(dup)))
        for k, v in dup:
            print("   x%d  %s" % (v, k[:110]))
print("\nTOTAL dup blocks:", total)
