# -*- coding: utf-8 -*-
import os, re, glob

PAIRS = []
for d in ["Modules", "Modules/video_bridge", "Algorithm", "Devices", "BSP"]:
    for h in sorted(glob.glob(os.path.join(d, "*.h"))):
        c = h[:-2] + ".c"
        if os.path.exists(c):
            PAIRS.append((h, c))

def txt(p): return open(p, "rb").read().decode("gbk", errors="replace").replace("\r\n", "\n")

def api_docs(t):
    """返回 [(brief, 声明行)] —— /** @brief */ 紧贴其后的声明"""
    L = t.split("\n"); out = []
    for i, l in enumerate(L):
        s = l.strip()
        if s.startswith("/** @brief"):
            brief = re.sub(r"^/\*\*\s*@brief\s*", "", s).rstrip("*/").strip()
            j = i + 1
            while j < len(L) and (not L[j].strip() or L[j].strip().startswith("//")):
                j += 1
            if j < len(L):
                out.append((brief, L[j].strip()))
    return out

def fn_name(decl):
    d = re.sub(r"//.*", "", decl).strip()
    m = re.match(r"(?:static\s+)?(?:const\s+)?(?:inline\s+)?[A-Za-z_][A-Za-z0-9_ \*]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(", d)
    return m.group(1) if m else None

print("### 同一 API 在 .h 与 .c 都写了 @brief\n")
tot = 0
for h, c in PAIRS:
    dh = [(b, fn_name(d)) for b, d in api_docs(txt(h))]
    dc = [(b, fn_name(d)) for b, d in api_docs(txt(c))]
    hm = {n: b for b, n in dh if n}
    hit = [(n, b, hm[n]) for b, n in dc if n in hm]
    if hit:
        tot += len(hit)
        print("== %s" % h.replace("\\", "/"))
        for n, bc, bh in hit:
            mark = "完全同文" if bc == bh else "同义/近似"
            print("   %-28s [%s]  .h: %s | .c: %s" % (n, mark, bh[:44], bc[:44]))
print("\nTOTAL:", tot)
