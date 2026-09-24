# -*- coding: utf-8 -*-
import os, re, glob

SCOPE_C = []
for d in ["Modules", "Modules/video_bridge"]:
    SCOPE_C += sorted(glob.glob(os.path.join(d, "*.c")))
SCOPE_C += ["Algorithm/pid.c", "Algorithm/smc.c", "Devices/servo.c", "BSP/bsp_ano_protocol.c"]

def txt(p): return open(p, "rb").read().decode("gbk", errors="replace").replace("\r\n", "\n")

print("### 范围内 .c 里所有 /* */ 注释块\n")
for c in SCOPE_C:
    t = txt(c); L = t.split("\n")
    blocks = []
    i = 0
    while i < len(L):
        if "/*" in L[i]:
            j = i
            while j < len(L) and "*/" not in L[j]:
                j += 1
            body = "\n".join(L[i:j+1])
            k = j + 1
            while k < len(L) and (not L[k].strip()):
                k += 1
            nxt = L[k].strip()[:64] if k < len(L) else ""
            nlines = j - i + 1
            blocks.append((i+1, nlines, body, nxt))
            i = j + 1
        else:
            i += 1
    if not blocks:
        print("== %s  (无 /* */ 块)" % c.replace("\\","/")); continue
    print("== %s" % c.replace("\\","/"))
    for ln, nl, body, nxt in blocks:
        first = body.split("\n")[0].strip()
        tag = "★紧跟函数" if nxt and re.match(r"^[A-Za-z_].*\(", nxt) and "=" not in nxt.split("(")[0] else "  "
        print("   L%-4d %2d行 %s | 后接: %s" % (ln, nl, first[:58], nxt[:52]))
        if nl > 1 and tag == "★紧跟函数":
            for x in body.split("\n")[1:]:
                print("            %s" % x.strip()[:76])
