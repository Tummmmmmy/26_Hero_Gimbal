# -*- coding: utf-8 -*-
import re, os, glob
def rd(p):
    raw = open(p, "rb").read()
    try: return raw.decode("utf-8")
    except UnicodeDecodeError: return raw.decode("gbk", errors="replace")

SCOPE = []
for d in ["Modules", "Modules/video_bridge"]:
    SCOPE += sorted(glob.glob(os.path.join(d, "*.c")))
SCOPE += ["Algorithm/pid.c", "Algorithm/smc.c", "Devices/servo.c", "BSP/bsp_ano_protocol.c"]

def find(p): return re.search(r"^[A-Za-z_][A-Za-z0-9_ \*]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(", p)

def hbriefs(hp):
    if not os.path.exists(hp): return {}
    out = {}
    t = rd(hp).replace("\r\n", "\n").split("\n")
    for i, l in enumerate(t):
        m = re.match(r"\s*/\*\*\s*@brief\s*(.*?)\s*\*/", l)
        if m:
            j = i + 1
            while j < len(t) and (not t[j].strip() or t[j].strip().startswith("//") or t[j].strip().startswith("*")):
                j += 1
            if j < len(t):
                f = find(t[j])
                if f: out[f.group(1)] = m.group(1)
    return out

print("### .c 中“紧贴函数定义”的 // 注释 vs .h 里的 @brief\n")
hits = 0
for c in SCOPE:
    t = rd(c).replace("\r\n", "\n"); L = t.split("\n")
    hb = hbriefs(c[:-2] + ".h")
    if not hb: continue
    rows = []
    for i, l in enumerate(L):
        s = l.strip()
        if s.startswith("//") and not s.startswith("// =====  ") and "====" not in s:
            j = i + 1
            while j < len(L) and (L[j].strip().startswith("//") or not L[j].strip()):
                j += 1
            if j < len(L) and L[j].strip() and not L[j].strip().startswith(("#", "}", "/", "return")):
                f = find(L[j])
                if f and f.group(1) in hb and "(" in L[j] and ";" not in L[j]:
                    rows.append((i + 1, s, f.group(1), hb[f.group(1)]))
    if rows:
        hits += len(rows)
        print("== %s" % c.replace("\\", "/"))
        for ln, cm, fn, br in rows:
            print("   L%-4d %-30s | .h: %s" % (ln, fn, br[:46]))
            print("         .c: %s" % cm[:80])
print("\nTOTAL:", hits)
