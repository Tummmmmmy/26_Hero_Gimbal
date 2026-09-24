# -*- coding: utf-8 -*-
import os, re, glob

TARGETS = ["Modules", "Modules/video_bridge", "Algorithm", "Devices", "BSP", "Task"]
FUN = re.compile(r"^[A-Za-z_][A-Za-z0-9_ \*]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(", re.M)

def txt(p): return open(p, "rb").read().decode("gbk", errors="replace").replace("\r\n","\n")

print("### .c 中自带 Doxygen 块（/** */）的位置\n")
for d in TARGETS:
    for c in sorted(glob.glob(os.path.join(d, "*.c"))):
        t = txt(c)
        lines = t.split("\n")
        hits = []
        for i, l in enumerate(lines):
            if l.strip() == "/**":
                # 找到注释块结束和紧随其后的代码行
                j = i + 1
                while j < len(lines) and "*/" not in lines[j]:
                    j += 1
                k = j + 1
                while k < len(lines) and not lines[k].strip():
                    k += 1
                nxt = lines[k].strip() if k < len(lines) else ""
                hits.append((i + 1, nxt[:70]))
        if hits:
            print("== %s  (%d 处)" % (c.replace("\\","/"), len(hits)))
            for ln, nxt in hits:
                print("   L%-5d %s" % (ln, nxt))
print()

print("### .c 文件头（前 6 行）\n")
for d in TARGETS:
    for c in sorted(glob.glob(os.path.join(d, "*.c"))):
        t = txt(c)
        print("== %s" % c.replace("\\","/"))
        print("\n".join("   " + x for x in t.split("\n")[:6]))
