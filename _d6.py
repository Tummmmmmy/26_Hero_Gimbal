# -*- coding: utf-8 -*-
import re, glob, os
def rd(p):
    raw = open(p, "rb").read()
    try: return raw.decode("utf-8")
    except UnicodeDecodeError: return raw.decode("gbk", errors="replace")

h = rd("BSP/bsp_ano_protocol.h").replace("\r\n", "\n").split("\n")
print("--- bsp_ano_protocol.h 55-90")
for i in range(54, min(90, len(h))):
    print("%4d| %s" % (i+1, h[i]))

print("\n### .h 分组标题 vs .c 区块标题\n")
for d in ["Modules", "Modules/video_bridge"]:
    for hh in sorted(glob.glob(os.path.join(d, "*.h"))):
        cc = hh[:-2] + ".c"
        if not os.path.exists(cc): continue
        gh = re.findall(r"//\s*=+\s*([^=\n]+?)\s*=+", rd(hh))
        gc = re.findall(r"//\s*=+\s*([^=\n]+?)\s*=+", rd(cc))
        if gh or gc:
            print("== %s" % hh.replace("\\", "/"))
            print("   .h :", " | ".join(gh) if gh else "-")
            print("   .c :", " | ".join(gc) if gc else "-")
