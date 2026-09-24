# -*- coding: utf-8 -*-
import re, glob, os
def rd(p):
    raw = open(p, "rb").read()
    try: return raw.decode("utf-8")
    except UnicodeDecodeError: return raw.decode("gbk", errors="replace")

FILES = []
for d in ["Modules", "Modules/video_bridge", "Task", "BSP"]:
    FILES += sorted(glob.glob(os.path.join(d, "*.c"))) + sorted(glob.glob(os.path.join(d, "*.h")))
FILES += ["Algorithm/pid.c", "Algorithm/pid.h", "Algorithm/smc.c", "Algorithm/smc.h",
          "Devices/servo.c", "Devices/servo.h"]
print("### 文件头最后一行（句尾标点检查）\n")
for f in FILES:
    t = rd(f).replace("\r\n", "\n")
    m = re.match(r"\s*/\*\*(.*?)\*/", t, re.S)
    if not m: continue
    body = [x.rstrip() for x in m.group(1).split("\n")]
    body = [re.sub(r"^\s*\*?\s?", "", x) for x in body]
    body = [x for x in body if x.strip() != ""]
    if len(body) < 3: continue
    last = body[-1].strip()
    if last.endswith(("，", ",", ";", "、")) or not last.endswith(("。", ")", "）", "接口")) and re.search(r"[\u4e00-\u9fff]", last):
        print("  %-46s %s" % (f.replace("\\", "/"), last[:78]))
