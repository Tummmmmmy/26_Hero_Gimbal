# -*- coding: utf-8 -*-
import re, glob, os
def rd(p):
    raw = open(p, "rb").read()
    try: return raw.decode("utf-8")
    except UnicodeDecodeError: return raw.decode("gbk", errors="replace")

FILES = []
for d in ["Modules", "Modules/video_bridge", "Task", "BSP"]:
    FILES += sorted(glob.glob(os.path.join(d, "*.c")))
FILES += ["Algorithm/pid.c", "Algorithm/smc.c", "Devices/servo.c"]

for f in FILES:
    t = rd(f).replace("\r\n", "\n")
    m = re.match(r"\s*/\*\*(.*?)\*/", t, re.S)
    if not m:
        print("== %-44s (无文件头)" % f.replace("\\", "/")); continue
    body = m.group(1)
    lines = [re.sub(r"^\s*\*?\s?", "", x).rstrip() for x in body.strip().split("\n")]
    print("== %s" % f.replace("\\", "/"))
    for x in lines:
        print("     %s" % x)
