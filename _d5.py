# -*- coding: utf-8 -*-
import re
def rd(p):
    raw = open(p, "rb").read()
    try: return raw.decode("utf-8")
    except UnicodeDecodeError: return raw.decode("gbk", errors="replace")
NAMES = {
 "Algorithm/smc.h":["SMC_Calc","sat_func"],
 "Devices/servo.h":["Servo_Set_Angle","VTM_Servo_Control_Loop","Magnifier_Servo_Control"],
 "Modules/shooter_control.h":["Shooter_Reset_Stats"],
 "BSP/bsp_ano_protocol.h":["ANO_Print_Wave","VOFA_Print_Wave","Print_Task","Ano_Print_Task"],
 "Modules/can_transmit.h":["LK_Send_Torque","float_to_uint"],
 "Modules/shooter_control.h ":["Shooter_Reset_Stats"],
}
for p, nms in NAMES.items():
    p = p.strip()
    t = rd(p).replace("\r\n", "\n"); L = t.split("\n")
    print("=" * 12, p)
    for i, l in enumerate(L):
        for n in nms:
            if re.search(r"\b" + n + r"\s*\(", l) and ";" in l:
                for x in L[max(0, i - 3):i + 1]:
                    print("   ", x)
                print("   ---")
