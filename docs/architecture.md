# 云台板电控架构说明

> 本文件描述 `26_Hero_Gimbal` 的代码结构与本次工程化重构的结果（“比赛验证版”，已完成实车验证）。

## 1. 现状（重构前）

> 注：整体重构已完成（2026-09），本节保留重构前的状态描述。

当前应用层逻辑高度集中在 `Core/Src/main.c`（约 4363 行），其中 `Robot_Control_Loop()`（约 1475 行）同时承担了：

- 遥控解析与模式映射（DR16 / VT03）
- 模式状态机（OFF / FOLLOW / GIMBAL / CHASSIS / SHOOT / SHOOT_LOW / AIMING / M_AIMING / CLIMB / BATTLE）
- 云台 IMU(SMC) / 编码器(PID) 控制
- 视觉自瞄、吊射、基地打击
- 底盘速度、跟随、坐标变换、Ramp、上坎
- 射击（摩擦轮、拨盘、弹速统计）
- 所有 CAN 发送与 UI 输出
- 安全停机、复位、电机重连

`main.c` 还包含：`BSP_FDCAN_Config`、`HAL_FDCAN_RxFifo0Callback`、`Can_Decode_Task`、`Gimbal_Control_task`、`Print_Entry_Task`、电机/CAN 发送函数、自己的 PID 实现等。

## 2. 目标架构

应用层按“职责域”拆成以下模块（放在现有 `Modules/`）：

```text
Robot_Control_Loop()                     ← 总调度器（Modules/robot_control.c）
    ├─ 就绪检查 / 安全停机 / 复位
    ├─ Robot_Mode_Update()               ← 模式状态机（Modules/robot_mode.c）
    ├─ Gimbal_Control_Update()           ← 云台 + 瞄准（Modules/gimbal_control.c）
    ├─ Chassis_Update_*()                ← 底盘 + 上坎（Modules/chassis_control.c）
    ├─ Shooter_Control_Update()          ← 射击（Modules/shooter_control.c）
    └─ 统一输出
```

CAN 分三层：

```text
应用层（gimbal/chassis/shooter）
        ↓
Modules/can_transmit.c   协议打包：LK / DJI / DM / Dial / Chassis / UI
        ↓
BSP/bsp_can.c            硬件收发：FDCAN 初始化、Filter、BSP_CAN_Send、RX 中断
        ↓
HAL_FDCAN
```

接收方向：

```text
FDCAN 中断 → BSP/bsp_can.c → osMessageQueuePut(canRxQueueHandle)
     → Task/can_task.c（Can_Decode_Task 取帧）
     → Modules/can_receive.c（按 ID 解码 → 电机/底盘/裁判反馈）
     → gimbal / chassis / shooter 模块
```

任务归属目标：

```text
Task/
├── INS_task.c        IMU 数据生产
├── device_task.c     设备任务
├── vision_task.c     视觉任务
├── video_task.c      视频 bridge
├── monitor_task.c    监控任务
├── control_task.c    Gimbal_Control_task（1ms 控制任务）
├── print_task.c      Print_Entry_Task
└── can_task.c        Can_Decode_Task
```

## 3. 数据流

```text
bsp_DR16 / bsp_VT03 ──RC_Data_t──┐
INS_task             ──IMU_Data_t─┤
vision_link          ──VisionData─┤
                                  ▼
                        Robot_Control_Loop()
                          ├─ 就绪检查(未就绪则零发并返回)
                          ├─ Robot_Mode_Update() + 复位(滚轮长按 / V键)
                          ├─ 按模式翻译指令(Gimbal/Chassis/Shooter 目标写入)
                          ├─ Gimbal_Control_Update()   → Gimbal_Output_t
                          ├─ Chassis_Update_*()        → Chassis_Output_t
                          ├─ Shooter_Control_Update()  → Shooter_Output_t
                          └─ 统一CAN输出(内联, can_transmit) → BSP/bsp_can → FDCAN
```

## 4. 状态归属（目标）

| 当前位置 | 目标归属 |
|---|---|
| `Robot_Mode_e / Gimbal_Mode_e / Climb_Mode_e / Battle_Mode_e` | `Modules/robot_mode.h` |
| `PID_Controller` + `PID_Init / PID_Calculate / PID_Cal_Enhanced / PID_Cal_Increment` | `Algorithm/pid.h/.c`（全项目统一 PID） |
| `IMU_Data_t` | `Task/INS_task.h` |
| `Shooter_Controller_t` | `Modules/shooter_control.h` |
| `gimbal_yaw/pitch_motor、SMC、rc_target_*、vision_target_*` | `Modules/gimbal_control.c`（static） |
| `send_*、supercap/climb/led_cmd、climb_mode、chassis_*_f` | `Modules/chassis_control.c`（`Chassis_Output_t` + static） |
| `dial_motor、friction_*、fric_level、high/low_speed、弹速统计` | `Modules/shooter_control.c` |
| `reset_trigger_timer、key_*_last_state` | `Modules/robot_control.c` |
| 中间变量 `raw_*、yaw_angle_diff、yaw_rad_diff、角度增量` | 各模块内部局部变量 |

## 5. 历史 / 废弃代码

- `Modules/Vision_can.c/.h`：历史 CAN 视觉实现，已确认无调用，不纳入主架构，已移出 Keil 工程编译（文件保留）。
- 哈工程遗留 PID（原 `Algorithm/pid.c/.h`）：已于 2026-09 合并进统一 `Algorithm/pid.h/.c`；`INS_task` 温度 PID 改用同一实现，参数/计算逻辑/限幅/调用周期不变。
- `Bullet_Speed_Adaptive_Control()`：已确认有调用（`shooter_control.c` 内部），保留；弹速统计函数保留。

## 6. 重构节奏

每步以“Keil 0 Error → 烧录 → 实车/上电验证 → git commit”为门槛，不一次性大改：

1. 基线、README、架构文档（本步） ✅
2. 只搬归属明确的类型/枚举 ✅
3. `BSP/bsp_can`（初始化 + 发送原语 + RX 中断） ✅
4. `Modules/can_transmit`（协议发送） ✅
5. `Modules/shooter_control` ✅
6. `Modules/gimbal_control`（含瞄准） ✅
7. `Modules/chassis_control`（含上坎） ✅
8. `Modules/robot_mode` + `Modules/robot_control` + 任务归位 ✅
9. `Modules/can_receive` + `Task/can_task`、死代码清理、收尾文档 ✅
10. 收尾清理: 视觉目标解析去重、AIMING/M_AIMING 公共主体提取、V键复位去重、死代码清理、文档修正 ✅
11. 目录/命名整理: Servo_control→Devices/servo、Control_Vision→vision_link、bridge 子系统→Modules/video_bridge/、清理空 Control 目录与无效包含路径 ✅
12. PID 合并 + 命名规范: 两套 PID 统一为 Algorithm/pid.h/.c（保留自有实现，IMU 温度 PID 行为不变）；video_bridge 文件更名为 video_bridge.c/video_bridge_platform.c ✅

## 7. 编码约定

- 源码（`.c/.h`）保持 GBK，不随重构转换编码。
- 文档（`.md`）为 UTF-8。

## 8. 完成状态（2026-09）

重构完成后的最终调用链：

```text
Robot_Control_Loop (Modules/robot_control.c, 1ms 调度)
    ├─ Robot_Mode_Update      Modules/robot_mode.c         模式选择
    ├─ Gimbal_Control_*       Modules/gimbal_control.c     云台/瞄准
    ├─ Chassis_Control_*      Modules/chassis_control.c    底盘/上坎
    ├─ Shooter_Control_*      Modules/shooter_control.c    射击
    └─ 统一 CAN 输出          Modules/can_transmit.c → BSP/bsp_can.c

接收: FDCAN 中断 → BSP/bsp_can.c → Task/can_task.c → Modules/can_receive.c → 各模块解码
```

`main.c` 仅保留 HAL 初始化与系统入口。
