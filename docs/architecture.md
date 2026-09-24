# 云台板电控架构说明

> 本文描述 `26_Hero_Gimbal`（RoboMaster 英雄机器人云台板）的代码结构、数据流与状态归属。

## 1. 模块划分

应用层按职责域拆分为以下模块（位于 `Modules/`）：

```text
Robot_Control_Loop()                     ← 总调度器（Modules/robot_control.c）
    ├─ 就绪检查 / 安全停机 / 复位
    ├─ Robot_Mode_Update()               ← 模式状态机（Modules/robot_mode.c）
    ├─ Gimbal_Control_Update()           ← 云台 + 瞄准（Modules/gimbal_control.c）
    ├─ Chassis_Update_*()                ← 底盘 + 上坎（Modules/chassis_control.c）
    ├─ Shooter_Control_Update()          ← 射击（Modules/shooter_control.c）
    └─ 统一输出
```

模块之间只通过结构体接口交互，不直接访问彼此的内部状态：

| 方向 | 接口 |
|---|---|
| 上层 → 云台 | `Gimbal_Set_*` 系列目标角接口 |
| 上层 → 底盘 | `Chassis_Update_*()` |
| 上层 → 射击 | `Shooter_Context_t` → `Shooter_Control_Update()` |
| 模块 → 上层 | `Gimbal_Output_t` / `Chassis_Output_t` / `Shooter_Output_t` 只读快照 |

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
     → Modules/can_receive.c（按总线/ID 分拣到各模块）
     → gimbal / chassis / shooter 模块内部解析字节
```

任务划分：

```text
Task/
├── INS_task.c        IMU 数据生产
├── device_task.c     设备任务（WS2812 状态指示）
├── vision_task.c     视觉任务
├── video_task.c      图传 bridge
├── monitor_task.c    电机在线检测
├── control_task.c    Gimbal_Control_task（1ms 控制任务）
├── print_task.c      Print_Entry_Task
└── can_task.c        Can_Decode_Task
```

## 2. 数据流

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

## 3. 状态归属

| 状态 | 归属 |
|---|---|
| `Robot_Mode_e / Gimbal_Mode_e / Climb_Mode_e / Battle_Mode_e` | `Modules/robot_mode.h` |
| `LED_State_e` | `Modules/robot_types.h` |
| `Motor_Feedback / Motor_Controller` | `Devices/motor.h` |
| `PID_Controller` + `PID_Init / PID_Calculate / PID_Cal_Enhanced / PID_Cal_Increment` | `Algorithm/pid.h/.c`（全项目统一 PID） |
| `IMU_Data_t` | `Task/INS_task.h` |
| `Shooter_Context_t / Shooter_Output_t` | `Modules/shooter_control.h` |
| `gimbal_yaw/pitch_motor、SMC、rc_target_*、vision_target_*` | `Modules/gimbal_control.c`（static） |
| `send_*、supercap/climb/led_cmd、climb_mode、chassis_*_f` | `Modules/chassis_control.c`（`Chassis_Output_t` + static） |
| `dial_motor、friction_*、fric_level、high/low_speed、弹速统计` | `Modules/shooter_control.c` |
| `reset_trigger_timer、key_*_last_state` | `Modules/robot_control.c` |
| `CAN_Rx_Message_t` | `BSP/bsp_can.h` |

## 4. 编码约定

- 源码（`.c/.h`）为 GBK 编码，请勿用会批量转换编码的工具处理。
- 文档（`.md`）为 UTF-8。

## 5. 废弃代码

- `Modules/Vision_can.c/.h`：早期 CAN 视觉实现，已被 `vision_link`（USB CDC）取代，未加入 Keil 工程编译，文件保留备查。
- `Bullet_Speed_Adaptive_Control()`：仍在 `shooter_control.c` 内部正常调用，保留。

## 6. 调用链总览

```text
Robot_Control_Loop (Modules/robot_control.c, 1ms 调度)
    ├─ Robot_Mode_Update      Modules/robot_mode.c         模式选择
    ├─ Gimbal_Control_*       Modules/gimbal_control.c     云台/瞄准
    ├─ Chassis_Control_*      Modules/chassis_control.c    底盘/上坎
    ├─ Shooter_Control_*      Modules/shooter_control.c    射击
    └─ 统一 CAN 输出          Modules/can_transmit.c → BSP/bsp_can.c

接收: FDCAN 中断 → BSP/bsp_can.c → Task/can_task.c → Modules/can_receive.c → 各模块解码
```

`Core/Src/main.c` 仅保留 HAL 初始化与系统入口。
