# 26_Hero_Gimbal 英雄机器人 · 云台板电控

## 简介
- 项目：RoboMaster 英雄机器人云台板（电控）
- MCU：STM32H723VGTx
- 系统：FreeRTOS（CMSIS-RTOS v2）+ STM32Cube HAL
- 通信：FDCAN1 / FDCAN2 / FDCAN3
- 遥控：DR16（主用）、VT03（备用）
- 状态：比赛验证版（已完成实车验证）

## 主要功能
- 云台控制：Yaw（LK 电机力矩控制）、Pitch（DJI 电机）；支持 IMU 闭环（SMC）与编码器闭环（PID）两套模式
- 瞄准：手操、自瞄、吊射、基地打击，与视觉（vision_link）协同
- 射击：拨盘（DM 电机）+ 摩擦轮（DJI 电机），含卡弹检测/回弹、弹速统计
- 底盘指令转发：速度、模式、超级电容、爬坡、LED、基地锁定
- 安全：IMU 未就绪强制停机、遥控断连自动失能、长按/按键复位

## 目录结构
- `Core/`：HAL 外设初始化、`main.c`、FreeRTOS 配置
- `Algorithm/`：PID（全项目统一一套）、SMC、Ramp、Mahony、EKF、Kalman、滤波等算法
- `BSP/`：DR16、VT03、ANO 协议、DWT、蜂鸣器等板级驱动
- `Devices/`：BMI088 IMU、WS2812、舵机(servo) 等器件驱动
- `Task/`：控制(control_task)、CAN接收(can_task)、打印(print_task)、IMU、device、vision、video、monitor 等 FreeRTOS 任务
- `Modules/`：机器人调度(robot_control)、模式(robot_mode)、云台(gimbal_control)、底盘(chassis_control)、射击(shooter_control)、CAN收发(can_transmit/can_receive)、视觉(vision_link)、图传链路(video_bridge) 等模块
- `docs/`：架构说明（`docs/architecture.md`）
- `Drivers/ Middlewares/`：ST HAL、CMSIS、FreeRTOS、USB 库

## 编译与烧录
1. 用 Keil MDK 打开 `MDK-ARM/h723_cloud.uvprojx`
2. 编译（应 0 Error）
3. 通过 J-Link / DAP 烧录到云台板

## 编码约定
- 现有 `.c/.h` 源文件为 **GBK** 编码，请勿用会批量转换编码的工具处理
- 例外：`Algorithm/` 下移植的算法文件（`LADRC.c`、`QuaternionEKF.c/.h`、`kalman_filter.c/.h`）与 `smc.c/.h`、`BSP/bsp_buzzer.c` 为 UTF-8，保持移植原样
- `README.md` 与 `docs/` 下的文档为 UTF-8

## 控制模式（Robot_Mode_e）
| 模式 | 说明 |
|---|---|
| OFF | 失能（默认） |
| GIMBAL_ONLY | 仅云台 |
| CHASSIS_ONLY | 仅底盘 |
| FOLLOW | 底盘跟随云台 |
| SHOOT / SHOOT_LOW | 射击（编码器闭环，低速/高速摩擦轮） |
| AIMING / M_AIMING | 自瞄 / 手动自瞄 |
| CLIMB | 爬坡 |
| BATTLE | 综合战斗（键鼠操作） |

## CAN 分配（关键）
| 总线 | 帧 ID | 方向 / 含义 |
|---|---|---|
| FDCAN1 | 0x141 | LK Yaw：TX 力矩指令 / RX 电机反馈 |
| FDCAN1 | 0x300 | TX 云台 → 底盘控制指令（速度/模式/超电/上坎/LED/摩擦轮状态/基地锁定） |
| FDCAN1 | 0x308 | TX 云台 → 底盘 UI 数据（50ms 周期：云台角度 + 摩擦轮转速） |
| FDCAN1 | 0x301 | TX 拨盘模式指令（0xFC 使能 / 0xFD 失能） |
| FDCAN1 | 0x303 | TX 拨盘速度指令（float，4 字节） |
| FDCAN1 | 0x302 | RX 拨盘反馈（达妙电机） |
| FDCAN1 | 0x304 | RX 底盘双板反馈（颜色 / 裁判系统数据转发） |
| FDCAN2 | 0x200 | TX 摩擦轮外圈电流（电机 ID 1/2/3） |
| FDCAN2 | 0x1FF | TX 摩擦轮内圈电流（电机 ID 5/6/7） |
| FDCAN2 | 0x201 ~ 0x207 | RX 摩擦轮反馈（电机 ID 4 未使用） |
| FDCAN3 | 0x1FF | TX Pitch 电流 |
| FDCAN3 | 0x205 | RX Pitch 反馈 |

## 安全须知
- 上电后默认 `OFF`，需要遥控切换到对应模式才使能
- IMU 未就绪时控制循环会强制清零所有输出
- 遥控断连超过阈值自动进入 `OFF`
- 上电/测试时注意摩擦轮、拨盘等运动部件

## 废弃说明
- `Modules/Vision_can.c/.h`：早期 CAN 视觉实现，已被 `vision_link`（USB CDC）取代，未加入 Keil 工程编译，文件保留备查
