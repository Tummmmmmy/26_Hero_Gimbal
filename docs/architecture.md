# RoboMaster英雄机器人云台板电控架构说明

本文用于说明 `26_Hero_Gimbal` 的软件结构、任务模型、主要数据流以及核心控制模块之间的关系。

该工程属于 RoboMaster 实车项目，代码结构是在实际比赛需求和调试过程中逐步形成的，并非严格按照某一种软件架构模板设计。

硬件侧的器件型号、CAN 接口与通信链路见 [硬件连接与通信拓扑](hardware-topology.png)。

---

## 1. 整体架构

程序可以从四个层次理解：

```text
┌─────────────────────────────────────────────┐
│                 FreeRTOS Tasks              │
│  IMU / Control / CAN / Vision / Monitor... │
└──────────────────────┬──────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────┐
│                  Modules                    │
│ Robot / Gimbal / Shooter / Chassis / CAN  │
└──────────────────────┬──────────────────────┘
                       │
             ┌─────────┴─────────┐
             ▼                   ▼
┌────────────────────┐  ┌────────────────────┐
│        BSP         │  │     Algorithm      │
│ CAN / RC / Timer   │  │ PID / SMC / AHRS   │
└─────────┬──────────┘  └─────────┬──────────┘
          │                       │
          └───────────┬───────────┘
                      ▼
             ┌──────────────────┐
             │     Devices      │
             │ IMU / Motor etc. │
             └──────────────────┘
```

这里的“层”主要用于理解代码职责，并不意味着模块之间完全没有直接依赖。

例如 CAN 接收、设备反馈和应用模块之间存在较直接的数据传递，这是典型的比赛型嵌入式工程结构。

---

## 2. FreeRTOS 任务模型

系统使用 FreeRTOS 进行任务调度，核心任务如下：

| 任务 | 主要周期 / 触发方式 | 主要职责 |
|---|---|---|
| `All_Init` | 启动阶段 | 完成 USB、CAN、遥控和机器人参数初始化 |
| `INSTask` | 1 ms | BMI088 读取、姿态解算、IMU 状态维护 |
| `Gimbal_Task` | 1 ms | 调用主控制循环并生成执行器输出 |
| `CanDecodeTask` | 事件驱动 | 从 CAN 消息队列读取数据并解析 |
| `VisionTask` | 周期任务 | 视觉通信及相关数据处理 |
| `VideoBridgeTask` | 周期任务 | 图传协议及 UART DMA 通信 |
| `MonitorTask` | 周期任务 | 电机在线状态等系统状态监控 |
| `DeviceTask` | 低频任务 | LED 等设备状态更新 |
| `AnoPrintTask` | 事件驱动 | 调试信息输出 |

控制相关任务以毫秒级周期运行，而 CAN 接收等通信处理则通过中断 + 消息队列的方式交给任务处理。

---

## 3. 系统启动流程

程序启动后首先完成 MCU 和外设初始化，然后启动 FreeRTOS：

```text
main()
  │
  ├── HAL_Init()
  ├── SystemClock_Config()
  ├── GPIO / DMA / FDCAN / SPI / UART 初始化
  ├── DWT 初始化
  ├── 部分设备初始化
  │
  ├── osKernelInitialize()
  ├── MX_FREERTOS_Init()
  │      ├── 创建任务
  │      ├── 创建消息队列
  │      └── 创建同步对象
  │
  └── osKernelStart()
          │
          ▼
       FreeRTOS
```

`All_Init` 在系统启动后继续完成部分依赖 FreeRTOS 的初始化工作。

---

## 4. 主控制链路

`Robot_Control_Loop()` 是应用层的核心控制入口。

每个控制周期主要完成：

1. 检查系统安全状态
2. 更新机器人工作模式
3. 根据当前模式生成云台、底盘和射击目标
4. 调用对应控制模块
5. 生成最终执行器输出
6. 通过 CAN 等接口发送

整体流程：

```text
DR16 / VT03
      │
      ▼
Remote Control
      │
      ├──────────────┐
      │              │
      ▼              ▼
  Robot Mode       Control Input
      │              │
      └──────┬───────┘
             ▼
    Robot_Control_Loop()
             │
      ┌──────┼──────┐
      ▼      ▼      ▼
   Gimbal Shooter Chassis
      │      │      │
      └──────┼──────┘
             ▼
          CAN TX
             │
             ▼
           Motors
```

同时，各模块的反馈通过 CAN、IMU 等路径返回：

```text
Motor / IMU / Chassis
          │
          ▼
      Feedback
          │
          ▼
   Module State Update
          │
          ▼
Robot_Control_Loop()
```

---

## 5. IMU 数据链路

BMI088 通过 SPI 获取陀螺仪、加速度计和温度数据。

基本流程：

```text
BMI088
  │
  ▼
SPI Read
  │
  ▼
INS Task
  │
  ├── 温度处理
  ├── 零偏处理
  └── 姿态解算
          │
          ▼
     IMU State
          │
          ├── Yaw
          ├── Pitch
          └── Gyroscope
```

工程中同时存在 EKF、Mahony 等姿态解算相关代码，实际控制所使用的数据以当前工程配置和控制链路为准。

IMU 就绪状态同时作为系统安全门控的一部分，在 IMU 尚未进入正常工作状态时禁止电机输出。

---

## 6. 云台控制

云台控制模块主要负责 Yaw 和 Pitch 两个轴。

### 6.1 IMU 闭环

```text
目标角度
   │
   ▼
角度误差
   │
   ▼
SMC
   │
   ▼
控制输出
   │
   ▼
CAN
   │
   ▼
云台电机
   ▲
   │
IMU / 角速度
```

SMC 用于根据角度误差和运动状态生成控制输出，并在不同瞄准模式下使用不同参数。

### 6.2 编码器闭环

编码器模式采用串级 PID：

```text
目标角度
   │
   ▼
位置环 PID
   │
   ▼
目标速度
   │
   ▼
速度环 PID
   │
   ▼
电机控制量
   │
   ▼
CAN
```

这种结构可以分别处理位置误差和速度响应。

### 6.3 模式切换

云台模块根据机器人当前工作模式，在 IMU 控制和编码器控制之间进行切换。

同时处理：

- Yaw / Pitch 独立控制
- 目标角度更新
- 电机反馈
- 电机重连
- Pitch 软限位
- 视觉瞄准相关参数

---

## 7. 射击控制

射击模块由：

- DM4340 拨盘
- 6 个摩擦轮

组成。

```text
                  Shooter
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
       Dial Motor          Friction Wheels
          │                     │
       Position              Speed PID
       / Speed                   │
          │                     │
          └──────────┬──────────┘
                     ▼
                  Fire Control
```

拨盘使用状态机管理射击流程，包含校位、准备、射击以及卡弹处理等状态。

摩擦轮根据射击模式设置目标速度，并根据电机反馈进行速度闭环控制。

同时结合裁判系统反馈处理射击热量等状态。

---

## 8. 底盘协同

云台板本身不直接控制底盘电机，而是生成底盘主控板所需要的控制指令。

```text
遥控输入
   │
   ▼
Robot Mode
   │
   ▼
Chassis Control
   │
   ├── vx
   ├── vy
   ├── wz
   ├── Follow
   ├── Climb
   └── Battle
   │
   ▼
CAN
   │
   ▼
Chassis Board
   │
   ▼
Chassis Motors
```

其中比较重要的是云台跟随逻辑：

```text
云台方向 / 车体方向
          │
          ▼
       角度误差
          │
          ▼
     Follow Control
          │
          ▼
        wz 指令
          │
          ▼
      Chassis Board
```

底盘板收到指令后再完成具体的底盘运动控制。

---

## 9. CAN 通信

CAN 是云台板与各电机、底盘之间的主要通信方式。

### 9.1 接收流程

```text
FDCAN RX Interrupt
        │
        ▼
HAL_FDCAN_RxFifo0Callback()
        │
        ▼
Message Queue
        │
        ▼
CanDecodeTask
        │
        ▼
CAN_Rx_Route()
        │
        ├── Yaw Feedback
        ├── Pitch Feedback
        ├── Shooter Feedback
        └── Chassis Feedback
```

中断主要负责接收和投递消息，具体解析放在任务中完成。

### 9.2 发送流程

```text
Robot Control
      │
      ├── Gimbal Output
      ├── Shooter Output
      └── Chassis Command
              │
              ▼
       CAN Transmit
              │
              ▼
          FDCAN TX
```

不同 CAN 总线分别承担云台、射击和底盘相关通信。

具体 ID 和协议定义以 `can_receive.c`、`can_transmit.c` 以及对应设备驱动为准。

---

## 10. 视觉通信

视觉模块通过 USB CDC 与 PC 端视觉程序通信。

发送方向主要提供：

- IMU 姿态
- 角速度
- 机器人相关状态

接收方向主要获取：

- 目标 Yaw
- 目标 Pitch
- 视觉瞄准状态

基本数据流：

```text
Robot IMU
    │
    ▼
Vision Link
    │
    ▼
USB CDC
    │
    ▼
Vision PC

Vision PC
    │
    ▼
USB CDC
    │
    ▼
Vision Link
    │
    ▼
Gimbal Control
```

视觉通信中包含数据合法性检查和协议校验。

---

## 11. 模块职责

### `robot_control`

系统控制入口。

负责协调：

- 工作模式
- 云台
- 射击
- 底盘
- 安全状态

不直接承担所有底层设备驱动。

### `robot_mode`

负责机器人工作模式的判断和切换。

### `gimbal_control`

负责 Yaw / Pitch：

- 目标角度
- SMC
- PID
- 电机反馈
- 限位
- 模式切换

### `shooter_control`

负责：

- 拨盘
- 摩擦轮
- 射击状态机
- 卡弹处理
- 热量相关逻辑

### `chassis_control`

负责生成发送给底盘板的控制指令，以及跟随、爬坡等相关逻辑。

### `can_receive`

负责 CAN 接收帧的路由和协议解析入口。

### `can_transmit`

负责各类 CAN 控制指令的打包和发送。

### `vision_link`

负责视觉通信协议。

---

## 12. 状态和数据归属

工程中的主要状态由对应模块维护。

```text
Robot Mode
    → robot_mode

Gimbal State
    → gimbal_control

Shooter State
    → shooter_control

Chassis State
    → chassis_control

Motor Feedback
    → motor / CAN receive

IMU State
    → INS / IMU module

Vision State
    → vision_link

CAN RX Message
    → CAN message queue
```

这种组织方式的目的，是尽量让具体模块负责自己的运行状态，而不是由一个全局控制模块保存所有状态。

---

## 13. 安全机制

系统主要通过以下方式保证运行安全：

### IMU 安全门控

```text
IMU Ready?
   │
 ┌─┴─┐
 No  Yes
 │    │
 ▼    ▼
输出0  正常控制
```

### 遥控失联

遥控数据超过规定时间没有更新后，系统进入失能状态。

### 电机在线检测

根据电机反馈更新时间判断通信是否正常，并在超时后标记为离线。

### Pitch 限位

对 Pitch 目标角度进行限制，避免超出机械允许范围。

### 射击保护

射击系统包含卡弹检测和退弹逻辑，减少拨盘堵转时持续施加输出。



---

## 14. 工程特点

这个项目并不是从零设计的一套通用机器人软件框架，而是在 RoboMaster 实际比赛需求下逐步开发形成的嵌入式控制工程。

它的主要特点是：

- STM32 + FreeRTOS 实时控制
- 多路 CAN 电机通信
- IMU 姿态解算
- PID / SMC 控制
- 云台、射击、底盘协同
- 视觉通信
- 多任务并发
- 实车调试和参数验证

因此理解该工程时，重点不在于某一个算法文件，而在于：

**输入 → 状态 → 控制 → 通信 → 执行器 → 反馈**

这一整条机器人控制链路。
