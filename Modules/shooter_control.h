#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// 发射器工作模式
typedef enum {
    SHOOTER_MODE_OFF = 0,
    SHOOTER_MODE_LOW_SPEED,   // 12m/s
    SHOOTER_MODE_HIGH_SPEED   // 16m/s
} Shooter_Mode_e;

// 输入指令：由主调度器翻译好后下达
typedef struct {
    uint8_t fire_command;          // 开火指令: 1=开火, 0=停止
    Shooter_Mode_e fric_mode;      // 摩擦轮模式
    bool use_referee_heat_limit;   // 是否使用裁判系统热量门控(VTM)
} Shooter_Cmd_t;

// 输出快照：由 Robot_Control_Loop 统一读取后执行 CAN 发送
typedef struct {
    float dial_target_speed;
    int16_t friction_current_inner_left;
    int16_t friction_current_inner_right;
    int16_t friction_current_inner_up;
    int16_t friction_current_outer_left;
    int16_t friction_current_outer_right;
    int16_t friction_current_outer_up;
    uint8_t friction_ready;          // 摩擦轮是否已转起来(1=就绪)
    float friction_target_speed;     // 目标转速(供底盘UI)
    uint8_t dial_mode_cmd;           // 拨盘使能/失能: 0=无, 0xFC=使能, 0xFD=失能
} Shooter_Output_t;

// 调试/调参打印快照
typedef struct {
    float referee_bullet_speed;
    float print_initial_speed;
    float bullet_speed_avg;
    float bullet_speed_variance;
    float bullet_speed_range;
    float friction_outer_left_speed;
    float friction_outer_right_speed;
    float friction_outer_up_speed;
    float friction_inner_left_speed;
    float friction_inner_right_speed;
    float friction_inner_up_speed;
} Shooter_Debug_Data_t;

// 初始化电机与射击状态
void Shooter_Control_Init(void);

// 射击控制更新：传入指令包，计算输出快照，不直接发 CAN
void Shooter_Control_Update(const Shooter_Cmd_t *cmd);

// 急停/模式失能
void Shooter_Control_Disable(void);

// 机器人整体启动/关闭时调用
void Shooter_On_Robot_Start(void);
void Shooter_On_Robot_Off(void);

// 获取输出快照(零拷贝只读)
const Shooter_Output_t* Shooter_Get_Output(void);

// CAN 反馈路由：把拨盘/摩擦轮反馈帧分发到 Shooter 内部
void Shooter_CAN_Feedback(uint32_t can_id, const uint8_t* data);

// 裁判系统数据注入(由底盘双板通信解码后调用)
void Shooter_Set_Referee_Data(uint16_t heat, uint16_t limit, uint16_t cooling, float speed);

// 重置弹速统计与自适应补偿状态
void Shooter_Reset_Stats(void);

// 系统状态注入：避免 Shooter 直接依赖 robot_mode/remote_control
void Shooter_Set_System_State(bool robot_off, bool aiming_mode, bool battle_aiming, bool wheel_released);

// 电机在线检测，供 Monitor_Task 使用
void Shooter_Check_Motors_Online(uint32_t current_tick);

// 调试打印周期处理
void Shooter_Debug_Tick(void);
const Shooter_Debug_Data_t* Shooter_Get_Debug_Data(void);

// 摩擦轮目标转速微调
void Shooter_Tune_High_Speed(int32_t delta);
void Shooter_Tune_Low_Speed(int32_t delta);

#endif
