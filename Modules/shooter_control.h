/**
 * @file shooter_control.h
 * @brief 发射控制接口：摩擦轮、拨盘、弹速统计与热量管理
 *
 * 本模块只产出控制量, CAN 发送由 Robot_Control_Loop 统一调度。
 */

#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"     // Robot_Mode_e / Gimbal_Mode_e
#include "remote_control.h"  // RC_Data_t / ControlSource_e

/** @brief 发射器工作模式(12m/s / 16m/s) */
typedef enum {
    SHOOTER_MODE_OFF = 0,
    SHOOTER_MODE_LOW_SPEED,   // 12m/s
    SHOOTER_MODE_HIGH_SPEED   // 16m/s
} Shooter_Mode_e;

/** @brief 发射决策所需的系统事实(上层每周期填入) */
typedef struct {
    const RC_Data_t* rc;           // 遥控数据
    Robot_Mode_e robot_mode;       // 当前机器人模式
    Gimbal_Mode_e gimbal_mode;     // 当前云台模式
    uint8_t is_aiming_now;         // 是否处于瞄准中(自瞄或手动自瞄)
    ControlSource_e control_source;// 当前控制源(DR16/VTM)
} Shooter_Context_t;

/** @brief 发射输出快照(只读) */
typedef struct {
    float dial_target_speed;
    int16_t friction_current_inner_left;
    int16_t friction_current_inner_right;
    int16_t friction_current_inner_up;
    int16_t friction_current_outer_left;
    int16_t friction_current_outer_right;
    int16_t friction_current_outer_up;
    uint8_t friction_ready;          // 摩擦轮是否已转起来(1=就绪)
    Shooter_Mode_e fric_mode;        // 当前摩擦轮模式
    uint8_t friction_report;         // 底盘状态上报值: 1/2(0=未就绪)
    float friction_target_speed;     // 目标转速(供底盘UI)
    uint8_t dial_mode_cmd;           // 拨盘使能/失能: 0=无, 0xFC=使能, 0xFD=失能
} Shooter_Output_t;

/** @brief 调试/调参打印快照 */
typedef struct {
    float referee_bullet_speed;
    float bullet_speed_avg;
    float bullet_speed_variance;
    float bullet_speed_range;
    float friction_outer_left_speed;
    float friction_outer_right_speed;
    float friction_outer_up_speed;
    float friction_inner_left_speed;
    float friction_inner_right_speed;
    float friction_inner_up_speed;
    float dial_total_angle;      //拨盘当前总角度(rad)
    float dial_target_rad;       //拨盘目标角度(rad)
    float dial_angle_target;     //拨盘角度环目标
    float dial_angle_measured;   //拨盘角度环反馈
    float dial_speed_target;     //拨盘速度环目标
    float dial_speed_measured;   //拨盘速度环反馈
    float dial_torque;           //拨盘力矩反馈
    float dial_velocity;         //拨盘速度反馈
    float shot_count;            //累计发弹数
    float predict_heat;          //预测热量
} Shooter_Debug_Data_t;

// ===== 生命周期与主更新 =====

/** @brief 初始化电机与射击状态 */
void Shooter_Control_Init(void);

/** @brief 射击控制更新：输入系统上下文, 输出控制快照(不直接发 CAN) */
void Shooter_Control_Update(const Shooter_Context_t *ctx);

/** @brief 急停/模式失能 */
void Shooter_Control_Disable(void);

/** @brief 机器人整体启动时调用 */
void Shooter_On_Robot_Start(void);

/** @brief 机器人整体关闭(失能)时调用 */
void Shooter_On_Robot_Off(void);

// ===== 数据注入与反馈 =====

/** @brief 获取输出快照(零拷贝只读) */
const Shooter_Output_t* Shooter_Get_Output(void);

/** @brief CAN 反馈路由：把拨盘/摩擦轮反馈帧分发到 Shooter 内部 */
void Shooter_CAN_Feedback(uint32_t can_id, const uint8_t* data);

/** @brief 注入裁判系统数据 */
void Shooter_Set_Referee_Data(uint16_t heat, uint16_t limit, uint16_t cooling, float speed);

/** @brief 重置弹速统计与自适应补偿状态 */
void Shooter_Reset_Stats(void);


/** @brief 检查发射机构电机在线状态 */
void Shooter_Check_Motors_Online(uint32_t current_tick);

// ===== 调试与调参 =====

/** @brief 调试打印周期处理 */
const Shooter_Debug_Data_t* Shooter_Get_Debug_Data(void);

/** @brief 高速档摩擦轮目标转速微调 */
void Shooter_Tune_High_Speed(int32_t delta);

/** @brief 低速档摩擦轮目标转速微调 */
void Shooter_Tune_Low_Speed(int32_t delta);

#endif
