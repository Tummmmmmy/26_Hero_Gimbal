/**
 * @file shooter_control.h
 * @brief 发射控制接口
 *
 */

#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "remote_control.h"


//拨盘电机反馈帧ID
#define RX_DIAL_FDB_ID 0x302


/** @brief 发射模式 */
typedef enum {
    SHOOTER_MODE_OFF = 0,
    SHOOTER_MODE_LOW_SPEED,   // 12m/s
    SHOOTER_MODE_HIGH_SPEED   // 16m/s
} Shooter_Mode_e;


/** @brief 发射模块所需的数据 */
typedef struct {
    const RC_Data_t* rc;
    Robot_Mode_e robot_mode;
    Gimbal_Mode_e gimbal_mode;
    uint8_t is_aiming_now;
    ControlSource_e control_source;
} Shooter_Context_t;


/** @brief 发射输出 */
typedef struct {
    float dial_target_speed;
    int16_t friction_current_inner_left;
    int16_t friction_current_inner_right;
    int16_t friction_current_inner_up;
    int16_t friction_current_outer_left;
    int16_t friction_current_outer_right;
    int16_t friction_current_outer_up;
    uint8_t friction_ready;
    Shooter_Mode_e fric_mode;
    uint8_t friction_report;
    float friction_target_speed;
    uint8_t dial_mode_cmd;
} Shooter_Output_t;



/** @brief 调试打印数据 */
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
    float dial_total_angle;
    float dial_target_rad;
    float dial_angle_target;
    float dial_angle_measured;
    float dial_speed_target;
    float dial_speed_measured;
    float dial_torque;
    float dial_velocity;
    float shot_count;
    float predict_heat;
} Shooter_Debug_Data_t;




// ===== 初始化与启停 =====

/** @brief 初始化发射 */
void Shooter_Control_Init(void);

/** @brief 射击控制更新 */
void Shooter_Control_Update(const Shooter_Context_t *ctx);

/** @brief 关闭发射 */
void Shooter_Control_Disable(void);

/** @brief 发射使能 */
void Shooter_Enable(void);

/** @brief 发射失能 */
void Shooter_Disable(void);



// ===== 数据注入与反馈 =====

/** @brief 获取发射输出 */
const Shooter_Output_t* Shooter_Get_Output(void);

/** @brief CAN 反馈路由 */
void Shooter_CAN_Feedback(uint32_t can_id, const uint8_t* data);

/** @brief 获得裁判系统数据 */
void Shooter_Set_Referee_Data(uint16_t heat, uint16_t limit, uint16_t cooling, float speed);

/** @brief 重置弹速统计 */
void Shooter_Reset_Stats(void);

/** @brief 检查发射机构电机在线状态 */
void Shooter_Check_Motors_Online(uint32_t current_tick);




// ===== 调试与调参 =====

/** @brief 获取调试打印信息 */
const Shooter_Debug_Data_t* Shooter_Get_Debug_Data(void);

/** @brief 高速档摩擦轮目标转速微调 */
void Shooter_Tune_High_Speed(int32_t delta);

/** @brief 低速档摩擦轮目标转速微调 */
void Shooter_Tune_Low_Speed(int32_t delta);

#endif
