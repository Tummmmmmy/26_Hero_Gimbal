/**
 * @file gimbal_control.h
 * @brief 云台控制接口
 * */

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "INS_task.h"


#define GIMBAL_PITCH_MOTOR_ID 5


/** @brief 云台输出*/
typedef struct
{
    float   yaw_current;
    int16_t pitch_current;
    float   yaw_degree;
    float   pitch_degree;
} Gimbal_Output_t;


/** @brief 调试打印信息 */
typedef struct
{
    float yaw_ref_pos;
    float yaw_fdb_pos;
    float pitch_ref_pos;
    float pitch_fdb_pos;
    float yaw_motor_angle;
    float pitch_motor_angle;
} Gimbal_Debug_Data_t;



// ===== 初始化与启停 =====

/** @brief 初始化云台 */
void Gimbal_Control_Init(void);

/** @brief 云台使能 */
void Gimbal_Enable(const IMU_Data_t* imu);

/** @brief 云台失能 */
void Gimbal_Disable(void);



// ===== 模式与运行状态 =====

/** @brief 云台控制模式切换 */
void Gimbal_Set_Mode(Gimbal_Mode_e mode, const IMU_Data_t* imu);

/** @brief 是否执行云台闭环控制 */
void Gimbal_Set_Active(bool active);

/** @brief 自瞄参数切换 */
void Gimbal_Set_Aiming(bool aiming);

/** @brief 雷达辅瞄参数切换 */
void Gimbal_Set_Base_Hitting(bool hitting);

/** @brief 电机重新上线目标对齐 */
void Gimbal_Handle_Motor_Reconnect(const IMU_Data_t* imu);



// ===== 首次就绪与复位 =====

/** @brief IMU首次就绪对齐云台目标 */
void Gimbal_First_Ready(const IMU_Data_t* imu);

/** @brief 首次对齐是否已完成 */
bool Gimbal_Is_Initialized(void);

/** @brief 清除首次就绪标记 */
void Gimbal_Reset_Initialized(void);

/** @brief Yaw SMC 复位 */
void Gimbal_Reset_Yaw_SMC(void);

/** @brief 将世界目标同步为当前IMU角度 */
void Gimbal_World_Target_To_IMU(const IMU_Data_t* imu);



// ===== 目标角度接口 =====

/** @brief 设置目标角度 */
void  Gimbal_Set_World_Target(float yaw, float pitch);

/** @brief 设置Yaw目标角度 */
void  Gimbal_Set_World_Yaw(float yaw);

/** @brief 获取Yaw目标角度 */
float Gimbal_Get_World_Yaw(void);

/** @brief 获取Pitch目标角度 */
float Gimbal_Get_World_Pitch(void);

/** @brief 目标角叠加增量 */
void  Gimbal_Add_World_Target(float dyaw, float dpitch);

/** @brief Pitch目标角限幅 */
void  Gimbal_Clamp_World_Pitch(float min_deg, float max_deg);

/** @brief 编码器目标叠加增量 */
void Gimbal_Add_Encoder_Target(float dyaw, float dpitch);

/** @brief 编码器Pitch限幅 */
void Gimbal_Clamp_Encoder_Pitch(float min_val, float max_val);

// ===== 闭环计算与数据读取 =====

/** @brief 云台闭环控制计算 */
void Gimbal_Control_Update(const IMU_Data_t* imu);

/** @brief 获取云台输出 */
const Gimbal_Output_t* Gimbal_Get_Output(void);

/** @brief 获取调试打印信息 */
const Gimbal_Debug_Data_t* Gimbal_Get_Debug_Data(void);

/** @brief Yaw编码器偏差 */
float Gimbal_Get_Yaw_Encoder_Diff(void);



// ===== CAN 反馈解码与在线检测 =====

/** @brief 解码Yaw电机的CAN反馈帧 */
void Gimbal_Update_Yaw(const uint8_t* data);

/** @brief 解码Pitch电机的CAN反馈帧 */
void Gimbal_Update_Pitch(const uint8_t* data);

/** @brief 检查云台电机在线状态 */
void Gimbal_Check_Motors_Online(uint32_t current_tick);

#endif
