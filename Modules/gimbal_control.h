/**
 * @file gimbal_control.h
 * @brief 云台控制接口：目标角管理、闭环控制与电机反馈处理
 *
 * 本模块只产出控制量, CAN 发送由 Robot_Control_Loop 统一调度。
 */

#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "INS_task.h"

//云台Pitch电机物理ID(反馈帧ID = 0x200 + ID)
#define GIMBAL_PITCH_MOTOR_ID 5

/** @brief 云台输出快照(只读) */
typedef struct
{
    float   yaw_current;    //Yaw目标电流(LK力矩电流, A)
    int16_t pitch_current;  //Pitch目标电流(DJI)
    float   yaw_degree;     //Yaw机械角度(度, 供UI)
    float   pitch_degree;   //Pitch相对水平角度(度, 供UI)
} Gimbal_Output_t;

/** @brief 调试打印快照 */
typedef struct
{
    float yaw_ref_pos;        //Yaw SMC 目标角度(deg)
    float yaw_fdb_pos;        //Yaw SMC 反馈角度(deg)
    float pitch_ref_pos;      //Pitch SMC 目标角度(deg)
    float pitch_fdb_pos;      //Pitch SMC 反馈角度(deg)
    float yaw_motor_angle;    //Yaw 电机编码器原始值
    float pitch_motor_angle;  //Pitch 电机编码器原始值
} Gimbal_Debug_Data_t;

// ===== 初始化与启停 =====

/** @brief 初始化云台电机与控制器 */
void Gimbal_Control_Init(void);

/** @brief 机器人启动时的云台侧处理(SMC 重初始化并令目标角对齐当前姿态) */
void Gimbal_On_Robot_Start(const IMU_Data_t* imu);

/** @brief 机器人失能时的云台侧处理(闭环积分与输出清零) */
void Gimbal_On_Robot_Off(void);

// ===== 模式与运行状态 =====

/** @brief 云台控制模式切换(IMU/编码器)，内部处理边沿与重初始化 */
void Gimbal_Set_Mode(Gimbal_Mode_e mode, const IMU_Data_t* imu);

/** @brief 是否执行闭环控制(非 OFF/CHASSIS_ONLY 时使能) */
void Gimbal_Set_Active(bool active);

/** @brief 自瞄参数切换(内部做边沿检测) */
void Gimbal_Set_Aiming(bool aiming);

/** @brief 雷达辅吊参数切换(内部做边沿检测) */
void Gimbal_Set_Base_Hitting(bool hitting);

/** @brief 电机重新上线后的目标对齐 */
void Gimbal_Handle_Motor_Reconnect(const IMU_Data_t* imu);

// ===== 首次就绪与复位 =====

/** @brief IMU首次就绪对齐 */
void Gimbal_Sync_On_First_Ready(const IMU_Data_t* imu);

/** @brief 首次就绪对齐是否已完成 */
bool Gimbal_Is_Initialized(void);

/** @brief RESET键：清除首次就绪标记 */
void Gimbal_Reset_Initialized(void);

/** @brief RESET键：Yaw SMC 复位 */
void Gimbal_Reset_Yaw_SMC(void);

/** @brief 将世界目标角同步到当前IMU角度 */
void Gimbal_Sync_World_Target_To_IMU(const IMU_Data_t* imu);

// ===== 目标角度接口 =====

/** @brief 设置世界坐标系目标角 */
void  Gimbal_Set_World_Target(float yaw, float pitch);

/** @brief 只更新世界坐标系 Yaw 目标角 */
void  Gimbal_Set_World_Yaw(float yaw);

/** @brief 获取世界坐标系 Yaw 目标角 */
float Gimbal_Get_World_Yaw(void);

/** @brief 获取世界坐标系 Pitch 目标角 */
float Gimbal_Get_World_Pitch(void);

/** @brief 在世界坐标系目标角上叠加增量 */
void  Gimbal_Add_World_Target(float dyaw, float dpitch);

/** @brief 世界坐标系 Pitch 目标角限幅 */
void  Gimbal_Clamp_World_Pitch(float min_deg, float max_deg);

/** @brief 在编码器坐标系目标角上叠加增量 */
void Gimbal_Add_Encoder_Target(float dyaw, float dpitch);

/** @brief 编码器坐标系 Pitch 目标角限幅 */
void Gimbal_Clamp_Encoder_Pitch(float min_val, float max_val);

// ===== 闭环计算与数据读取 =====

/** @brief 闭环控制计算(每个控制周期调用一次) */
void Gimbal_Control_Update(const IMU_Data_t* imu);

/** @brief 获取输出快照(零拷贝只读) */
const Gimbal_Output_t* Gimbal_Get_Output(void);

/** @brief 获取调试打印快照(每次调用刷新) */
const Gimbal_Debug_Data_t* Gimbal_Get_Debug_Data(void);

/** @brief Yaw编码器偏差(已做±半圈归一化) */
float Gimbal_Get_Yaw_Encoder_Diff(void);

// ===== CAN 反馈解码与在线检测 =====

/** @brief 解码 Yaw 电机的 CAN 反馈帧 */
void Gimbal_Update_Yaw(const uint8_t* data);

/** @brief 解码 Pitch 电机的 CAN 反馈帧 */
void Gimbal_Update_Pitch(const uint8_t* data);

/** @brief 检查云台电机在线状态 */
void Gimbal_Check_Motors_Online(uint32_t current_tick);

#endif
