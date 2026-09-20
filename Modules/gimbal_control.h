#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "INS_task.h"

//云台Pitch电机物理ID(反馈帧ID = 0x200 + ID)
#define GIMBAL_PITCH_MOTOR_ID 5

//云台输出快照：由 Robot_Control_Loop 统一读取后执行 CAN 发送
typedef struct
{
    float   yaw_current;    //Yaw目标电流(LK力矩电流, A)
    int16_t pitch_current;  //Pitch目标电流(DJI)
    float   yaw_degree;     //Yaw机械角度(度, 供UI)
    float   pitch_degree;   //Pitch相对水平角度(度, 供UI)
} Gimbal_Output_t;

//调试打印快照(供 Print_Entry_Task 使用)
typedef struct
{
    float yaw_ref_pos;        //Yaw SMC 目标角度(deg)
    float yaw_fdb_pos;        //Yaw SMC 反馈角度(deg)
    float pitch_ref_pos;      //Pitch SMC 目标角度(deg)
    float pitch_fdb_pos;      //Pitch SMC 反馈角度(deg)
    float yaw_motor_angle;    //Yaw 电机编码器原始值
    float pitch_motor_angle;  //Pitch 电机编码器原始值
} Gimbal_Debug_Data_t;

//初始化云台电机与控制器(Robot_Param_Init 中调用)
void Gimbal_Control_Init(void);

//机器人整体启动/关闭时的云台侧处理
void Gimbal_On_Robot_Start(const IMU_Data_t* imu);
void Gimbal_On_Robot_Off(void);

//云台控制模式切换(IMU/编码器)，内部处理边沿与重初始化
void Gimbal_Set_Mode(Gimbal_Mode_e mode, const IMU_Data_t* imu);

//是否执行闭环控制(非 OFF/CHASSIS_ONLY 时使能)
void Gimbal_Set_Active(bool active);

//自瞄/雷达辅吊参数切换(内部做边沿检测)
void Gimbal_Set_Aiming(bool aiming);
void Gimbal_Set_Base_Hitting(bool hitting);

//电机重新上线后的目标对齐
void Gimbal_Handle_Motor_Reconnect(const IMU_Data_t* imu);

//IMU首次就绪对齐
void Gimbal_Sync_On_First_Ready(const IMU_Data_t* imu);
bool Gimbal_Is_Initialized(void);

//RESET键：清除首次就绪标记/Yaw SMC复位
void Gimbal_Reset_Initialized(void);
void Gimbal_Reset_Yaw_SMC(void);

//将世界目标角同步到当前IMU角度
void Gimbal_Sync_World_Target_To_IMU(const IMU_Data_t* imu);

//目标角度接口(由主调度器翻译遥控/视觉指令后调用)
void  Gimbal_Set_World_Target(float yaw, float pitch);
void  Gimbal_Set_World_Yaw(float yaw);
float Gimbal_Get_World_Yaw(void);
float Gimbal_Get_World_Pitch(void);
void  Gimbal_Add_World_Target(float dyaw, float dpitch);
void  Gimbal_Clamp_World_Pitch(float min_deg, float max_deg);

void Gimbal_Add_Encoder_Target(float dyaw, float dpitch);
void Gimbal_Clamp_Encoder_Pitch(float min_val, float max_val);

//闭环控制计算(每个控制周期调用一次)
void Gimbal_Control_Update(const IMU_Data_t* imu);
const Gimbal_Output_t* Gimbal_Get_Output(void);

//获取调试打印快照(每次调用刷新)
const Gimbal_Debug_Data_t* Gimbal_Get_Debug_Data(void);

//Yaw编码器偏差(已做±半圈归一化, 供底盘跟随计算)
float Gimbal_Get_Yaw_Encoder_Diff(void);

//CAN反馈解码(由 Can_Decode_Task 分发调用)
void Gimbal_Update_Yaw(const uint8_t* data);
void Gimbal_Update_Pitch(const uint8_t* data);

//电机在线检测(供 Monitor_Task)
void Gimbal_Check_Motors_Online(uint32_t current_tick);

#endif
