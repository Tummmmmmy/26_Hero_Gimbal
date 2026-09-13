/**
 * @file chassis_control.h
 * @brief 底盘控制模块接口
 *
 * 英雄云台板上的底盘相关职责:
 * - 键盘/遥控指令 -> 底盘速度指令(vx/vy/wz)的计算(各模式规则、云台坐标变换、斜坡滤波)
 * - 底盘跟随PID(云台-底盘相对角度收敛)
 * - 上坎状态机与上坎/LED指令
 * - 双板通信(0x304)底盘反馈解码(颜色/裁判系统在线/裁判数据转发)
 * 计算结果写入输出快照, CAN 发送仍由 Robot_Control_Loop 统一调度。
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "remote_control.h"

//双板通信CAN ID(底盘 -> 云台)
#define RX_CHASSIS_DATA_ID 0x304

//常规底盘移动风格(对应各机器人模式)
typedef enum
{
    CHASSIS_MOVE_GIMBAL_ONLY,   //云台单开: 只平移, 不改自旋, 不请求超级电容
    CHASSIS_MOVE_FOLLOW,        //跟随: 拨轮小陀螺或底盘跟随PID
    CHASSIS_MOVE_AIMING,        //自瞄: 拨轮小陀螺(1.5倍速)
    CHASSIS_MOVE_M_AIMING       //手动自瞄: 只平移, 不改自旋
} Chassis_Move_Style_e;

//裸底盘模式(不做坐标变换)
typedef enum
{
    CHASSIS_DIRECT_AUTO_SUPERCAP,   //底盘单开: 左杆前推超阈值时请求超级电容
    CHASSIS_DIRECT_NO_SUPERCAP      //上坎: 不请求超级电容
} Chassis_Direct_Style_e;

//底盘输出快照：由 Robot_Control_Loop 统一读取后执行 CAN 发送
typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t wz;
    uint8_t mode;        //0:Disable, 1:Enable
    uint8_t supercap;    //超级电容请求
    uint8_t climb;       //上坎指令: 0全收 1前伸 2前收后伸
    uint8_t led;         //底盘LED
    uint8_t shift;       //小陀螺标志(键鼠模式按住SHIFT)
    uint8_t battle_mode; //战斗子状态(供底盘上报)
} Chassis_Output_t;

//初始化(底盘跟随PID等, Robot_Param_Init 中调用)
void Chassis_Control_Init(void);

//机器人整体启动/关闭时的底盘侧处理(PID限幅复位/输出清零)
void Chassis_On_Robot_Start(void);
void Chassis_On_Robot_Off(void);

//上坎状态机与上坎/LED指令更新(每个控制周期调用一次, 先于模式 switch)
void Chassis_Update_Climb(const RC_Data_t* rc, Robot_Mode_e robot_mode, bool e_key_just_pressed);

//战斗模式(键鼠)底盘计算: 目标速度/底盘锁定/斜坡滤波/坐标变换
void Chassis_Update_Battle(const RC_Data_t* rc, float yaw_encoder_diff, uint8_t is_aiming_now, uint8_t is_base_shooting);

//常规底盘移动计算(云台单开/跟随/自瞄/手动自瞄)
void Chassis_Update_Move(const RC_Data_t* rc, float yaw_encoder_diff, Chassis_Move_Style_e style);

//裸底盘计算(底盘单开/上坎, 不做坐标变换)
void Chassis_Set_Direct(const RC_Data_t* rc, Chassis_Direct_Style_e style);

//底盘指令清零(发射模式等不动底盘的场景)
void Chassis_Stop_Movement(void);

//上坎时清零横移/自旋(主循环 climb_cmd 后处理)
void Chassis_Zero_Turn(void);

//战斗子状态写入(供主循环翻译后写入底盘上报字段)
void Chassis_Set_Battle_Mode(Battle_Mode_e mode);

//上坎状态/指令查询(供主循环模式选择与云台限位判断)
Climb_Mode_e Chassis_Get_Climb_Mode(void);
uint8_t Chassis_Get_Climb_Cmd(void);

//获取输出快照(零拷贝只读)
const Chassis_Output_t* Chassis_Get_Output(void);

//CAN反馈解码(0x304 双板通信, 由 Can_Decode_Task 分发调用)
void Chassis_Decode_Feedback(const uint8_t* data);

//己方颜色(0=蓝, 1=红, 供视觉模块)
uint8_t Chassis_Get_Color(void);

#endif
