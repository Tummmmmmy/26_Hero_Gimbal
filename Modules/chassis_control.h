/**
 * @file chassis_control.h
 * @brief 底盘控制模块接口
 *
 */


#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_mode.h"
#include "remote_control.h"


//双板通信CAN ID(底盘 -> 云台)
#define RX_CHASSIS_DATA_ID 0x304


/** @brief 底盘移动风格 */
typedef enum
{
    CHASSIS_MOVE_GIMBAL_ONLY,
    CHASSIS_MOVE_FOLLOW,
    CHASSIS_MOVE_AIMING,
    CHASSIS_MOVE_M_AIMING
} Chassis_Move_Style_e;



/** @brief 裸底盘模式 */
typedef enum
{
    CHASSIS_DIRECT_AUTO_SUPERCAP,
    CHASSIS_DIRECT_NO_SUPERCAP
} Chassis_Direct_Style_e;



/** @brief 底盘输出信息 */
typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t wz;
    uint8_t mode;        //0:Disable, 1:Enable
    uint8_t supercap;    //超级电容请求
    uint8_t climb;       //上坎指令: 0全收 1前伸 2前收后伸
    uint8_t led;         //LED
    uint8_t shift;       //小陀螺标志(SHIFT)
    uint8_t battle_mode; //键鼠子状态
} Chassis_Output_t;


// ===== 初始化与启停 =====

/** @brief 初始化底盘 */
void Chassis_Control_Init(void);

/** @brief 底盘使能 */
void Chassis_Enable(void);

/** @brief 底盘失能 */
void Chassis_Disable(void);



// ===== 底盘计算 =====

/** @brief 更新底盘上坎状态 */
void Chassis_Update_Climb(const RC_Data_t* rc, Robot_Mode_e robot_mode, bool e_key_just_pressed);

/** @brief 键鼠模式底盘计算 */
void Chassis_Update_Battle(const RC_Data_t* rc, float yaw_encoder_diff, uint8_t is_aiming_now, uint8_t is_base_shooting);

/** @brief 常规底盘移动计算 */
void Chassis_Update_Move(const RC_Data_t* rc, float yaw_encoder_diff, Chassis_Move_Style_e style);

/** @brief 裸底盘计算 */
void Chassis_Set_Direct(const RC_Data_t* rc, Chassis_Direct_Style_e style);

/** @brief 底盘锁定 */
void Chassis_Stop_Movement(void);

/** @brief 上坎时清零横移/自旋 */
void Chassis_Zero_Turn(void);

/** @brief 键鼠模式子状态写入 */
void Chassis_Set_Battle_Mode(Battle_Mode_e mode);



// ===== 状态查询与反馈解码 =====

/** @brief 获取当前上坎状态 */
Climb_Mode_e Chassis_Get_Climb_Mode(void);

/** @brief 获取当前上坎指令 */
uint8_t Chassis_Get_Climb_Cmd(void);

/** @brief 获取底盘输出 */
const Chassis_Output_t* Chassis_Get_Output(void);

/** @brief 解码底盘发来的双板通信反馈帧 */
void Chassis_Decode_Feedback(const uint8_t* data);

/** @brief 获得机器人颜色(0=蓝, 1=红) */
uint8_t Chassis_Get_Color(void);

#endif
