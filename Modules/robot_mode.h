/**
 * @file robot_mode.h
 * @brief 机器人工作模式定义与模式选择接口
 *
 * Robot_Mode_Update 根据遥控来源与上坎状态决定当前模式并维护模式LED,
 */

#ifndef ROBOT_MODE_H
#define ROBOT_MODE_H

#include "robot_types.h"
#include "remote_control.h"

/** @brief 机器人工作模式 */
typedef enum
{
    ROBOT_MODE_OFF = 0,
    ROBOT_MODE_GIMBAL_ONLY,
    ROBOT_MODE_CHASSIS_ONLY,
    ROBOT_MODE_FOLLOW,
    ROBOT_MODE_SHOOT,
    ROBOT_MODE_SHOOT_LOW,
    ROBOT_MODE_AIMING,
    ROBOT_MODE_M_AIMING,
    ROBOT_MODE_CLIMB,
    ROBOT_MODE_BATTLE
} Robot_Mode_e;

/** @brief 云台控制模式(IMU闭环 / 编码器闭环) */
typedef enum
{
    GIMBAL_MODE_IMU,
    GIMBAL_MODE_ENCODER
} Gimbal_Mode_e;

/** @brief 上坎状态 */
typedef enum
{
    CLIMB_MODE_OFF,
    CLIMB_MODE_STANDBY,
    CLIMB_MODE_ACTIVE
} Climb_Mode_e;

/** @brief 战斗子状态 */
typedef enum
{
    BATTLE_OFF = 0,
    BATTLE_FOLLOW = 1,
    BATTLE_GIMBAL = 2,
    BATTLE_LOB    = 3
} Battle_Mode_e;

/** @brief 更新工作模式选择(每个控制周期调用一次) */
void Robot_Mode_Update(const RC_Data_t* rc, ControlSource_e control_source);

/** @brief 获取当前工作模式 */
Robot_Mode_e Robot_Mode_Get(void);

/** @brief 模式对应的LED指示(无控制源时保持上一次) */
LED_State_e Robot_Mode_Get_Led_State(void);

#endif
