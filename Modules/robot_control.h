/**
 * @file robot_control.h
 * @brief 机器人总调度接口
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "INS_task.h"
#include "robot_types.h"

/** @brief 初始化所有控制模块 */
void Robot_Param_Init(void);

/** @brief 机器人主控制循环 */
void Robot_Control_Loop(const IMU_Data_t* imu_data);

/** @brief 获取系统LED状态 */
LED_State_e Robot_Get_Led_State(void);

#endif
