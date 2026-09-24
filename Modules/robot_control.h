/**
 * @file robot_control.h
 * @brief 机器人总调度接口
 *
 * Robot_Control_Loop() 为 1ms 控制循环, 顺序为:
 *   输入 -> 安全/模式 -> 复位 -> 云台 -> 底盘 -> 射击 -> 统一CAN输出
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "INS_task.h"
#include "robot_types.h"

/** @brief 上电初始化所有控制模块 */
void Robot_Param_Init(void);

/** @brief 1ms 控制主循环 */
void Robot_Control_Loop(const IMU_Data_t* imu_data);

/** @brief 获取系统LED状态 */
LED_State_e Robot_Get_Led_State(void);

#endif
