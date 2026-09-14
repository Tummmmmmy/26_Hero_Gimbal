/**
 * @file control_task.h
 * @brief 1ms 控制任务接口
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "main.h"

// 1ms 控制任务: 采集IMU数据包并调用 Robot_Control_Loop
void Gimbal_Control_task(void const * argument);

#endif
