/**
 * @file INS_task.h
 * @brief IMU 姿态解算任务接口与 IMU_Data_t 定义
 */

#ifndef INS_TASK__H
#define INS_TASK__H

#include "main.h"

typedef struct
{
    float yaw_angle;
    float pitch_angle;
    float yaw_gyro;
    float pitch_gyro;
} IMU_Data_t;

extern float total_yaw;
extern float pitch;
extern uint8_t attitude_flag;
extern volatile uint8_t imu_ready_flag; //0:未就绪, 1:已就绪
extern float gyro[3];

void IMU_task(void const * argument);

void Reset_All_IMU_Yaw_Only(void);
#endif
