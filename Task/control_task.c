/**
 * @file control_task.c
 * @brief 1ms 控制任务(Gimbal_Control_task)
 *
 * 周期采集 IMU 数据打包, 调用 Robot_Control_Loop 完成整机控制。
 */

#include "control_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "robot_control.h"
#include <string.h>

void Gimbal_Control_task(void const * argument)
{
	
	IMU_Data_t imu_data_pack = {0};
 
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
	
  for(;;)
  {
		
				imu_data_pack.yaw_angle = total_yaw; 
				imu_data_pack.yaw_gyro  = gyro[2]; 
				imu_data_pack.pitch_angle = pitch;
				imu_data_pack.pitch_gyro = gyro[0];

		
				Robot_Control_Loop(&imu_data_pack);
		
	    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}
