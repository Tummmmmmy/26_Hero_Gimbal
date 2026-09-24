/**
 * @file vision_task.c
 * @brief 视觉通信任务
 *
 * 周期向视觉小电脑发送状态数据; 接收与在线判定在 vision_link 内部完成。
 */

#include "vision_task.h"
#include "vision_link.h"
#include "cmsis_os.h"
#include "INS_task.h"


void vision_task(void const * argument)
{
	osDelay(2000); 

	Vision_Init();

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

	for(;;)
	{

		  Vision_Send_Data();


			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
